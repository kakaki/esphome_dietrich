#include "dietrich.h"
#include <cmath>
#include <cstring>
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dietrich {

static const char *const TAG = "dietrich";

static const uint8_t CMD_SAMPLE_MCR3[10] = {0x02, 0xFE, 0x01, 0x05, 0x08, 0x02, 0x01, 0x69, 0xAB, 0x03};
static const uint8_t CMD_COUNTER1_MCR3[10] = {0x02, 0xFE, 0x00, 0x05, 0x08, 0x10, 0x1C, 0x98, 0xC2, 0x03};
static const uint8_t CMD_COUNTER2_MCR3[10] = {0x02, 0xFE, 0x00, 0x05, 0x08, 0x10, 0x1D, 0x59, 0x02, 0x03};

static const uint8_t CMD_SAMPLE_CALENTA[8] = {0x02, 0x52, 0x05, 0x06, 0x02, 0x00, 0x53, 0x03};
static const uint8_t CMD_COUNTER1_CALENTA[8] = {0x02, 0x52, 0x05, 0x06, 0x10, 0x01, 0x40, 0x03};
static const uint8_t CMD_COUNTER2_CALENTA[8] = {0x02, 0x52, 0x05, 0x06, 0x10, 0x02, 0x43, 0x03};

float Dietrich::signed_float_(float value) {
  if (value > 32768)
    value -= 65536;
  return value;
}

// A disconnected (open) temperature sensor input reads as 0x8000, which would
// scale to 327.68 degC - publish NAN instead so Home Assistant shows "unknown"
float Dietrich::temp_or_nan_(uint16_t raw) {
  if (raw == 0x8000)
    return NAN;
  return signed_float_(raw) * 0.01f;
}

std::string Dietrich::hex_str_(const uint8_t *data, size_t len) {
  std::string s;
  s.reserve(len * 2);
  static const char HEX_CHARS[] = "0123456789ABCDEF";
  for (size_t i = 0; i < len; i++) {
    s += HEX_CHARS[(data[i] >> 4) & 0x0F];
    s += HEX_CHARS[data[i] & 0x0F];
  }
  return s;
}

// Values are not published straight away: they are queued and published one
// per loop() pass, so a full frame (up to ~60 sensors) never blocks the main
// loop and the API/WiFi stack (and other components) keep running.
void Dietrich::publish_(sensor::Sensor *s, float value) {
  if (s == nullptr)
    return;
  this->publish_queue_.emplace_back(s, value);
}

// CRC16 (poly 0xA001, init 0xFFFF) over bytes 1..n-4; the frame ends with CRC (LSB, MSB) and 0x03
bool Dietrich::is_valid_crc_(const uint8_t *response, size_t n) {
  if (n < 3)
    return false;

  uint16_t expected_crc = response[n - 3] | (response[n - 2] << 8);

  uint16_t calculated_crc = 0xFFFF;
  for (size_t i = 1; i < n - 3; i++) {
    calculated_crc ^= response[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((calculated_crc & 0x0001) != 0) {
        calculated_crc >>= 1;
        calculated_crc ^= 0xA001;
      } else {
        calculated_crc >>= 1;
      }
    }
  }

  return expected_crc == calculated_crc;
}

// mcr3 frames carry a CRC16; calenta_v1_p5 frames use an XOR checksum instead,
// so for that variant we validate the response header (0x02 0x41 0x06)
bool Dietrich::frame_valid_(const uint8_t *response, size_t n) const {
  if (this->variant_ == DIETRICH_VARIANT_CALENTA_V1_P5)
    return n >= 3 && response[0] == 2 && response[1] == 65 && response[2] == 6;
  return is_valid_crc_(response, n);
}

void Dietrich::start_request_(Request request) {
  const bool calenta = this->variant_ == DIETRICH_VARIANT_CALENTA_V1_P5;
  const uint8_t *cmd;
  size_t cmd_len;
  switch (request) {
    case REQUEST_COUNTER1:
      cmd = calenta ? CMD_COUNTER1_CALENTA : CMD_COUNTER1_MCR3;
      cmd_len = calenta ? sizeof(CMD_COUNTER1_CALENTA) : sizeof(CMD_COUNTER1_MCR3);
      break;
    case REQUEST_COUNTER2:
      cmd = calenta ? CMD_COUNTER2_CALENTA : CMD_COUNTER2_MCR3;
      cmd_len = calenta ? sizeof(CMD_COUNTER2_CALENTA) : sizeof(CMD_COUNTER2_MCR3);
      break;
    case REQUEST_SAMPLE:
    default:
      cmd = calenta ? CMD_SAMPLE_CALENTA : CMD_SAMPLE_MCR3;
      cmd_len = calenta ? sizeof(CMD_SAMPLE_CALENTA) : sizeof(CMD_SAMPLE_MCR3);
      break;
  }

  // drop stale bytes (e.g. the tail of a reply that arrived after a timeout)
  while (this->available())
    this->read();

  memset(this->rx_buffer_, 0, sizeof(this->rx_buffer_));
  this->rx_len_ = 0;
  this->write_array(cmd, cmd_len);
  this->request_ = request;
  this->request_start_ = millis();
  this->last_rx_ = this->request_start_;
}

// Called from loop(): collect whatever bytes arrived since the last pass and
// decide whether the reply is complete. Never waits.
bool Dietrich::reply_complete_() {
  static const uint32_t RESPONSE_TIMEOUT_MS = 1000;
  static const uint32_t INTER_BYTE_GAP_MS = 50;

  const uint32_t now = millis();
  bool got = false;
  while (this->available() && this->rx_len_ < sizeof(this->rx_buffer_)) {
    this->rx_buffer_[this->rx_len_++] = this->read();
    got = true;
  }

  if (this->rx_len_ >= sizeof(this->rx_buffer_))
    return true;
  if (got) {
    this->last_rx_ = now;
    // mcr3 frames carry a CRC, so a valid one is known to be complete
    return this->variant_ == DIETRICH_VARIANT_MCR3 && this->rx_buffer_[this->rx_len_ - 1] == 0x03 &&
           this->frame_valid_(this->rx_buffer_, this->rx_len_);
  }
  if (this->rx_len_ > 0 && now - this->last_rx_ > INTER_BYTE_GAP_MS)
    return true;  // line went quiet: reply finished
  return now - this->request_start_ > RESPONSE_TIMEOUT_MS;
}

void Dietrich::handle_sample_(const uint8_t *readdata, size_t n) {
  // calenta_v1_p5 sample data (up to sub_state) is shifted one byte towards the frame start
  const int o = this->variant_ == DIETRICH_VARIANT_CALENTA_V1_P5 ? -1 : 0;

  if (this->frame_valid_(readdata, n)) {
    uint8_t bits = 0;

    this->publish_(this->flow_temp_sensor_, temp_or_nan_((readdata[8 + o] * 256) + readdata[7 + o]));
    this->publish_(this->return_temp_sensor_, temp_or_nan_((readdata[10 + o] * 256) + readdata[9 + o]));
    this->publish_(this->dhw_in_temp_sensor_, temp_or_nan_((readdata[12 + o] * 256) + readdata[11 + o]));
    this->publish_(this->outside_temp_sensor_, temp_or_nan_((readdata[14 + o] * 256) + readdata[13 + o]));
    this->publish_(this->calorifier_temp_sensor_, temp_or_nan_((readdata[16 + o] * 256) + readdata[15 + o]));
    this->publish_(this->boiler_control_temp_sensor_, temp_or_nan_((readdata[20 + o] * 256) + readdata[19 + o]));
    this->publish_(this->room_temp_sensor_, temp_or_nan_((readdata[22 + o] * 256) + readdata[21 + o]));
    this->publish_(this->ch_setpoint_sensor_, signed_float_((readdata[24 + o] * 256) + readdata[23 + o]) * 0.01f);
    this->publish_(this->dhw_setpoint_sensor_, signed_float_((readdata[26 + o] * 256) + readdata[25 + o]) * 0.01f);
    this->publish_(this->room_temp_setpoint_sensor_, signed_float_((readdata[28 + o] * 256) + readdata[27 + o]) * 0.01f);

    if (this->read_all_) {
      this->publish_(this->fan_speed_setpoint_sensor_, signed_float_((readdata[30 + o] * 256) + readdata[29 + o]));
      this->publish_(this->fan_speed_sensor_, signed_float_((readdata[32 + o] * 256) + readdata[31 + o]));
      this->publish_(this->ionisation_current_sensor_, readdata[33 + o] * 0.1f);
      this->publish_(this->internal_setpoint_sensor_, signed_float_((readdata[35 + o] * 256) + readdata[34 + o]) * 0.01f);
      this->publish_(this->available_power_sensor_, readdata[36 + o]);
      this->publish_(this->pump_percentage_sensor_, readdata[37 + o]);
      this->publish_(this->desired_max_power_sensor_, readdata[39 + o]);
      this->publish_(this->actual_power_sensor_, readdata[40 + o]);

      bits = readdata[43 + o];
      this->publish_(this->demand_source_bit0_sensor_, (bits >> 0) & 1);
      this->publish_(this->demand_source_bit1_sensor_, (bits >> 1) & 1);
      this->publish_(this->demand_source_bit2_sensor_, (bits >> 2) & 1);
      this->publish_(this->demand_source_bit3_sensor_, (bits >> 3) & 1);
      this->publish_(this->demand_source_bit4_sensor_, (bits >> 4) & 1);
      this->publish_(this->demand_source_bit5_sensor_, (bits >> 5) & 1);
      this->publish_(this->demand_source_bit6_sensor_, (bits >> 6) & 1);
      this->publish_(this->demand_source_bit7_sensor_, (bits >> 7) & 1);

      bits = readdata[44 + o];
      this->publish_(this->input_bit0_sensor_, (bits >> 0) & 1);
      this->publish_(this->input_bit1_sensor_, (bits >> 1) & 1);
      this->publish_(this->input_bit2_sensor_, (bits >> 2) & 1);
      this->publish_(this->input_bit3_sensor_, (bits >> 3) & 1);
      this->publish_(this->input_bit5_sensor_, (bits >> 5) & 1);
      this->publish_(this->input_bit6_sensor_, (bits >> 6) & 1);
      this->publish_(this->input_bit7_sensor_, (bits >> 7) & 1);

      bits = readdata[45 + o];
      this->publish_(this->valve_bit0_sensor_, (bits >> 0) & 1);
      this->publish_(this->valve_bit2_sensor_, (bits >> 2) & 1);
      this->publish_(this->valve_bit3_sensor_, (bits >> 3) & 1);
      this->publish_(this->valve_bit4_sensor_, (bits >> 4) & 1);
      this->publish_(this->valve_bit6_sensor_, (bits >> 6) & 1);

      bits = readdata[46 + o];
      this->publish_(this->pump_bit0_sensor_, (bits >> 0) & 1);
      this->publish_(this->pump_bit1_sensor_, (bits >> 1) & 1);
      this->publish_(this->pump_bit2_sensor_, (bits >> 2) & 1);
      this->publish_(this->pump_bit4_sensor_, (bits >> 4) & 1);
      this->publish_(this->pump_bit7_sensor_, (bits >> 7) & 1);
    }

    this->publish_(this->state_sensor_, readdata[47 + o]);
    this->publish_(this->lockout_sensor_, readdata[48 + o]);
    this->publish_(this->blocking_sensor_, readdata[49 + o]);
    this->publish_(this->sub_state_sensor_, readdata[50 + o]);

    if (this->read_all_) {
      this->publish_(this->hydro_pressure_sensor_, readdata[56] * 0.1f);

      bits = readdata[57];
      this->publish_(this->hru_sensor_, (bits >> 1) & 1);

      this->publish_(this->control_temp_sensor_, temp_or_nan_((readdata[59] * 256) + readdata[58]));
      this->publish_(this->dhw_flowrate_sensor_, signed_float_((readdata[61] * 256) + readdata[60]) * 0.01f);
    }

    this->read_all_ = !this->read_all_;
  } else if (n == 0) {
    ESP_LOGW(TAG, "no response from boiler - check wiring (ESP TX -> boiler RXD, ESP RX <- boiler TXD, common GND)");
  } else {
    ESP_LOGW(TAG, "invalid response frame (%u bytes)", (unsigned) n);
  }

  ESP_LOGD(TAG, "sample data (%u bytes): %s", (unsigned) n, hex_str_(readdata, n).c_str());
}

void Dietrich::handle_counter1_(const uint8_t *readdata, size_t n) {
  const bool calenta = this->variant_ == DIETRICH_VARIANT_CALENTA_V1_P5;

  if (this->frame_valid_(readdata, n)) {
    if (calenta) {
      this->publish_(this->hours_run_pump_sensor_, ((readdata[12] * 256) + readdata[13]) * 2);
      this->publish_(this->hours_run_3way_sensor_, ((readdata[14] * 256) + readdata[15]) * 2);
      this->publish_(this->hours_run_ch_sensor_, ((readdata[16] * 256) + readdata[17]) * 2);
      this->publish_(this->hours_run_dhw_sensor_, (readdata[18] * 256) + readdata[19]);
      this->publish_(this->power_supply_aval_hours_sensor_, ((readdata[20] * 256) + readdata[21]) * 2);
    } else {
      this->publish_(this->hours_run_pump_sensor_, ((readdata[7] * 256) + readdata[8]) * 2);
      this->publish_(this->hours_run_3way_sensor_, ((readdata[9] * 256) + readdata[10]) * 2);
      this->publish_(this->hours_run_ch_sensor_, ((readdata[11] * 256) + readdata[12]) * 2);
      this->publish_(this->hours_run_dhw_sensor_, (readdata[13] * 256) + readdata[14]);
      this->publish_(this->power_supply_aval_hours_sensor_, ((readdata[15] * 256) + readdata[16]) * 2);
      this->publish_(this->pump_starts_sensor_, ((readdata[17] * 256) + readdata[18]) * 8);
      this->publish_(this->number_of_3way_valve_cycles_sensor_, ((readdata[19] * 256) + readdata[20]) * 8);
      this->publish_(this->burner_start_dhw_sensor_, ((readdata[21] * 256) + readdata[22]) * 8);
    }
  }

  ESP_LOGD(TAG, "counter1 data (%u bytes): %s", (unsigned) n, hex_str_(readdata, n).c_str());
}

void Dietrich::handle_counter2_(const uint8_t *readdata, size_t n) {
  const bool calenta = this->variant_ == DIETRICH_VARIANT_CALENTA_V1_P5;

  if (this->frame_valid_(readdata, n)) {
    if (calenta) {
      this->publish_(this->pump_starts_sensor_, ((readdata[6] * 256) + readdata[7]) * 8);
      this->publish_(this->number_of_3way_valve_cycles_sensor_, ((readdata[8] * 256) + readdata[9]) * 8);
      this->publish_(this->burner_start_dhw_sensor_, ((readdata[10] * 256) + readdata[11]) * 8);
      this->publish_(this->total_burner_start_sensor_, ((readdata[12] * 256) + readdata[13]) * 8);
      this->publish_(this->failed_burner_start_sensor_, (readdata[14] * 256) + readdata[15]);
      this->publish_(this->number_flame_loss_sensor_, (readdata[16] * 256) + readdata[17]);
    } else {
      this->publish_(this->total_burner_start_sensor_, ((readdata[7] * 256) + readdata[8]) * 8);
      this->publish_(this->failed_burner_start_sensor_, (readdata[9] * 256) + readdata[10]);
      this->publish_(this->number_flame_loss_sensor_, (readdata[11] * 256) + readdata[12]);
    }
  }

  ESP_LOGD(TAG, "counter2 data (%u bytes): %s", (unsigned) n, hex_str_(readdata, n).c_str());
}

void Dietrich::update() {
  if (this->request_ != REQUEST_NONE) {
    ESP_LOGW(TAG, "previous request still running, skipping this update");
    return;
  }

  this->counter_timer_++;
  if (this->counter_timer_ >= 8) {
    this->counter_timer_ = 0;
    this->start_request_(REQUEST_COUNTER1);  // REQUEST_COUNTER2 follows from loop()
  } else {
    this->start_request_(REQUEST_SAMPLE);
  }
}

void Dietrich::loop() {
  if (this->request_ != REQUEST_NONE && this->reply_complete_()) {
    const Request done = this->request_;
    this->request_ = REQUEST_NONE;
    switch (done) {
      case REQUEST_SAMPLE:
        this->handle_sample_(this->rx_buffer_, this->rx_len_);
        break;
      case REQUEST_COUNTER1:
        this->handle_counter1_(this->rx_buffer_, this->rx_len_);
        this->start_request_(REQUEST_COUNTER2);
        break;
      case REQUEST_COUNTER2:
        this->handle_counter2_(this->rx_buffer_, this->rx_len_);
        break;
      default:
        break;
    }
  }

  // publish at most one queued value per loop pass
  if (this->publish_index_ < this->publish_queue_.size()) {
    auto &item = this->publish_queue_[this->publish_index_++];
    item.first->publish_state(item.second);
    if (this->publish_index_ >= this->publish_queue_.size()) {
      this->publish_queue_.clear();
      this->publish_index_ = 0;
    }
  }
}

void Dietrich::dump_config() {
  ESP_LOGCONFIG(TAG, "Dietrich boiler:");
  ESP_LOGCONFIG(TAG, "  Variant: %s",
                this->variant_ == DIETRICH_VARIANT_CALENTA_V1_P5 ? "calenta_v1_p5" : "mcr3");
  LOG_UPDATE_INTERVAL(this);
  this->check_uart_settings(9600);
}

}  // namespace dietrich
}  // namespace esphome
