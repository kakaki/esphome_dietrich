#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace dietrich {

enum DietrichVariant : uint8_t {
  DIETRICH_VARIANT_MCR3 = 0,
  DIETRICH_VARIANT_CALENTA_V1_P5,
};

class Dietrich : public PollingComponent, public uart::UARTDevice {
 public:
  void set_variant(DietrichVariant variant) { this->variant_ = variant; }
  // frame status/state
  SUB_SENSOR(state)
  SUB_SENSOR(sub_state)
  SUB_SENSOR(lockout)
  SUB_SENSOR(blocking)

  // sample data - temperatures
  SUB_SENSOR(flow_temp)
  SUB_SENSOR(return_temp)
  SUB_SENSOR(dhw_in_temp)
  SUB_SENSOR(outside_temp)
  SUB_SENSOR(calorifier_temp)
  SUB_SENSOR(boiler_control_temp)
  SUB_SENSOR(room_temp)
  SUB_SENSOR(ch_setpoint)          // co
  SUB_SENSOR(dhw_setpoint)         // cwu
  SUB_SENSOR(room_temp_setpoint)

  // fan
  SUB_SENSOR(fan_speed_setpoint)
  SUB_SENSOR(fan_speed)

  // power / misc
  SUB_SENSOR(ionisation_current)
  SUB_SENSOR(internal_setpoint)
  SUB_SENSOR(available_power)
  SUB_SENSOR(pump_percentage)
  SUB_SENSOR(desired_max_power)
  SUB_SENSOR(actual_power)

  SUB_SENSOR(demand_source_bit0)  // BIT0=Mod.Controller Connected
  SUB_SENSOR(demand_source_bit1)  // BIT1=Heat demand from Mod.Controller
  SUB_SENSOR(demand_source_bit2)  // BIT2=Heat demand from on/off controller
  SUB_SENSOR(demand_source_bit3)  // BIT3=Frost Protection
  SUB_SENSOR(demand_source_bit4)  // BIT4=DHW Eco
  SUB_SENSOR(demand_source_bit5)  // BIT5=DHW Blocking
  SUB_SENSOR(demand_source_bit6)  // BIT6=Anti Legionella
  SUB_SENSOR(demand_source_bit7)  // BIT7=DHW Heat Demand

  SUB_SENSOR(input_bit0)  // BIT0=Shutdown Input
  SUB_SENSOR(input_bit1)  // BIT1=Release Input
  SUB_SENSOR(input_bit2)  // BIT2=Ionisation
  SUB_SENSOR(input_bit3)  // BIT3=Flow Switch detecting DHW
  SUB_SENSOR(input_bit5)  // BIT5=Min Gas Pressure
  SUB_SENSOR(input_bit6)  // BIT6=CH Enable
  SUB_SENSOR(input_bit7)  // BIT7=DHW Enable

  SUB_SENSOR(valve_bit0)  // BIT0=Gas Valve
  SUB_SENSOR(valve_bit2)  // BIT2=Ignition
  SUB_SENSOR(valve_bit3)  // BIT3=3-Way valve position
  SUB_SENSOR(valve_bit4)  // BIT4=Ext.3-Way Valve
  SUB_SENSOR(valve_bit6)  // BIT6=Ext. Gas Valve

  SUB_SENSOR(pump_bit0)  // BIT0=Pump
  SUB_SENSOR(pump_bit1)  // BIT1=Calorifier Pump
  SUB_SENSOR(pump_bit2)  // BIT2=Ext.CH Pump
  SUB_SENSOR(pump_bit4)  // BIT4=Status Report
  SUB_SENSOR(pump_bit7)  // BIT7=Opentherm SmartPower

  SUB_SENSOR(hydro_pressure)
  SUB_SENSOR(hru)
  SUB_SENSOR(control_temp)
  SUB_SENSOR(dhw_flowrate)

  // counter data 1
  SUB_SENSOR(hours_run_pump)
  SUB_SENSOR(hours_run_3way)
  SUB_SENSOR(hours_run_ch)
  SUB_SENSOR(hours_run_dhw)
  SUB_SENSOR(power_supply_aval_hours)
  SUB_SENSOR(pump_starts)
  SUB_SENSOR(number_of_3way_valve_cycles)
  SUB_SENSOR(burner_start_dhw)

  // counter data 2
  SUB_SENSOR(total_burner_start)
  SUB_SENSOR(failed_burner_start)
  SUB_SENSOR(number_flame_loss)

  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void get_sample_();
  void get_counter_();
  // publish only if the sensor is configured; the short delay keeps the
  // API/WiFi stack fed so Home Assistant does not disconnect
  void publish_(sensor::Sensor *s, float value, uint32_t wait = 100);
  size_t transact_(const uint8_t *request, size_t req_len, uint8_t *buffer, size_t len);
  bool frame_valid_(const uint8_t *response, size_t n) const;
  static bool is_valid_crc_(const uint8_t *response, size_t n);
  static float signed_float_(float value);
  static float temp_or_nan_(uint16_t raw);
  static std::string hex_str_(const uint8_t *data, size_t len);

  DietrichVariant variant_{DIETRICH_VARIANT_MCR3};
  bool reading_data_{false};
  bool read_all_{true};
  int counter_timer_{99};
};

}  // namespace dietrich
}  // namespace esphome
