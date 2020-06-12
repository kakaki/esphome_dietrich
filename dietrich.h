#include "esphome.h"

class Dietrich : public PollingComponent, public UARTDevice {
 public:
 
  //sample data sensors
  Sensor *flow_temp_sensor = new Sensor(); 
  Sensor *return_temp_sensor = new Sensor(); 
  Sensor *dhw_in_temp_sensor = new Sensor(); 
  Sensor *outside_temp_sensor = new Sensor();
  Sensor *calorifier_temp_sensor = new Sensor(); 
  
  Sensor *boiler_control_temp_sensor = new Sensor();
  Sensor *room_temp_sensor = new Sensor();
  Sensor *ch_setpoint_sensor = new Sensor(); //co 
  Sensor *dhw_setpoint_sensor = new Sensor(); //cwu
  Sensor *room_temp_setpoint_sensor = new Sensor();
  
  Sensor *fan_speed_setpoint_sensor = new Sensor();
  Sensor *fan_speed_sensor = new Sensor();
  
  Sensor *ionisation_current_sensor = new Sensor();
  Sensor *internal_setpoint_sensor = new Sensor();
  Sensor *available_power_sensor = new Sensor();
  Sensor *pump_percentage_sensor = new Sensor();

  Sensor *desired_max_power_sensor = new Sensor();
  Sensor *actual_power_sensor = new Sensor();

  Sensor *demand_source_bit0_sensor = new Sensor(); //BIT0=Mod.Controller Connected,  
  Sensor *demand_source_bit1_sensor = new Sensor(); //BIT1=Heat demand from Mod.Controller,
  Sensor *demand_source_bit2_sensor = new Sensor(); //BIT2=Heat demand from on/off controller, 
  Sensor *demand_source_bit3_sensor = new Sensor(); //BIT3=Frost Protection, 
  Sensor *demand_source_bit4_sensor = new Sensor(); //BIT4=DHW Eco, 
  Sensor *demand_source_bit5_sensor = new Sensor(); //BIT5=DHW Blocking, 
  Sensor *demand_source_bit6_sensor = new Sensor(); //BIT6=Anti Legionella
  Sensor *demand_source_bit7_sensor = new Sensor(); //BIT7=DHW Heat Demand  
    
  Sensor *input_bit0_sensor = new Sensor(); //BIT0=Shudown Input, 
	Sensor *input_bit1_sensor = new Sensor(); //BIT1=Release Input, 
	Sensor *input_bit2_sensor = new Sensor(); //BIT2=Ionisation, 
	Sensor *input_bit3_sensor = new Sensor(); //BIT3=Flow Switch detecting DHW, 
	Sensor *input_bit5_sensor = new Sensor(); //BIT5=Min Gas Pressure, 
	Sensor *input_bit6_sensor = new Sensor(); //BIT6=CH Enable, 
	Sensor *input_bit7_sensor = new Sensor(); //BIT7=DHW Enable

  Sensor *valve_bit0_sensor = new Sensor(); //BIT0=Gas Valve, 
  Sensor *valve_bit2_sensor = new Sensor(); //BIT2=Ignition, 
  Sensor *valve_bit3_sensor = new Sensor(); //BIT3=3-Way valve position, 
  Sensor *valve_bit4_sensor = new Sensor(); //BIT4=Ext.3-Way Valve, 
  Sensor *valve_bit6_sensor = new Sensor(); //BIT6=Ext. Gas Valve

  Sensor *pump_bit0_sensor = new Sensor(); //BIT0=Pump, 
  Sensor *pump_bit1_sensor = new Sensor(); //BIT1=Calorifier Pump, 
  Sensor *pump_bit2_sensor = new Sensor(); //BIT2=Ext.CH Pump, 
  Sensor *pump_bit4_sensor = new Sensor(); //BIT4=Status Report, 
  Sensor *pump_bit7_sensor = new Sensor(); //BIT7=Opentherm SmartPower
        
  Sensor *state_sensor = new Sensor();
  Sensor *lockout_sensor = new Sensor();
  Sensor *blocking_sensor = new Sensor();
  Sensor *sub_state_sensor = new Sensor();

  Sensor *hydro_pressure_sensor = new Sensor();
  Sensor *hru_sensor = new Sensor();
  Sensor *control_temp_sensor = new Sensor();
  Sensor *dhw_flowrate_sensor = new Sensor();
  
  //counter data sensors 1
  
	Sensor *hours_run_pump_sensor = new Sensor();
	Sensor *hours_run_3way_sensor = new Sensor();
	Sensor *hours_run_ch_sensor = new Sensor();
	Sensor *hours_run_dhw_sensor = new Sensor();

	Sensor *power_supply_aval_hours_sensor = new Sensor();
	Sensor *pump_starts_sensor = new Sensor();
	Sensor *number_of3way_valce_cycles_sensor = new Sensor();
	Sensor *burner_start_dhw_sensor = new Sensor();

  //counter data sensors 2
	Sensor *total_burner_start_sensor = new Sensor();
	Sensor *failed_burner_start_sensor = new Sensor();
	Sensor *number_flame_loss_sensor = new Sensor();

  
  Dietrich(UARTComponent *parent) : PollingComponent(10000), UARTDevice(parent) {}
  
  bool sem_reading_data = false;
  bool sem_read_all = true;
  int counter_timer = 99;

  byte sample[10] =    {0x02, 0xFE, 0x01, 0x05, 0x08, 0x02, 0x01, 0x69, 0xAB, 0x03 };
  byte counter1[10] =  {0x02, 0xFE, 0x00, 0x05, 0x08, 0x10, 0x1C, 0x98, 0xC2, 0x03 };
  byte counter2[10] =  {0x02, 0xFE, 0x00, 0x05, 0x08, 0x10, 0x1D, 0x59, 0x02, 0x03 };
    
  void array_to_string(byte array[], unsigned int len, char buffer[]) {
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
  }
  
  float signedFloat(float avalue) {
	  float f = avalue;
    if (f>32768) f = f-65536;
    return f;
  }
  

  void getSample() {
    byte readdata[80];
    char str[80] = "";
    
    //ESP_LOGD("custom", "read sample");
    
    write_array(sample,sizeof(sample));
    delay(500);
 
    int n=0;
    while(available()) {
      readdata[n] = read();
      n++;
    }
        
    if (readdata[0]==2 && readdata[1]==1 && readdata[2]==254) {//add crc check
    
        int bits = 0;
        
        if (flow_temp_sensor->get_name().length()>0) flow_temp_sensor->publish_state(signedFloat((readdata[8]*256)+readdata[7])*0.01); delay(100); //delay for esphome to not disconnect api        
        if (return_temp_sensor->get_name().length()>0) return_temp_sensor->publish_state(signedFloat((readdata[10]*256)+readdata[9])*0.01); delay(100); //delay for esphome to not disconnect api        
        if (dhw_in_temp_sensor->get_name().length()>0) dhw_in_temp_sensor->publish_state(signedFloat((readdata[12]*256)+readdata[11])*0.01); delay(100); //delay for esphome to not disconnect api
        if (outside_temp_sensor->get_name().length()>0) outside_temp_sensor->publish_state(signedFloat((readdata[14]*256)+readdata[13])*0.01); delay(100); //delay for esphome to not disconnect api
        if (calorifier_temp_sensor->get_name().length()>0) calorifier_temp_sensor->publish_state(signedFloat((readdata[16]*256)+readdata[15])*0.01); delay(100); //delay for esphome to not disconnect api
        if (boiler_control_temp_sensor->get_name().length()>0) boiler_control_temp_sensor->publish_state(signedFloat((readdata[20]*256)+readdata[19])*0.01); delay(100); //delay for esphome to not disconnect api
        if (room_temp_sensor->get_name().length()>0) room_temp_sensor->publish_state(signedFloat((readdata[22]*256)+readdata[21])*0.01); delay(100); //delay for esphome to not disconnect api
        if (ch_setpoint_sensor->get_name().length()>0) ch_setpoint_sensor->publish_state(signedFloat((readdata[24]*256)+readdata[23])*0.01); delay(100); //delay for esphome to not disconnect api
        if (dhw_setpoint_sensor->get_name().length()>0) dhw_setpoint_sensor->publish_state(signedFloat((readdata[26]*256)+readdata[25])*0.01); delay(100); //delay for esphome to not disconnect api
        if (room_temp_setpoint_sensor->get_name().length()>0) room_temp_setpoint_sensor->publish_state(signedFloat((readdata[28]*256)+readdata[27])*0.01); delay(100); //delay for esphome to not disconnect api
        if (fan_speed_setpoint_sensor->get_name().length()>0) fan_speed_setpoint_sensor->publish_state(signedFloat((readdata[30]*256)+readdata[29])); delay(100); //delay for esphome to not disconnect api
        if (fan_speed_sensor->get_name().length()>0) fan_speed_sensor->publish_state(signedFloat((readdata[32]*256)+readdata[31])); delay(100); //delay for esphome to not disconnect api        
        if (ionisation_current_sensor->get_name().length()>0) ionisation_current_sensor->publish_state(readdata[33]); delay(100); //delay for esphome to not disconnect api
        if (internal_setpoint_sensor->get_name().length()>0) internal_setpoint_sensor->publish_state(signedFloat((readdata[35]*256)+readdata[34])*0.01); delay(100); //delay for esphome to not disconnect api
        if (available_power_sensor->get_name().length()>0) available_power_sensor->publish_state(readdata[36]); delay(100); //delay for esphome to not disconnect api
        if (pump_percentage_sensor->get_name().length()>0) pump_percentage_sensor->publish_state(readdata[37]); delay(100); //delay for esphome to not disconnect api
        if (desired_max_power_sensor->get_name().length()>0) desired_max_power_sensor->publish_state(readdata[39]); delay(100); //delay for esphome to not disconnect api
        if (actual_power_sensor->get_name().length()>0) actual_power_sensor->publish_state(readdata[40]); delay(100); //delay for esphome to not disconnect api
        
        if (sem_read_all) {   
					bits = readdata[43];
					if (demand_source_bit0_sensor->get_name().length()>0) demand_source_bit0_sensor->publish_state(bitRead(bits, 0)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit1_sensor->get_name().length()>0) demand_source_bit1_sensor->publish_state(bitRead(bits, 1)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit2_sensor->get_name().length()>0) demand_source_bit2_sensor->publish_state(bitRead(bits, 2)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit3_sensor->get_name().length()>0) demand_source_bit3_sensor->publish_state(bitRead(bits, 3)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit4_sensor->get_name().length()>0) demand_source_bit4_sensor->publish_state(bitRead(bits, 4)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit5_sensor->get_name().length()>0) demand_source_bit5_sensor->publish_state(bitRead(bits, 5)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit6_sensor->get_name().length()>0) demand_source_bit6_sensor->publish_state(bitRead(bits, 6)); delay(100); //delay for esphome to not disconnect api
					if (demand_source_bit7_sensor->get_name().length()>0) demand_source_bit7_sensor->publish_state(bitRead(bits, 7)); delay(100); //delay for esphome to not disconnect api
				
					bits = readdata[44];        
					if (input_bit0_sensor->get_name().length()>0) input_bit0_sensor->publish_state(bitRead(bits, 0)); delay(100); //delay for esphome to not disconnect api
					if (input_bit1_sensor->get_name().length()>0) input_bit1_sensor->publish_state(bitRead(bits, 1)); delay(100); //delay for esphome to not disconnect api
					if (input_bit2_sensor->get_name().length()>0) input_bit2_sensor->publish_state(bitRead(bits, 2)); delay(100); //delay for esphome to not disconnect api
					if (input_bit3_sensor->get_name().length()>0) input_bit3_sensor->publish_state(bitRead(bits, 3)); delay(100); //delay for esphome to not disconnect api
					if (input_bit5_sensor->get_name().length()>0) input_bit5_sensor->publish_state(bitRead(bits, 5)); delay(100); //delay for esphome to not disconnect api
					if (input_bit6_sensor->get_name().length()>0) input_bit6_sensor->publish_state(bitRead(bits, 6)); delay(100); //delay for esphome to not disconnect api
					if (input_bit7_sensor->get_name().length()>0) input_bit7_sensor->publish_state(bitRead(bits, 7)); delay(100); //delay for esphome to not disconnect api

					bits = readdata[45];        
					if (valve_bit0_sensor->get_name().length()>0) valve_bit0_sensor->publish_state(bitRead(bits, 0)); delay(100); //delay for esphome to not disconnect api
					if (valve_bit2_sensor->get_name().length()>0) valve_bit2_sensor->publish_state(bitRead(bits, 2)); delay(100); //delay for esphome to not disconnect api
					if (valve_bit3_sensor->get_name().length()>0) valve_bit3_sensor->publish_state(bitRead(bits, 3)); delay(100); //delay for esphome to not disconnect api
					if (valve_bit4_sensor->get_name().length()>0) valve_bit4_sensor->publish_state(bitRead(bits, 4)); delay(100); //delay for esphome to not disconnect api
					if (valve_bit6_sensor->get_name().length()>0) valve_bit6_sensor->publish_state(bitRead(bits, 6)); delay(100); //delay for esphome to not disconnect api
				
					bits = readdata[46];        
					if (pump_bit0_sensor->get_name().length()>0) pump_bit0_sensor->publish_state(bitRead(bits, 0)); delay(100); //delay for esphome to not disconnect api
					if (pump_bit1_sensor->get_name().length()>0) pump_bit1_sensor->publish_state(bitRead(bits, 1)); delay(100); //delay for esphome to not disconnect api
					if (pump_bit2_sensor->get_name().length()>0) pump_bit2_sensor->publish_state(bitRead(bits, 2)); delay(100); //delay for esphome to not disconnect api
					if (pump_bit4_sensor->get_name().length()>0) pump_bit4_sensor->publish_state(bitRead(bits, 4)); delay(100); //delay for esphome to not disconnect api
					if (pump_bit7_sensor->get_name().length()>0) pump_bit7_sensor->publish_state(bitRead(bits, 7)); delay(100); //delay for esphome to not disconnect api
        }

        state_sensor->publish_state(readdata[47]); delay(200); //delay for esphome to not disconnect api
        lockout_sensor->publish_state(readdata[48]); delay(200); //delay for esphome to not disconnect api
        blocking_sensor->publish_state(readdata[49]); delay(200); //delay for esphome to not disconnect api
        sub_state_sensor->publish_state(readdata[50]); delay(200); //delay for esphome to not disconnect api

			  if (hydro_pressure_sensor->get_name().length()>0) hydro_pressure_sensor->publish_state(readdata[56]); delay(100); //delay for esphome to not disconnect api
			  
        bits = readdata[57];
        if (hru_sensor->get_name().length()>0) hru_sensor->publish_state(bitRead(bits, 1)); delay(100); //delay for esphome to not disconnect api
        
        if (control_temp_sensor->get_name().length()>0) control_temp_sensor->publish_state(signedFloat((readdata[59]*256)+readdata[58])*0.01); delay(100); //delay for esphome to not disconnect api
        if (dhw_flowrate_sensor->get_name().length()>0) dhw_flowrate_sensor->publish_state(signedFloat((readdata[61]*256)+readdata[60])*0.01); delay(100); //delay for esphome to not disconnect api
                       
        sem_read_all=!sem_read_all;
                        
    }
    else {
        ESP_LOGD("custom", "crc error");
    }
    
    array_to_string(readdata, 80, str);
    ESP_LOGD("custom", "sample data: %s", str);    
    
  }
  
  void getCounter() {
    byte readdata[28];
    char str[28] = "";
        
    write_array(counter1,sizeof(counter1));
    delay(500);

    int n=0;
    while(available()) {
      readdata[n] = read();
      n++;
    }
    
    if (readdata[0]==2 && readdata[1]==0 && readdata[2]==254) {//add crc check
    
      if (hours_run_pump_sensor->get_name().length()>0) hours_run_pump_sensor->publish_state(((readdata[7]*256)+readdata[8])*2); delay(100); //delay for esphome to not disconnect api      
      if (hours_run_3way_sensor->get_name().length()>0) hours_run_3way_sensor->publish_state(((readdata[9]*256)+readdata[10])*2); delay(100); //delay for esphome to not disconnect api
      if (hours_run_ch_sensor->get_name().length()>0) hours_run_ch_sensor->publish_state(((readdata[11]*256)+readdata[12])*2); delay(100); //delay for esphome to not disconnect api
      if (hours_run_dhw_sensor->get_name().length()>0) hours_run_dhw_sensor->publish_state(((readdata[13]*256)+readdata[14])); delay(100); //delay for esphome to not disconnect api
      if (power_supply_aval_hours_sensor->get_name().length()>0) power_supply_aval_hours_sensor->publish_state(((readdata[15]*256)+readdata[16])*2); delay(100); //delay for esphome to not disconnect api
      if (pump_starts_sensor->get_name().length()>0) pump_starts_sensor->publish_state(((readdata[17]*256)+readdata[18])*8); delay(100); //delay for esphome to not disconnect api
      if (number_of3way_valce_cycles_sensor->get_name().length()>0) number_of3way_valce_cycles_sensor->publish_state(((readdata[19]*256)+readdata[20])*8); delay(100); //delay for esphome to not disconnect api
      if (burner_start_dhw_sensor->get_name().length()>0) burner_start_dhw_sensor->publish_state(((readdata[21]*256)+readdata[22])*8); delay(100); //delay for esphome to not disconnect api
    }
    
    array_to_string(readdata, 28, str);
    ESP_LOGD("custom", "counter1 data: %s", str);
    
    write_array(counter2,sizeof(counter2));
    delay(500);

    n=0;
    while(available()) {
      readdata[n] = read();
      n++;
    }

    if (readdata[0]==2 && readdata[1]==0 && readdata[2]==254) {//add crc check
      if (total_burner_start_sensor->get_name().length()>0) total_burner_start_sensor->publish_state(((readdata[7]*256)+readdata[8])*8); delay(100); //delay for esphome to not disconnect api
      if (failed_burner_start_sensor->get_name().length()>0) failed_burner_start_sensor->publish_state(((readdata[9]*256)+readdata[10])); delay(100); //delay for esphome to not disconnect api
      if (number_flame_loss_sensor->get_name().length()>0) number_flame_loss_sensor->publish_state(((readdata[11]*256)+readdata[12])); delay(100); //delay for esphome to not disconnect api    
	}
    
    array_to_string(readdata, 28, str);
    ESP_LOGD("custom", "counter2 data: %s", str);
  }

  void setup() override {
  }

  void update() override {
  
    if (sem_reading_data) return;
          
    sem_reading_data=true;
    
    getSample();
    
    counter_timer++;
        
    if (counter_timer>=12) {
			counter_timer=0;
			getCounter();
    }
    
    sem_reading_data=false;
    
  }
};