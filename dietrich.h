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
    
  Sensor *state_sensor = new Sensor();
  Sensor *sub_state_sensor = new Sensor();
  
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
    
        float temp = 0.0;
        int bits = 0;
        
        temp = (readdata[8]*256)+readdata[7];
        if (temp>32768) temp = temp-65536;
        flow_temp_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[10]*256)+readdata[9];
        if (temp>32768) temp = temp-65536;
        return_temp_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[12]*256)+readdata[11];
        if (temp>32768) temp = temp-65536;
        dhw_in_temp_sensor->publish_state(temp*0.01);
        
        delay(100); //delay for esphome to not disconnect api
                
        temp = (readdata[14]*256)+readdata[13];
        if (temp>32768) temp = temp-65536;
        outside_temp_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[16]*256)+readdata[15];
        if (temp>32768) temp = temp-65536;
        calorifier_temp_sensor->publish_state(temp*0.01);
        
        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[20]*256)+readdata[19];
        if (temp>32768) temp = temp-65536;
        boiler_control_temp_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[22]*256)+readdata[21];
        if (temp>32768) temp = temp-65536;
        room_temp_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[24]*256)+readdata[23];
        if (temp>32768) temp = temp-65536;
        ch_setpoint_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[26]*256)+readdata[25];
        if (temp>32768) temp = temp-65536;
        dhw_setpoint_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[28]*256)+readdata[27];
        if (temp>32768) temp = temp-65536;
        room_temp_setpoint_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
                
        temp = (readdata[30]*256)+readdata[29];
        if (temp>32768) temp = temp-65536;
        fan_speed_setpoint_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[32]*256)+readdata[31];
        if (temp>32768) temp = temp-65536;
        fan_speed_sensor->publish_state(temp*0.01);
        
        delay(100); //delay for esphome to not disconnect api
        
        ionisation_current_sensor->publish_state(readdata[33]);

        delay(100); //delay for esphome to not disconnect api
        
        temp = (readdata[35]*256)+readdata[33];
        if (temp>32768) temp = temp-65536;
        internal_setpoint_sensor->publish_state(temp*0.01);

        delay(100); //delay for esphome to not disconnect api

        available_power_sensor->publish_state(readdata[36]);

        delay(100); //delay for esphome to not disconnect api

        pump_percentage_sensor->publish_state(readdata[37]);

        delay(100); //delay for esphome to not disconnect api
        
        desired_max_power_sensor->publish_state(readdata[39]);

        delay(100); //delay for esphome to not disconnect api

        actual_power_sensor->publish_state(readdata[40]);          
        
        delay(200); //delay for esphome to not disconnect api
        
        bits = readdata[43];
        
        demand_source_bit0_sensor->publish_state(bitRead(bits, 0));

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit1_sensor->publish_state(bitRead(bits, 1));        

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit2_sensor->publish_state(bitRead(bits, 2));  		

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit3_sensor->publish_state(bitRead(bits, 3));        

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit4_sensor->publish_state(bitRead(bits, 4));

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit5_sensor->publish_state(bitRead(bits, 5));

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit6_sensor->publish_state(bitRead(bits, 6));

        delay(100); //delay for esphome to not disconnect api

        demand_source_bit7_sensor->publish_state(bitRead(bits, 7));                                
  
        delay(100); //delay for esphome to not disconnect api
        
        state_sensor->publish_state(readdata[47]);

        delay(100); //delay for esphome to not disconnect api

        sub_state_sensor->publish_state(readdata[50]);
                
    }
    else {
        ESP_LOGD("custom", "crc error");
    }
    
    array_to_string(readdata, 80, str);
    ESP_LOGD("custom", "data: %s", str);    
    
  }
  
  void getCounter() {
    byte readdata[28];
    char str[28] = "";
    
    ESP_LOGD("custom", "read counter");
    
    write_array(counter1,sizeof(counter1));
    delay(500);

    int n=0;
    while(available()) {
      readdata[n] = read();
      n++;
    }
    
    if (readdata[0]==2 && readdata[1]==0 && readdata[2]==254) {//add crc check
    
      hours_run_pump_sensor->publish_state(((readdata[7]*256)+readdata[8])*2);
      
			delay(100); //delay for esphome to not disconnect api
      
      hours_run_3way_sensor->publish_state(((readdata[9]*256)+readdata[10])*2);      

			delay(100); //delay for esphome to not disconnect api

      hours_run_ch_sensor->publish_state(((readdata[11]*256)+readdata[12])*2);

			delay(100); //delay for esphome to not disconnect api

      hours_run_dhw_sensor->publish_state(((readdata[13]*256)+readdata[14])*2);

      power_supply_aval_hours_sensor->publish_state(((readdata[15]*256)+readdata[16])*2);

			delay(100); //delay for esphome to not disconnect api

      pump_starts_sensor->publish_state(((readdata[17]*256)+readdata[18])*8);

			delay(100); //delay for esphome to not disconnect api

      number_of3way_valce_cycles_sensor->publish_state(((readdata[19]*256)+readdata[20])*8);

			delay(100); //delay for esphome to not disconnect api

      burner_start_dhw_sensor->publish_state(((readdata[21]*256)+readdata[22])*8);

    }
    
    array_to_string(readdata, 28, str);
    ESP_LOGD("custom", "data: %s", str);
    
    write_array(counter2,sizeof(counter2));
    delay(500);

    n=0;
    while(available()) {
      readdata[n] = read();
      n++;
    }

    if (readdata[0]==2 && readdata[1]==0 && readdata[2]==254) {//add crc check
      total_burner_start_sensor->publish_state(((readdata[7]*256)+readdata[8])*8);

			delay(100); //delay for esphome to not disconnect api

      failed_burner_start_sensor->publish_state(((readdata[9]*256)+readdata[10]));

			delay(100); //delay for esphome to not disconnect api

      number_flame_loss_sensor->publish_state(((readdata[11]*256)+readdata[12]));
    
	}

    
    array_to_string(readdata, 28, str);
    ESP_LOGD("custom", "data: %s", str);
  }

  void setup() override {
  }

  void update() override {
  
    if (sem_reading_data) return;
    
      
    sem_reading_data=true;
    
    getSample();
    
    counter_timer++;
        
    if (counter_timer>=12) {
    	delay(200);
			counter_timer=0;
			getCounter();
    }
    
    sem_reading_data=false;
    
  }
};