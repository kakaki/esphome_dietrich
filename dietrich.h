#include "esphome.h"

class Dietrich : public PollingComponent, public UARTDevice {
 public:
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
  
  Dietrich(UARTComponent *parent) : PollingComponent(10000), UARTDevice(parent) {}

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
        
        temp = (readdata[10]*256)+readdata[9];
        if (temp>32768) temp = temp-65536;
        return_temp_sensor->publish_state(temp*0.01);
        
        temp = (readdata[12]*256)+readdata[11];
        if (temp>32768) temp = temp-65536;
        dhw_in_temp_sensor->publish_state(temp*0.01);
        
        delay(200); //delay for esphome to not disconnect api
                
        temp = (readdata[14]*256)+readdata[13];
        if (temp>32768) temp = temp-65536;
        outside_temp_sensor->publish_state(temp*0.01);
        
        temp = (readdata[16]*256)+readdata[15];
        if (temp>32768) temp = temp-65536;
        calorifier_temp_sensor->publish_state(temp*0.01);
        
        
        temp = (readdata[20]*256)+readdata[19];
        if (temp>32768) temp = temp-65536;
        boiler_control_temp_sensor->publish_state(temp*0.01);

        delay(200); //delay for esphome to not disconnect api
        
        temp = (readdata[22]*256)+readdata[21];
        if (temp>32768) temp = temp-65536;
        room_temp_sensor->publish_state(temp*0.01);
        
        temp = (readdata[24]*256)+readdata[23];
        if (temp>32768) temp = temp-65536;
        ch_setpoint_sensor->publish_state(temp*0.01);
        
        temp = (readdata[26]*256)+readdata[25];
        if (temp>32768) temp = temp-65536;
        dhw_setpoint_sensor->publish_state(temp*0.01);

        delay(200); //delay for esphome to not disconnect api
        
        temp = (readdata[28]*256)+readdata[27];
        if (temp>32768) temp = temp-65536;
        room_temp_setpoint_sensor->publish_state(temp*0.01);
                
        temp = (readdata[30]*256)+readdata[29];
        if (temp>32768) temp = temp-65536;
        fan_speed_setpoint_sensor->publish_state(temp*0.01);
        
        temp = (readdata[32]*256)+readdata[31];
        if (temp>32768) temp = temp-65536;
        fan_speed_sensor->publish_state(temp*0.01);
        
        delay(200); //delay for esphome to not disconnect api
        
        ionisation_current_sensor->publish_state(readdata[33]);
        
        temp = (readdata[35]*256)+readdata[33];
        if (temp>32768) temp = temp-65536;
        internal_setpoint_sensor->publish_state(temp*0.01);

        available_power_sensor->publish_state(readdata[36]);

        delay(200); //delay for esphome to not disconnect api

        pump_percentage_sensor->publish_state(readdata[37]);
        
        desired_max_power_sensor->publish_state(readdata[39]);
        actual_power_sensor->publish_state(readdata[40]);          
        
        delay(200); //delay for esphome to not disconnect api
        
        bits = readdata[43];
        
        demand_source_bit0_sensor->publish_state(bitRead(bits, 0));
        demand_source_bit1_sensor->publish_state(bitRead(bits, 1));        
        demand_source_bit2_sensor->publish_state(bitRead(bits, 2));  		

        delay(200); //delay for esphome to not disconnect api

        demand_source_bit3_sensor->publish_state(bitRead(bits, 3));        
        demand_source_bit4_sensor->publish_state(bitRead(bits, 4));
        demand_source_bit5_sensor->publish_state(bitRead(bits, 5));

        delay(200); //delay for esphome to not disconnect api

        demand_source_bit6_sensor->publish_state(bitRead(bits, 6));
        demand_source_bit7_sensor->publish_state(bitRead(bits, 7));                                
  
        
        delay(200); //delay for esphome to not disconnect api
        
        state_sensor->publish_state(readdata[47]);
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
    
    array_to_string(readdata, 28, str);
    ESP_LOGD("custom", "data: %s", str);
    
    write_array(counter2,sizeof(counter2));
    delay(500);

    n=0;
    while(available()) {
      readdata[n] = read();
      n++;
    }
    
    array_to_string(readdata, 28, str);
    ESP_LOGD("custom", "data: %s", str);
  }

  void setup() override {
  }

  void update() override {
    getSample();
    delay(500);
    //getCounter();
    
  }
};