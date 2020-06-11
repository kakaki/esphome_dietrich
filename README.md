# Dietrich (Remeha) Boiler connectivity using ESP8266

Library for reading data from De Dietrich (or Remeha) PC interface. 
For this we use an ESP8266 (Wemos D1)


It connects to the boiler using a 4P4C (RJ10) connector with the following pinouts:
```
 Heater Board from top       ESP8266
    4P4C RJ connector
    
       +---------+
GND 4  ---       +--+        GND
TXD 3  ---          |        RX
RXD 2  ---          |        TX
5V  1  ---       +--+        5V
       +---------+
```

