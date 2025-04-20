# ESP8266-drone-Blynk

NodeMCU GPIO (D5-D8)  
       ↓  
  [SI2303 MOSFET]  
    Gate → GPIO  
    Drain → Motor wire  
    Source → GND  
       ↑  
Motor casing → GND (via frame)  


+-------------------+       +-------------------+
|                   |       |                   |
|    LiPo Battery   |       |      MPU6050      |
|     (3.7V-5V)     |       |                   |
+--------+----------+       +--------+----------+
         |                           |
         |                           |
     +---v---+                   +---v---+
     |       |                   |       |
     | 5V BEC|                   | 3.3V  |
     |       |                   |       |
     +---+---+                   +---+---+
         |                           |
  +-----v-----+               +------v------+
  |  NodeMCU  |               |   SI2303    |
  |   ESP8266 |               |  MOSFET x4  |
  +-----+-----+               +------+------+
        |  D5-D8 (PWM)               |
        +--------+-------------------+
                 |
             +---v---+
             | Motor |
             | Driver|
             +---+---+
                 |
             +---v---+
             | Coreless|
             | Motor x4|
             +---------+


       Front
        ↑
M2 (CCW)   M1 (CW)
    \     /
       X
    /     \
M3 (CW)   M4 (CCW)
        ↓
       Back
