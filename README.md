# ESP8266-drone-Blynk
# ESP8266-Based WiFi Drone with MPU6050 and Coreless Motors Controlled via Blynk App
# Project Description:
<br>  This project showcases the development of a compact, WiFi-controlled drone built using an ESP8266 microcontroller, MPU6050 gyroscope/accelerometer, and coreless motors. The drone is designed to be controlled remotely through the Blynk mobile application, offering an easy and modern interface for flight control.
<br>  The ESP8266 serves as the central flight controller, handling both sensor data processing and communication. The MPU6050 sensor provides real-time orientation and motion data, which is used to maintain balance and stability through a PID control algorithm. The drone's propulsion system consists of coreless DC motors, providing lightweight and efficient thrust suitable for micro aerial vehicles.
<br>  The Blynk app on a smartphone connects to the drone over WiFi, enabling the user to control throttle, pitch, and roll via virtual joystick or sliders. The use of Blynk simplifies the control interface and eliminates the need for physical RC transmitters.

# Key Features:
  WiFi-based control using Blynk mobile app<br>
  Real-time orientation sensing with MPU6050<br>
  PID stabilization for smooth flight<br>
  Lightweight design with coreless motors<br>
  Compact and beginner-friendly architecture<br>
  Fully wireless, smartphone-operated drone<br>

# Blynk App Setup Instructions
  Create a new project in the Blynk app.<br>
  <br>
  Add the following widgets:<br>
      Two Joystick widgets:<br>
        Left Joystick (Virtual Pin V0 for Y-axis - Throttle, V1 for X-axis - Yaw).<br>
        Right Joystick (Virtual Pin V2 for X-axis - Roll, V3 for Y-axis - Pitch).<br>
        <br>
  Configure each joystick to output values from 0-1023.<br>
  Make sure the ESP8266 is connected to the same network as your phone.<br>
<br>
# Circuit Diagram
![circuit dia](https://github.com/user-attachments/assets/c575ba27-fbf4-461d-bf2e-4a55b83f2836)<br>

# Motor Alignments
![motor alignments](https://github.com/user-attachments/assets/91b084fc-5c3c-4f2b-9d8f-1196b5381ae8)

