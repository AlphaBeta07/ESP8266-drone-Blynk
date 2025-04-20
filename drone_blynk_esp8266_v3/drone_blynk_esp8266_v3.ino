#define BLYNK_TEMPLATE_ID "TMPL3rPMucg_r"
#define BLYNK_TEMPLATE_NAME "DRONE"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "Wire.h"

// Motor Control Pins
#define MOTOR1_PIN D5
#define MOTOR2_PIN D6
#define MOTOR3_PIN D7
#define MOTOR4_PIN D8

// Blynk Credentials
char auth[] = "I_cMxg1joHbquc8RcZmUF2WwOi07Ujvb";
char ssid[] = "Anish's S22 Ultra";
char pass[] = "ph82ubqayk";

// Blynk Virtual Pins
#define THROTTLE_JOYSTICK V0
#define YAW_JOYSTICK      V1
#define ROLL_JOYSTICK     V2
#define PITCH_JOYSTICK    V3
#define MODE_SWITCH       V4

// Motor Control Variables
int motorSpeed[4] = {0};
int input_PITCH = 50;
int input_ROLL = 50;
int input_YAW = 50;
volatile int input_THROTTLE = 0;
int Mode = 0;

// MPU6050 Variables
int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature; // Added temperature
float angle_pitch, angle_roll, angle_yaw;
float angle_pitch_output, angle_roll_output, angle_yaw_output;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

// PID Variables
float roll_PID, pitch_PID, yaw_PID;
double twoX_kp = 5, twoX_ki = 0.003, twoX_kd = 1.4;
double yaw_kp = 8, yaw_ki = 0, yaw_kd = 4;

void setup() {
  // Initialize Motor PWM Pins
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
  
  // Set 8kHz PWM frequency
  analogWriteFreq(8000);
  analogWriteRange(255);

  // Initialize Serial
  Serial.begin(115200);

  // Initialize Blynk
  Blynk.begin(auth, ssid, pass);

  // Initialize MPU6050
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  
  // Configure MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x08); // ±4g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x18); // 2000dps
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x03); // DLPF
  Wire.endTransmission();

  // Calibrate Gyro
  for (int i = 0; i < 2000; i++) {
    if(i % 100 == 0) Serial.print(".");
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    acc_y = Wire.read()<<8|Wire.read();
    acc_x = Wire.read()<<8|Wire.read();
    acc_z = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read(); // Now properly declared
    gyro_y = Wire.read()<<8|Wire.read();
    gyro_x = Wire.read()<<8|Wire.read();
    gyro_z = Wire.read()<<8|Wire.read();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delayMicroseconds(100);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
}

// ... [Rest of the code remains exactly the same as previously provided] ...

void loop() {
  Blynk.run();
  
  // Read MPU6050
  static uint32_t last_read = 0;
  float elapsedTime = (micros() - last_read) / 1000000.0;
  last_read = micros();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,14);
  acc_y = Wire.read()<<8|Wire.read();
  acc_x = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  temperature = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();

  // Process IMU Data
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  angle_pitch += gyro_x * elapsedTime * 0.0610687023;
  angle_roll += gyro_y * elapsedTime * 0.0610687023;
  
  // Complementary Filter
  angle_pitch_output = 0.9 * (angle_pitch_output + gyro_x * elapsedTime) + 0.1 * atan(acc_y/sqrt(acc_x*acc_x + acc_z*acc_z)) * 57.296;
  angle_roll_output = 0.9 * (angle_roll_output + gyro_y * elapsedTime) + 0.1 * atan(acc_x/sqrt(acc_y*acc_y + acc_z*acc_z)) * -57.296;

  // PID Calculation
  float roll_error = angle_roll_output - (3 * (input_ROLL - 50) / 10.0);
  float pitch_error = angle_pitch_output - (3 * (input_PITCH - 50) / 10.0);
  
  roll_PID = twoX_kp * roll_error + twoX_kd * (gyro_y * 0.0610687023);
  pitch_PID = twoX_kp * pitch_error + twoX_kd * (gyro_x * 0.0610687023);
  yaw_PID = yaw_kp * angle_yaw_output + yaw_kd * (gyro_z * 0.0610687023);

  // Calculate Motor Speeds
  motorSpeed[0] = constrain(input_THROTTLE + pitch_PID - roll_PID + yaw_PID, 0, 255);
  motorSpeed[1] = constrain(input_THROTTLE + pitch_PID + roll_PID - yaw_PID, 0, 255);
  motorSpeed[2] = constrain(input_THROTTLE - pitch_PID + roll_PID + yaw_PID, 0, 255);
  motorSpeed[3] = constrain(input_THROTTLE - pitch_PID - roll_PID - yaw_PID, 0, 255);

  // Apply PWM to Motors
  analogWrite(MOTOR1_PIN, motorSpeed[0]);
  analogWrite(MOTOR2_PIN, motorSpeed[1]);
  analogWrite(MOTOR3_PIN, motorSpeed[2]);
  analogWrite(MOTOR4_PIN, motorSpeed[3]);

  // Debug Output
  static uint32_t last_print = 0;
  if(millis() - last_print > 100) {
    last_print = millis();
    Serial.printf("Thr:%3d | R:%3d P:%3d | PID: R:%5.1f P:%5.1f\n",
      input_THROTTLE, input_ROLL, input_PITCH, roll_PID, pitch_PID);
  }
}

// Blynk Control Handlers
BLYNK_WRITE(THROTTLE_JOYSTICK) {
  input_THROTTLE = map(param[1].asInt(), 0, 255, 0, 255);
}
BLYNK_WRITE(YAW_JOYSTICK) {
  input_YAW = map(param[0].asInt(), 0, 255, 0, 100);
}
BLYNK_WRITE(ROLL_JOYSTICK) {
  input_ROLL = map(param[0].asInt(), 0, 255, 0, 100);
}
BLYNK_WRITE(PITCH_JOYSTICK) {
  input_PITCH = map(param[1].asInt(), 0, 255, 0, 100);
}
BLYNK_WRITE(MODE_SWITCH) {
  Mode = param.asInt();
}
