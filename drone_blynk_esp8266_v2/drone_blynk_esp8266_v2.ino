#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>

// Blynk 2.0 Configuration
#define BLYNK_TEMPLATE_ID "TMPL3rPMucg_r" // Replace with your Template ID
#define BLYNK_DEVICE_NAME "DRONE"
#define BLYNK_AUTH_TOKEN "I_cMxg1joHbquc8RcZmUF2WwOi07Ujvb" // From Blynk app
#define BLYNK_FIRMWARE_VERSION "1.0"

// WiFi credentials
char ssid[] = "YourWiFiSSID";
char pass[] = "YourWiFiPassword";

// Motor control variables
int ESCout_1, ESCout_2, ESCout_3, ESCout_4;
int input_PITCH = 50;
int input_ROLL = 50;
int input_YAW = 50;
volatile int input_THROTTLE = 0;

// IMU and PID variables
struct IMUData {
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t acc_x, acc_y, acc_z;
  int16_t temperature;
  float angle_pitch, angle_roll, angle_yaw;
  float angle_pitch_output, angle_roll_output, angle_yaw_output;
} imu;

struct PID {
  float error, previous_error;
  float pid_p, pid_i, pid_d;
  float output;
  double kp, ki, kd;
} roll_pid, pitch_pid, yaw_pid;

// Timing variables
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 4000; // microseconds

// PWM control
struct PWMControl {
  volatile int order[4] = {0};
  int pulldown_time[5] = {0};
  volatile int pulldown_time_temp[5] = {0};
  uint8_t pins[4] = {14, 12, 13, 15}; // D5, D6, D7, D8
  uint8_t pwm_stops = 0;
  bool timer_initialized = false;
} pwm;

// Blynk Widgets
WidgetJoystick leftStick(V0);  // Throttle (Y), Yaw (X)
WidgetJoystick rightStick(V1); // Pitch (Y), Roll (X)

void ICACHE_RAM_ATTR PWM_callback() {
  switch (pwm.pwm_stops) {
    case 0:
      memcpy((void*)pwm.pulldown_time_temp, (void*)pwm.pulldown_time, sizeof(pwm.pulldown_time));
      pwm.pwm_stops = 1;
      if (input_THROTTLE != 0) {
        GPOS = (1 << pwm.pins[0]) | (1 << pwm.pins[1]) | (1 << pwm.pins[2]) | (1 << pwm.pins[3]);
      }
      timer1_write(80 * pwm.pulldown_time_temp[0]);
      break;
      
    case 1: case 2: case 3: case 4:
      pwm.pwm_stops++;
      GPOC = (1 << pwm.pins[pwm.order[pwm.pwm_stops-2]]);
      timer1_write(80 * pwm.pulldown_time_temp[pwm.pwm_stops-1]);
      break;
      
    default:
      pwm.pwm_stops = 0;
      GPOC = (1 << pwm.pins[pwm.order[3]]);
      timer1_write(80 * pwm.pulldown_time_temp[4]);
  }
}

// Blynk input handlers
BLYNK_WRITE(V0) { // Left Stick (Throttle/Yaw)
  input_THROTTLE = map(param[1].asInt(), 0, 1023, 0, 1200); // Y-axis -> Throttle
  input_YAW = map(param[0].asInt(), 0, 1023, 0, 100);      // X-axis -> Yaw
}

BLYNK_WRITE(V1) { // Right Stick (Pitch/Roll)
  input_PITCH = map(param[1].asInt(), 0, 1023, 0, 100);    // Y-axis -> Pitch
  input_ROLL = map(param[0].asInt(), 0, 1023, 0, 100);     // X-axis -> Roll
}

BLYNK_CONNECTED() {
  // Sync the hardware state when device connects
  Blynk.syncVirtual(V0, V1);
}

void setupPIDs() {
  // Roll PID
  roll_pid.kp = 5.0;
  roll_pid.ki = 0.003;
  roll_pid.kd = 1.4;
  
  // Pitch PID
  pitch_pid.kp = 5.0;
  pitch_pid.ki = 0.003;
  pitch_pid.kd = 1.4;
  
  // Yaw PID
  yaw_pid.kp = 8.0;
  yaw_pid.ki = 0.0;
  yaw_pid.kd = 4.0;
}

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(100);

  // Initialize motor pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pwm.pins[i], OUTPUT);
    digitalWrite(pwm.pins[i], LOW);
  }

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 8080);

  // Initialize MPU6050
  Wire.begin();
  Wire.setClock(400000);
  
  // Configure MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00); // Wake up
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x08); // ±4g accelerometer range
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x18); // 2000dps gyro range
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x03); // DLPF config
  Wire.endTransmission();

  // Calibrate gyro
  calibrateGyro();

  // Setup PIDs
  setupPIDs();

  // Initialize PWM timer
  timer1_attachInterrupt(PWM_callback);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
}

void calibrateGyro() {
  long gyro_x_total = 0, gyro_y_total = 0, gyro_z_total = 0;
  
  for (int i = 0; i < 4000; i++) {
    if (i % 125 == 0) Serial.print(".");
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    
    while (Wire.available() < 14);
    
    // Skip accelerometer and temperature data
    for (int j = 0; j < 7; j++) Wire.read() << 8 | Wire.read();
    
    gyro_y_total += Wire.read() << 8 | Wire.read();
    gyro_x_total += Wire.read() << 8 | Wire.read();
    gyro_z_total += Wire.read() << 8 | Wire.read();
    
    delayMicroseconds(100);
  }
  
  imu.gyro_x = gyro_x_total / 4000;
  imu.gyro_y = gyro_y_total / 4000;
  imu.gyro_z = gyro_z_total / 4000;
}

void readIMU() {
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - lastReadTime) / 1000000.0;
  lastReadTime = currentTime;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  
  while (Wire.available() < 14);
  
  imu.acc_y = Wire.read() << 8 | Wire.read();
  imu.acc_x = Wire.read() << 8 | Wire.read();
  imu.acc_z = Wire.read() << 8 | Wire.read();
  imu.temperature = Wire.read() << 8 | Wire.read();
  imu.gyro_y = Wire.read() << 8 | Wire.read();
  imu.gyro_x = Wire.read() << 8 | Wire.read();
  imu.gyro_z = Wire.read() << 8 | Wire.read();

  // Remove calibration offsets
  imu.gyro_x -= imu.gyro_x;
  imu.gyro_y -= imu.gyro_y;
  imu.gyro_z -= imu.gyro_z;

  // Convert to degrees/sec
  float gyro_x_rate = imu.gyro_x * (-0.0610687023);
  float gyro_y_rate = imu.gyro_y * (-0.0610687023);
  float gyro_z_rate = imu.gyro_z * (-0.0610687023);

  // Integrate gyro rates to get angles
  imu.angle_pitch += gyro_x_rate * elapsedTime;
  imu.angle_roll += gyro_y_rate * elapsedTime;
  imu.angle_yaw += gyro_z_rate * elapsedTime;

  // Normalize yaw angle
  if (imu.angle_yaw >= 180.0) imu.angle_yaw -= 360.0;
  else if (imu.angle_yaw < -180.0) imu.angle_yaw += 360.0;

  // Calculate angles from accelerometer
  imu.angle_roll = atan2(imu.acc_x, sqrt(imu.acc_y * imu.acc_y + imu.acc_z * imu.acc_z)) * (-57.296);
  imu.angle_pitch = atan2(imu.acc_y, sqrt(imu.acc_x * imu.acc_x + imu.acc_z * imu.acc_z)) * 57.296;

  // Apply complementary filter
  imu.angle_pitch_output = imu.angle_pitch_output * 0.9 + imu.angle_pitch * 0.1;
  imu.angle_roll_output = imu.angle_roll_output * 0.9 + imu.angle_roll * 0.1;
  imu.angle_yaw_output = imu.angle_yaw_output * 0.9 + imu.angle_yaw * 0.1;
}

void calculatePID() {
  // Calculate desired angles from controller input
  float roll_target = 3.0 * (input_ROLL - 50) / 10.0;
  float pitch_target = 3.0 * (input_PITCH - 50) / 10.0;
  
  // Calculate P-factor based on throttle
  float P_factor = 0.001286376 * input_THROTTLE + 0.616932;

  // Roll PID
  roll_pid.error = imu.angle_roll_output - roll_target;
  roll_pid.pid_p = P_factor * roll_pid.kp * roll_pid.error;
  roll_pid.pid_i += roll_pid.ki * roll_pid.error;
  roll_pid.pid_d = roll_pid.kd * (imu.gyro_y * (-0.0610687023));
  roll_pid.output = roll_pid.pid_p + roll_pid.pid_i + roll_pid.pid_d;

  // Pitch PID
  pitch_pid.error = imu.angle_pitch_output - pitch_target;
  pitch_pid.pid_p = P_factor * pitch_pid.kp * pitch_pid.error;
  pitch_pid.pid_i += pitch_pid.ki * pitch_pid.error;
  pitch_pid.pid_d = pitch_pid.kd * (imu.gyro_x * (-0.0610687023));
  pitch_pid.output = pitch_pid.pid_p + pitch_pid.pid_i + pitch_pid.pid_d;

  // Yaw PID
  yaw_pid.error = imu.angle_yaw_output;
  yaw_pid.pid_p = yaw_pid.kp * yaw_pid.error;
  yaw_pid.pid_i += yaw_pid.ki * yaw_pid.error;
  yaw_pid.pid_d = yaw_pid.kd * (imu.gyro_z * (-0.0610687023));
  yaw_pid.output = yaw_pid.pid_p + yaw_pid.pid_i + yaw_pid.pid_d;

  // Reset integrators when throttle is zero
  if (input_THROTTLE == 0) {
    roll_pid.pid_i = 0;
    pitch_pid.pid_i = 0;
    yaw_pid.pid_i = 0;
    yaw_pid.output = 0;
    imu.angle_yaw_output = 0;
    imu.angle_yaw = 0;
  }
}

void calculateMotorOutputs() {
  // Calculate motor outputs with PID corrections
  ESCout_1 = input_THROTTLE + pitch_pid.output - roll_pid.output + yaw_pid.output;
  ESCout_2 = input_THROTTLE + pitch_pid.output + roll_pid.output - yaw_pid.output;
  ESCout_3 = input_THROTTLE - pitch_pid.output + roll_pid.output + yaw_pid.output;
  ESCout_4 = input_THROTTLE - pitch_pid.output - roll_pid.output - yaw_pid.output;

  // Constrain motor outputs
  ESCout_1 = constrain(ESCout_1, 1, 1199);
  ESCout_2 = constrain(ESCout_2, 1, 1199);
  ESCout_3 = constrain(ESCout_3, 1, 1199);
  ESCout_4 = constrain(ESCout_4, 1, 1199);
}

void sortMotorOutputs() {
  int arr[4] = {ESCout_1, ESCout_2, ESCout_3, ESCout_4};
  int temp_arr[4] = {ESCout_1, ESCout_2, ESCout_3, ESCout_4};
  
  // Sort the array
  for (int i = 0; i < 3; i++) {
    for (int j = i+1; j < 4; j++) {
      if (temp_arr[j] < temp_arr[i]) {
        int temp = temp_arr[i];
        temp_arr[i] = temp_arr[j];
        temp_arr[j] = temp;
      }
    }
  }

  // Calculate pulldown times
  pwm.pulldown_time[0] = temp_arr[0];
  pwm.pulldown_time[1] = temp_arr[1] - temp_arr[0];
  pwm.pulldown_time[2] = temp_arr[2] - temp_arr[1];
  pwm.pulldown_time[3] = temp_arr[3] - temp_arr[2];
  pwm.pulldown_time[4] = 1200 - temp_arr[3];

  // Ensure no zero times
  for (int i = 1; i < 5; i++) {
    if (pwm.pulldown_time[i] == 0) pwm.pulldown_time[i] = 1;
  }

  // Determine motor order
  bool orderFound[4] = {false};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (temp_arr[i] == arr[j] && !orderFound[j]) {
        pwm.order[i] = j;
        orderFound[j] = true;
        break;
      }
    }
  }

  // Copy to volatile array
  memcpy((void*)pwm.pulldown_time_temp, (void*)pwm.pulldown_time, sizeof(pwm.pulldown_time));

  // Start PWM cycle if not already running
  if (!pwm.timer_initialized) {
    timer1_write(80);
    pwm.timer_initialized = true;
  }
}

void loop() {
  Blynk.run();
  
  unsigned long currentTime = micros();
  if (currentTime - lastLoopTime < LOOP_INTERVAL) return;
  lastLoopTime = currentTime;

  readIMU();
  calculatePID();
  calculateMotorOutputs();
  sortMotorOutputs();

  // Debug output
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) {
    lastPrintTime = millis();
    Serial.printf("Thr:%4d | R:%4d P:%4d Y:%4d | Angles: R:%6.1f P:%6.1f Y:%6.1f\n",
      input_THROTTLE, input_ROLL, input_PITCH, input_YAW,
      imu.angle_roll_output, imu.angle_pitch_output, imu.angle_yaw_output);
  }
}
