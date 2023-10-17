#include <SimpleFOC.h>
#include <SPI.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include <Preferences.h>

// REMEMBER TO SELECT - FLASH SIZE 4MB (SKETCH: 4032KB, FS: 64KB)

Preferences prefs;
const char * MOTOR_ZERO_KEY = "zero";
const char * MOTOR_DIR_KEY = "dir";
const char * MOTOR_ANG_KEY = "ang";
const char * DRIVER_INDEX_KEY = "index";

BLDCMotor motor = BLDCMotor(11);
// BLDCMotor motor = BLDCMotor(14);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_PWMH_A, PIN_PWML_A, PIN_PWMH_B, PIN_PWML_B, PIN_PWMH_C, PIN_PWML_C);
MagneticSensorMT6701SSI sensor(PIN_HALL_CS);
InlineCurrentSense current_sense = InlineCurrentSense(CURR_SENSE_RES, CURR_SENSE_GAIN, PIN_IOUT_A, NOT_SET, PIN_IOUT_B);

Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
// void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
// void doVelocity(char* cmd) { command.scalar(&motor.velocity_limit, cmd); }

void onMotor(char* cmd){command.motor(&motor, cmd);}
void onConfig(char* cmd){
    if(cmd[0] == 'C') {
      Serial.println("Clearing preferences...");
      prefs.clear();
    }
    if(cmd[0] == 'H') {
      float angle = sensor.getAngle() * (motor.sensor_direction == Direction::CW ? 1 : -1);
      prefs.putFloat(MOTOR_ANG_KEY, angle);
      motor.sensor_offset = angle;
      motor.target = 0;
    }
    if(cmd[0] == 'P') {
      // motor.target = motor.shaft_angle;
      motor.target = sensor.getAngle();
      motor.controller = MotionControlType::angle;
    }
    if(cmd[0] == 'T') {
      motor.target = 0;
      motor.controller = MotionControlType::torque;
    }
    if(cmd[0] == 'F') {
      if (cmd[1] == '1') digitalWrite(PIN_LED, LOW);
      else digitalWrite(PIN_LED, HIGH);
    }
    if(cmd[0] == 'I') {
      if (cmd[1] == command.eol) {
        int8_t index = prefs.getChar(DRIVER_INDEX_KEY, -1);
        Serial.println(index);
      } else {
        prefs.putChar(DRIVER_INDEX_KEY, String(cmd[1]).toInt());
      }
    }
  }

unsigned long old_time = millis();
int n = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  delay(1000);
  digitalWrite(PIN_LED, HIGH);
  
  Serial.begin(115200);
  // while(!Serial);
  
  prefs.begin("haptics");
  
  SimpleFOCDebug::enable();
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable monitor at first
  
  sensor.init(&SPI);
  motor.linkSensor(&sensor);

  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = driver.voltage_power_supply * 0.95;
  driver.init();

  current_sense.linkDriver(&driver);
  motor.linkDriver(&driver);

  motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 1000;
  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f; // 1ms default
  motor.LPF_current_d.Tf = 0.002f; // 1ms default


  // velocity PI controller parameters
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  // velocity low pass filtering time constant
  // the lower the less filtered
  // motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 1;
  motor.P_angle.I = 0;
  motor.P_angle.D = .01;
  

  motor.voltage_limit = 5;   // [V]
  motor.velocity_limit = 1000; // [rad/s]
  motor.voltage_sensor_align = motor.voltage_limit*.7; //need to make this a bit higher since I'm using a gimbal motor
  
  // init motor hardware
  motor.init();
  
  // init current sense
  // current_sense.addPin(PIN_VCC_SENSE);
  // current_sense.addPin(25); //TODO: Add DC bus current sense pin to pinmap
  current_sense.init();

  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  
  current_sense.gain_a *=-1;

  float temp = prefs.getFloat(MOTOR_ZERO_KEY, NOT_SET);
  if (temp == NOT_SET) {
    wait_for_calibration();
  } else {
    motor.zero_electric_angle = prefs.getFloat(MOTOR_ZERO_KEY, 0);
    motor.sensor_direction = static_cast<Direction>(prefs.getChar(MOTOR_DIR_KEY, 0));
    motor.sensor_offset = prefs.getFloat(MOTOR_ANG_KEY, 0);
    motor.target = 0;
    
    current_sense.skip_align = true;
    motor.initFOC();
  }

  command.add('M', onMotor, "motor");
  command.add('C', onConfig, "config");
}

void loop() {
  unsigned long curr_time = millis();
  n++;
  if (curr_time > old_time + 50) {
    old_time = curr_time;
    n = 0;
  }

  motor.loopFOC();
  
  // open  loop angle movements
  // using motor.voltage_limit and motor.velocity_limit
  // motor.move(target_angle);
  motor.move();

  motor.monitor();
  
  // user communication
  command.run();

}

void wait_for_calibration() {
  Serial.println("Waiting for calibration...");
  float old_voltage_limit = motor.voltage_limit;
  motor.voltage_limit = motor.voltage_sensor_align;
  motor.sensor_direction = UNKNOWN; // Force discovery of motor direction

  // motor.move(0);
  pinMode(PIN_LED, INPUT_PULLUP);
  while(digitalRead(PIN_LED)) {
    delay(100);
  }
  pinMode(PIN_LED, OUTPUT);
  motor.initFOC();

  prefs.putFloat(MOTOR_ZERO_KEY, motor.zero_electric_angle);
  prefs.putChar(MOTOR_DIR_KEY, static_cast<uint8_t>(motor.sensor_direction));

  motor.voltage_limit = old_voltage_limit;
}