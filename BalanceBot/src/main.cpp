#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <QuickPID.h>
/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the angular velocity in radians/second of a DC motor
 * with a built-in encoder (forward = positive; reverse = negative) 
 */
 
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 620
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define I2C_SCL 32
#define LOGIC_HIGH_PIN 35

#define M1_Speed_Pin 26
#define M1_Speed_Chn 0
#define M1_Dir1_Pin 13
#define M1_Dir2_Pin 12
#define M2_Speed_Pin 25
#define M2_Speed_Chn 1
#define M2_Dir1_Pin 14
#define M2_Dir2_Pin 27
#define M1_EncA_Pin 19 // interrupt
#define M2_EncA_Pin 5 // interrupt
#define M1_EncB_Pin 21 // direction
#define M2_EncB_Pin 18 // direction

#define PWM_Freq 5000
#define PWM_Resolution 8
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_Resolution) - 1);
#define sgn(x) ({ __typeof__(x) _x = (x); _x < 0 ? -1 : _x ? 1 : 0; })
template <class X, class M, class N, class O, class Q>
X map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.


const float Speed_Params[3] = {.50,0,0};
const float Speed_Params_tight[3] = {1.0,0,0};

float M1speed_input=10,M1speed_response, M1speed_setpoint;
float M2speed_input=10,M2speed_response, M2speed_setpoint;

float M1_rpm = 0;
float M2_rpm = 0;

QuickPID M1_Speed(&M1_rpm, &M1speed_response, &M1speed_setpoint);
QuickPID M2_Speed(&M2_rpm, &M2speed_response, &M2speed_setpoint);

// True = Forward; False = Reverse
boolean M1_Dir = true;
boolean M2_Dir = true;

void M1_pulse();
void M2_pulse();
 
// Keep track of the number of right wheel pulses
volatile long M1_wheel_pulse_count = 0;
volatile long M2_wheel_pulse_count = 0;
 
// One-second interval for measurements
int interval = 100,M1_zero_count=0,M2_zero_count=0;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment

 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


float speedToPWM(float speed){
  if (speed==0.0)
    return 0;
  int speed_range=100,min_response=100;
  return sgn(speed)*map_Generic(abs(speed),0,speed_range,min_response,255);
}

void updateMotorSpeeds(){
  if(M1_wheel_pulse_count!=0 || M1_zero_count>5){
      M1_rpm = (float)(M1_wheel_pulse_count*60000 / (ENC_COUNT_REV*(currentMillis - previousMillis)));
      M1_zero_count=0;
    }
    else{
      M1_zero_count++;
    }
    
    if(M2_wheel_pulse_count!=0 || M2_zero_count>5){
      M2_rpm = (float)(M2_wheel_pulse_count*60000 / (ENC_COUNT_REV*(currentMillis - previousMillis)));
      M2_zero_count=0;
    }
    else{
      M2_zero_count++;
    }

    M1_wheel_pulse_count = 0;
    M2_wheel_pulse_count = 0;

  if (abs(M1_rpm-M1speed_setpoint)>10){M1_Speed.SetTunings(Speed_Params[0],Speed_Params[1],Speed_Params[2]);}
  else{M1_Speed.SetTunings(Speed_Params_tight[0],Speed_Params_tight[1],Speed_Params_tight[2]);}
  if (abs(M2_rpm-M2speed_setpoint)>10){M2_Speed.SetTunings(Speed_Params[0],Speed_Params[1],Speed_Params[2]);}
  else{M2_Speed.SetTunings(Speed_Params_tight[0],Speed_Params_tight[1],Speed_Params_tight[2]);}
}

void setMotorPWM(){
  M1speed_input+=M1speed_response;
  M2speed_input+=M2speed_response;

  if (abs(M1speed_input)>100)
    M1speed_input=100*sgn(M1speed_input);
  if (abs(M2speed_input)>100)
    M2speed_input=100*sgn(M2speed_input);
  
  if(M1speed_input>0){digitalWrite(M1_Dir1_Pin,HIGH);digitalWrite(M1_Dir2_Pin,LOW);}
  else{digitalWrite(M1_Dir2_Pin,HIGH);digitalWrite(M1_Dir1_Pin,LOW);}

  if(M2speed_input>0){digitalWrite(M2_Dir1_Pin,HIGH);digitalWrite(M2_Dir2_Pin,LOW);}
  else{digitalWrite(M2_Dir2_Pin,HIGH);digitalWrite(M2_Dir1_Pin,LOW);}

  ledcWrite(M1_Speed_Chn, abs(speedToPWM(M1speed_input)));
  ledcWrite(M2_Speed_Chn, abs(speedToPWM(M2speed_input)));
}
int set_point_count=0;
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
  ledcAttachPin(M1_Speed_Pin, M1_Speed_Chn);
  ledcSetup(M2_Speed_Chn, PWM_Freq, PWM_Resolution);
  ledcAttachPin(M2_Speed_Pin, M2_Speed_Chn);
  pinMode(M1_Dir1_Pin,OUTPUT);
  pinMode(M2_Dir1_Pin,OUTPUT);
  pinMode(M1_Dir2_Pin,OUTPUT);
  pinMode(M2_Dir2_Pin,OUTPUT);

  pinMode(LOGIC_HIGH_PIN,OUTPUT);
  digitalWrite(M1_Dir1_Pin,HIGH);
  digitalWrite(M1_Dir2_Pin,LOW);
  digitalWrite(M2_Dir1_Pin,HIGH);
  digitalWrite(M2_Dir2_Pin,LOW);

  digitalWrite(LOGIC_HIGH_PIN,HIGH);
 
  // Set pin states of the encoder
  pinMode(M1_EncA_Pin , INPUT_PULLUP);
  pinMode(M1_EncB_Pin , INPUT);
  pinMode(M2_EncA_Pin , INPUT_PULLUP);
  pinMode(M2_EncB_Pin , INPUT);
  M1_Speed.SetTunings(Speed_Params[0],Speed_Params[1],Speed_Params[2]);
  M2_Speed.SetTunings(Speed_Params[0],Speed_Params[1],Speed_Params[2]);
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(M1_EncA_Pin), M1_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_EncA_Pin), M2_pulse, RISING);
  M1speed_setpoint=0;M2speed_setpoint=0;
  M1_Speed.SetMode(M1_Speed.Control::automatic);
  M2_Speed.SetMode(M2_Speed.Control::automatic);
  M1_Speed.SetOutputLimits(-50,50);
  M2_Speed.SetOutputLimits(-50,50);
}
 
void loop() {
 
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  
 
  if (currentMillis - previousMillis > interval) {
    set_point_count++;
    if (set_point_count%50==0 && M1speed_setpoint<140){M1speed_setpoint+=10;M2speed_setpoint+=10;}
    updateMotorSpeeds();
    M1_Speed.Compute();
    M2_Speed.Compute();
    setMotorPWM();

    // Serial.print(" Resp: ");
    // Serial.print(speedToPWM(M1speed_input));
    // Serial.print("  ");
    Serial.print(M1speed_setpoint);
    Serial.print(",");
    Serial.print(M1speed_input);
    Serial.print(",");
    Serial.print(M1speed_response);
    Serial.print(",");
    // Serial.print(" Speed: ");
    Serial.print(M1_rpm);
    // Serial.print(" RPM");
    // Serial.print(" Resp: ");
    Serial.print(",");
    // Serial.print(speedToPWM(M2speed_input));
    // Serial.print("  ");
    Serial.print(M2speed_input);
    Serial.print(",");
    Serial.print(M2speed_response);
    Serial.print(",");
    // Serial.print(" Speed: ");
    Serial.println(M2_rpm);
    // Serial.println(" RPM");

   previousMillis = currentMillis;
  }
}
 
// Increment the number of pulses by 1
void M1_pulse() {
  if (digitalRead(M1_EncB_Pin))
    M1_wheel_pulse_count++;
  else
    M1_wheel_pulse_count--;
}

void M2_pulse() {
  if (digitalRead(M2_EncB_Pin))
    M2_wheel_pulse_count--;
  else
    M2_wheel_pulse_count++;
}