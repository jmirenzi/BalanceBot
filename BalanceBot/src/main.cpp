#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <QuickPID.h>
 
#define I2C_SDA 33
#define I2C_SCL 32
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

#define ENC_COUNT_REV 620
#define PWM_Freq 5000
#define PWM_Resolution 8
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_Resolution) - 1);
#define sgn(x) ({ __typeof__(x) _x = (x); _x < 0 ? -1 : _x ? 1 : 0; })
template <class X, class M, class N, class O, class Q>
X map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const float Speed_Params[3] = {.250,0,0};
const float Speed_Params_tight[3] = {.50,0,0};
const float pitch_Params[3]         = {10.0,0,8};
const float pitch_Params_tight[3]   = {8.0,0.0000,4};
float pitch_setpoint=-7.0;

volatile long M1_wheel_pulse_count = 0;
volatile long M2_wheel_pulse_count = 0;
float M1_rpm = 0,M1speed_input=10,M1speed_response, M1speed_setpoint;
float M2_rpm = 0,M2speed_input=10,M2speed_response, M2speed_setpoint;
float pitch,pitch_response,pitch_response_mapped;
int interval_speedometer_ms = 100,M1_zero_count=0,M2_zero_count=0;
long previousMillis = 0,currentMillis = 0;

QuickPID M1_Speed(&M1_rpm, &M1speed_response, &M1speed_setpoint);
QuickPID M2_Speed(&M2_rpm, &M2speed_response, &M2speed_setpoint);
QuickPID pitch_PID(&pitch, &pitch_response, &pitch_setpoint);

TwoWire I2CBNO = TwoWire(0);

Adafruit_BNO055 bno = Adafruit_BNO055(55,BNO055_ADDRESS_A,&I2CBNO);
sensors_event_t sensor_event; 

void M1_pulse();
void M2_pulse();
float speedToPWM(float);
void updateMotorSpeeds();
void setMotorPWM();


void setup(void) 
{
  ledcSetup(M1_Speed_Chn, PWM_Freq, PWM_Resolution);
  ledcAttachPin(M1_Speed_Pin, M1_Speed_Chn);
  ledcSetup(M2_Speed_Chn, PWM_Freq, PWM_Resolution);
  ledcAttachPin(M2_Speed_Pin, M2_Speed_Chn);
  pinMode(M1_Dir1_Pin,OUTPUT);
  pinMode(M2_Dir1_Pin,OUTPUT);
  pinMode(M1_Dir2_Pin,OUTPUT);
  pinMode(M2_Dir2_Pin,OUTPUT);
  pinMode(M1_EncA_Pin , INPUT_PULLUP);
  pinMode(M1_EncB_Pin , INPUT);
  pinMode(M2_EncA_Pin , INPUT_PULLUP);
  pinMode(M2_EncB_Pin , INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_EncA_Pin), M1_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_EncA_Pin), M2_pulse, RISING);
  M1_Speed.SetTunings(Speed_Params[0],Speed_Params[1],Speed_Params[2]);
  M2_Speed.SetTunings(Speed_Params[0],Speed_Params[1],Speed_Params[2]);
  M1_Speed.SetMode(M1_Speed.Control::automatic);
  M2_Speed.SetMode(M2_Speed.Control::automatic);
  M1_Speed.SetOutputLimits(-50,50);
  M2_Speed.SetOutputLimits(-50,50);

  pitch_PID.SetTunings(pitch_Params[0],pitch_Params[1],pitch_Params[2]);
  pitch_PID.SetMode(pitch_PID.Control::automatic);
  pitch_PID.SetOutputLimits(-60,60); 
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  I2CBNO.begin(I2C_SDA, I2C_SCL, 100000);
  // Initialise the sensor 
  if(!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Starting");
}

void loop(){
  bno.getEvent(&sensor_event);
  pitch = sensor_event.orientation.y;
  
  currentMillis = millis();
  if (currentMillis - previousMillis > interval_speedometer_ms) {
  if(abs(pitch-pitch_setpoint)<10){
    pitch_PID.SetTunings(pitch_Params_tight[0],pitch_Params_tight[1],pitch_Params_tight[2]);
  }
  else{
    pitch_PID.SetTunings(pitch_Params[0],pitch_Params[1],pitch_Params[2]);
  }
  pitch_PID.Compute();
  M1speed_setpoint=pitch_response;
  M2speed_setpoint=pitch_response;

  // if (currentMillis - previousMillis > interval_speedometer_ms) {
    updateMotorSpeeds();
    M1_Speed.Compute();
    M2_Speed.Compute();
    
    previousMillis = currentMillis;
    // Serial.print("p:[");

    Serial.print(pitch-pitch_setpoint);Serial.print(",");
    Serial.print(pitch_response);Serial.print(",");
    // Serial.print(M1speed_response);Serial.print(",");
    Serial.print(M1_rpm);Serial.print(",");
    // Serial.print(M2speed_response);Serial.print(",");
    Serial.println(M2_rpm);


    // Serial.print(M1speed_setpoint);
    // Serial.print(",");
    // Serial.print(M1speed_input);
    // Serial.print(",");
    // Serial.print(M1speed_response);
    // Serial.print(",");
    // Serial.print(M1_rpm);
    // Serial.print(",");
    // Serial.print(M2speed_input);
    // Serial.print(",");
    // Serial.print(M2speed_response);
    // Serial.print(",");
    // Serial.println(M2_rpm);
    if (abs(pitch-pitch_setpoint)<60)
    setMotorPWM();
  else{ledcWrite(M1_Speed_Chn, 0);ledcWrite(M2_Speed_Chn,0);M1speed_input=0;M2speed_input=0;}
}
  }
  


float speedToPWM(float speed){
  if (abs(speed)<=1.0)
    return 0;
  int speed_range=100,min_response=50;
  return sgn(speed)*map_Generic(abs(speed),0,speed_range,min_response,255);
}

void updateMotorSpeeds(){
  if(M1_wheel_pulse_count!=0 || M1_zero_count>5){
      M1_rpm = (float)(M1_wheel_pulse_count*60000.0 / (ENC_COUNT_REV*(currentMillis - previousMillis)));
      M1_zero_count=0;
    }
    else{
      M1_zero_count++;
    }
    
    if(M2_wheel_pulse_count!=0 || M2_zero_count>5){
      M2_rpm = (float)(M2_wheel_pulse_count*60000.0 / (ENC_COUNT_REV*(currentMillis - previousMillis)));
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