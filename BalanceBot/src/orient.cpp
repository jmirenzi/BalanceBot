#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <QuickPID.h>

#define sgn(x) ({ __typeof__(x) _x = (x); _x < 0 ? -1 : _x ? 1 : 0; })

template <class X, class M, class N, class O, class Q>
X map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define I2C_SDA 33
#define I2C_SCL 32
#define LOGIC_HIGH_PIN 35

#define M1_Speed_Pin 26
#define M1_Speed_Chn 0
#define M1_Dir1_Pin 13
#define M1_Dir2_Pin 12
// #define M1_EnAint_Pin 
// #define M1_EnBDIR_Pin
#define M2_Speed_Pin 25
#define M2_Speed_Chn 1
#define M2_Dir1_Pin 14
#define M2_Dir2_Pin 27
// #define M2_EnAint_Pin
// #define M2_EnBDIR_Pin

#define PWM_Freq 5000
#define PWM_Resolution 8
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_Resolution) - 1);

const float roll_Params[3]          = {1.0,0,0};
const float pitch_Params[3]         = {5.0,0,0};
const float pitch_Params_tight[3]   = {15.0,0,0};
const float yaw_Params[3]           = {1.0,0,0};

float roll,pitch,yaw,roll_response,pitch_response,yaw_response,roll_setpoint,pitch_setpoint,yaw_setpoint,pitch_response_mapped;

QuickPID roll_PID(&roll, &roll_response, &roll_setpoint);
QuickPID pitch_PID(&pitch, &pitch_response, &pitch_setpoint);
QuickPID yaw_PID(&yaw, &yaw_response, &yaw_setpoint);

TwoWire I2CBNO = TwoWire(0);

Adafruit_BNO055 bno = Adafruit_BNO055(55,BNO055_ADDRESS_A,&I2CBNO);
sensors_event_t sensor_event; 

const int min_response = 250;
const int response_range = 255;

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

  pinMode(LOGIC_HIGH_PIN,OUTPUT);

  // pinMode(M1_EnAint_Pin,INPUT);
  // pinMode(M1_EnBDIR_Pin,INPUT);
  // pinMode(M2_EnAint_Pin,INPUT);
  // pinMode(M2_EnBDIR_Pin,INPUT);


  digitalWrite(M1_Dir1_Pin,HIGH);
  digitalWrite(M1_Dir2_Pin,LOW);
  digitalWrite(M2_Dir1_Pin,HIGH);
  digitalWrite(M2_Dir2_Pin,LOW);

  digitalWrite(LOGIC_HIGH_PIN,HIGH);

  roll_PID.SetTunings(roll_Params[0],roll_Params[1],roll_Params[2]);
  pitch_PID.SetTunings(pitch_Params[0],pitch_Params[1],pitch_Params[2]);
  yaw_PID.SetTunings(yaw_Params[0],yaw_Params[1],yaw_Params[2]);
  roll_setpoint=0;pitch_setpoint=-12;yaw_setpoint=0;
  roll_PID.SetMode(roll_PID.Control::automatic);
  pitch_PID.SetMode(pitch_PID.Control::automatic);
  yaw_PID.SetMode(yaw_PID.Control::automatic);
  roll_PID.SetOutputLimits(-1*response_range,response_range);
  pitch_PID.SetOutputLimits(-1*response_range,response_range); 
  yaw_PID.SetOutputLimits(-1*response_range,response_range);
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  I2CBNO.begin(I2C_SDA, I2C_SCL, 100000);
  // Initialise the sensor 
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  
  
}

void loop(void){
  bno.getEvent(&sensor_event);
  roll = sensor_event.orientation.z;
  pitch = sensor_event.orientation.y;
  yaw = sensor_event.orientation.x;

  if(abs(pitch-pitch_setpoint)<10){
    pitch_PID.SetTunings(pitch_Params_tight[0],pitch_Params_tight[1],pitch_Params_tight[2]);
  }
  else{
    pitch_PID.SetTunings(pitch_Params[0],pitch_Params[1],pitch_Params[2]);
  }


  pitch_PID.Compute();
  
  
  pitch_response_mapped = sgn(pitch_response)*map_Generic(abs(pitch_response),0,response_range,min_response,255);
  // Serial.printf("orient:(%f, %f, %f)  resp:(%f, %f, %f)\n",roll,pitch,yaw,roll_response,pitch_response_mapped,yaw_response);

  Serial.printf("(%f, %f, %f)\n",pitch,pitch_response_mapped,pitch_response);
  if(pitch_response>0){
    digitalWrite(M1_Dir1_Pin,HIGH);
    digitalWrite(M1_Dir2_Pin,LOW);
    digitalWrite(M2_Dir1_Pin,HIGH);
    digitalWrite(M2_Dir2_Pin,LOW);
  }
  else{
    digitalWrite(M1_Dir2_Pin,HIGH);
    digitalWrite(M1_Dir1_Pin,LOW);
    digitalWrite(M2_Dir2_Pin,HIGH);
    digitalWrite(M2_Dir1_Pin,LOW);
  }
  if(abs(pitch)<50){
    ledcWrite(M1_Speed_Chn, abs(pitch_response));
    ledcWrite(M2_Speed_Chn, abs(pitch_response));
  }
}