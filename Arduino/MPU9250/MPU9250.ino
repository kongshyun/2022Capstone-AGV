
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

unsigned long t_now;  //현재 측정 주기 시간
unsigned long t_prev; //이전 측정 주기 시간

float filtered_angle;
float pitch_acc;
float GryAngle;

float dt;// = 0.048;


void initDT(){
  t_prev = millis();
}

void calcDT(){
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0; 
  t_prev = t_now;
}


void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) gx / 131;//* 250 / 32768;
    Gxyz[1] = (double) gy / 131;//* 250 / 32768;
    Gxyz[2] = (double) gz / 131;//* 250 / 32768;
}

void Filter(){
  pitch_acc = atan2(Axyz[1],sqrt(Axyz[0]*Axyz[0] + Axyz[2]*Axyz[2]))*180/PI;
  GryAngle = GryAngle + Gxyz[0] * dt;
  const float alpha = 0.98;
  float tmp_angle; //이전 필터 각도
  tmp_angle = filtered_angle + Gxyz[0]*dt;
  filtered_angle = alpha*tmp_angle + (1.0-alpha)*pitch_acc;
}

//모터 핀 설정
int M1P1 = 3, M1P2 = 9;
int M2P1 = 11, M2P2 = 10;

float compX;

//PID제어 관련 변수

float kp = 14;//13.5;            //19;//32;
float ki = 95;//140;            //220;//23;
float kd = 1.22;//1.22;           //1.98;//0.8;

float pitch_target_angle = -4.2;
float pitch_prev_angle = 0.0;

float pitch_iterm;
float output;

//PID제어 함수
void myPID(float& setpoint, float& input, float& prev_input, 
float& kp, float& ki, float&kd, float& iterm, float& output)
{
  float error;
  float dInput;
  float pterm, dterm;

  error = input - setpoint;
  dInput = input - prev_input;
  prev_input = input;

  pterm = kp * error;
  iterm += ki * error * dt;
  dterm = kd * (dInput / dt);

  output = pterm + iterm + dterm;

}

//각도 PID 계산 함수
void calcXtoSTDPID()
{
  myPID(pitch_target_angle, filtered_angle, 
  pitch_prev_angle, kp, ki, kd, pitch_iterm, output);
}





void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  accelgyro.initialize();
  
  accelgyro.setDLPFMode(MPU9250_DLPF_BW_20);  // 저역통과필터
 
  pinMode(M1P1,OUTPUT);
  pinMode(M1P2,OUTPUT);
  pinMode(M2P1,OUTPUT);
  pinMode(M2P2,OUTPUT);

  initDT();

}



void loop() {

   
    getAccel_Data();
    getGyro_Data();
    
    calcDT();
    
    Filter();
    
    calcXtoSTDPID();         //각도값 PID제어
    
    MotorControl();
    
    Serial.println("======================================");
    Serial.print("ax = ");
    Serial.println(Axyz[0]);
    Serial.print("ay = ");
    Serial.println(Axyz[1]);
    
    Serial.print("az = ");
    Serial.println(Axyz[2]);
    Serial.println("-----------------------");
    Serial.print("gx = ");
    Serial.println(Gxyz[0]);
    Serial.print("gy = ");
    Serial.println(Gxyz[1]);
    Serial.print("gz = ");
    Serial.println(Gxyz[2]);
    delay(100);
      

}

void MotorControl(){
  if(output<255 && output >= 0)
      {
        analogWrite(M1P2,output);
        analogWrite(M1P1,0);
      
        analogWrite(M2P1,output);
        analogWrite(M2P2,0);
      }

      else if(output>255){
        analogWrite(M1P2,255);
        analogWrite(M1P1,0);
      
        analogWrite(M2P1,255);
        analogWrite(M2P2,0);
      }

      else if(output>=-255 && output < 0)
        {
          analogWrite(M1P1,-1 * output);
          analogWrite(M1P2,0);
      
          analogWrite(M2P2,-1 * output);
          analogWrite(M2P1,0);
        }

       else if(output<-255){
          analogWrite(M1P1,255);
          analogWrite(M1P2,0);
      
          analogWrite(M2P2,255);
          analogWrite(M2P1,0);
       }
}
