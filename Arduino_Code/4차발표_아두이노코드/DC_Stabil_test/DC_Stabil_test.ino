#include <PIDController.h>       // PID 라이브러리  
#include <Wire.h>    
#include <TimerOne.h>            // 타이머 인터럽트 라이브러리


#include <ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include<std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include<ros/time.h>

PIDController Motor1;         // PID 함수 이름 선언 (포지션 LEFT)
PIDController Motor2;
PIDController Motor3;

double theta_x, theta_y, theta_z;
volatile long int encoder_pos_1 = 0;   // 엔코더 신호 저장 변수
volatile long int encoder_pos_2 = 0;   // 오른쪽
volatile long int encoder_pos_3 = 0;   

long int          last_millis = 0;



long int   last_encoder_pos_1 = 0;
int             motor_value_1 = 0;
double             SetPoint_1 = 0;

long int   last_encoder_pos_2 = 0;
int             motor_value_2 = 0;
double             SetPoint_2 = 0;

long int   last_encoder_pos_3 = 0;
int             motor_value_3 = 0;
double             SetPoint_3 = 0;

int                     dir_1 = 0;     // 방향 결정
int                     dir_2 = 0;
int                     dir_3 = 0;  

#define               Forward   1
#define              Backward   2
#define                  Stop   0
#define                  LEFT   1
#define                 RIGHT   2


#define         encoderPinA_3   21      // pin  21 
#define         encoderPinB_3   20      // pin  20
#define         encoderPinA_2   19      // pin  19 
#define         encoderPinB_2   18      // pin  18
#define         encoderPinA_1   2       // pin  2 
#define         encoderPinB_1   3       // pin  3
#define          MOTOR_1R_PWM   5       // pin  5  OCR3A
#define          MOTOR_1L_PWM   6       // pin  6  OCR4A
#define          MOTOR_2R_PWM   7       // pin  7  OCR4B
#define          MOTOR_2L_PWM   8       // pin  8  OCR4C
#define          MOTOR_3R_PWM   9       // pin  9  OCR2B
#define          MOTOR_3L_PWM   10      // pin  10 OCR2A

#define                Kp_posA   200     
#define                Ki_posA   0.0001       
#define                Kd_posA   6000   

#define                Kp_posB   2000    
#define                Ki_posB   0    
#define                Kd_posB   100  

#define                Kp_posC   300    
#define                Ki_posC   2       
#define                Kd_posC   10000   

 const float pi = 3.141592;              // 파이
const float g = 9.81;                   // 중력가속도
      float av_dt =    0;

const float ratio =    360./139./52.;// 360도,기어비,ppr 34*2 채널



float a;                                // accel 현재 가속도
float theta;                            // 보정각도 프로파일 세타

float roll;
float pitch;

unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임
unsigned long sttime;

const int top = 255;



ros::NodeHandle  nh;

geometry_msgs::PointStamped roll_pitch_theta;
ros::Publisher imu_data("imu_data", &roll_pitch_theta);


// cmd_vel 콜백 함수
void EBimu_callback(const geometry_msgs :: PointStamped& imu) {
  roll=imu.point.x;
  pitch=imu.point.y;
  theta=imu.point.z;
  
  roll_pitch_theta.point.x=roll;
  roll_pitch_theta.point.y=pitch;
  roll_pitch_theta.point.z=theta;
  imu_data.publish(&roll_pitch_theta);//제대로 sub됐는지 pub해서 확인 
}

// subscriber
ros::Subscriber<geometry_msgs::PointStamped> imu("imu",EBimu_callback);


void setup() {

   Serial.begin(57600);
   Wire.begin(20);
   //Wire.onReceive(dataReceive);

   pinMode(encoderPinA_1,    INPUT_PULLUP);   // 엔코더 핀 0~5
   attachInterrupt(0, doEncoderA1, CHANGE);
   pinMode(encoderPinB_1,    INPUT_PULLUP);
   attachInterrupt(1, doEncoderB1, CHANGE);
   pinMode(encoderPinA_2,    INPUT_PULLUP);
   attachInterrupt(4, doEncoderA2, CHANGE);
   pinMode(encoderPinB_2,    INPUT_PULLUP);
   attachInterrupt(5, doEncoderB2, CHANGE);
   pinMode(encoderPinA_3,    INPUT_PULLUP);
   attachInterrupt(2, doEncoderA3, CHANGE);
   pinMode(encoderPinB_3,    INPUT_PULLUP);
   attachInterrupt(3, doEncoderB3, CHANGE);

     
   pinMode(          MOTOR_1R_PWM, OUTPUT);  
   pinMode(          MOTOR_1L_PWM, OUTPUT);
   pinMode(          MOTOR_2R_PWM, OUTPUT);
   pinMode(          MOTOR_2L_PWM, OUTPUT);  
   pinMode(          MOTOR_3R_PWM, OUTPUT);
   pinMode(          MOTOR_3L_PWM, OUTPUT);   

   
   TCCR3A = bit(COM3A1) | bit(WGM30);
   TCCR3B = bit(WGM32)  | bit(CS30);
   TCCR4A = bit(COM4A1) | bit(COM4B1) | bit(COM4C1) | bit(WGM40);
   TCCR4B = bit(WGM42)  | bit(CS40);
   TCCR2A = bit(COM2A1) | bit(COM2B1) | bit(WGM21)  | bit(WGM20);
   TCCR2B = bit(CS20); 
   
   
   Motor1.begin();                           //pid 라이브러리 따라서 선언
   Motor1.tune(Kp_posA, Ki_posA, Kd_posA);      //pid 계수
   Motor1.limit(-top, top);                  //pid 범위 제한
   Motor1.setpoint(0);                       //pid 목표값
   
   Motor2.begin();    
   Motor2.tune(Kp_posB, Ki_posB, Kd_posB);    
   Motor2.limit(-top, top);
   Motor2.setpoint(0);

   Motor3.begin();    
   Motor3.tune(Kp_posC, Ki_posC, Kd_posC);    
   Motor3.limit(-top, top);
   Motor3.setpoint(0);   
     
  nh.initNode();
  
  nh.subscribe(imu);
  nh.advertise(imu_data);

}

void loop(){
  nh.spinOnce();
  Plate_angle(roll+theta,pitch,0);
}////////////////////////////////////////////////////////////////////////////////////////





void degree_control(){                                          // 각도 제어
  
     float motorDeg_1 = float(encoder_pos_1)*ratio;             // 현재 각도 계산, 각도는 엔코더 신호 * 기어비 * PPR
     float motorDeg_2 = float(encoder_pos_2)*ratio;
     float motorDeg_3 = float(encoder_pos_3)*ratio;
     
     motor_value_1 = Motor1.compute(motorDeg_1);             //현재 각도를 PID 함수에 넣고 motor_value로 계산된 pid를 꺼내온다.
     motor_value_2 = Motor2.compute(motorDeg_2);
     motor_value_3 = Motor3.compute(motorDeg_3);
     
     dir_1 = motor_direction(motor_value_1);                    //motor_value의 부호를 보고 방향을 판단한다.
     dir_2 = motor_direction(motor_value_2);
     dir_3 = motor_direction(motor_value_3);
     
     motor_controlA((dir_1 > 0 )?Forward:Backward, min(abs(motor_value_1),top));  
     motor_controlB((dir_2 > 0 )?Forward:Backward, min(abs(motor_value_2),top));
     motor_controlC((dir_3 > 0 )?Forward:Backward, min(abs(motor_value_3),top));
     
}

int motor_direction(double pid){                 // 모터 방향 결정하는 함수
  int Dir;
  if(pid > 0) Dir =  1;
  if(pid < 0) Dir = -1;
  return Dir;
}


void motor_controlA(int motor_direction, int degree){  

    if(motor_direction == Forward){ 
    OCR3A  = degree;
    OCR4A  = 0; 
    }
    if(motor_direction == Backward){  
    OCR3A  = 0;
    OCR4A  = degree; 
    }
}

void motor_controlB(int motor_direction, int degree){  

    if(motor_direction == Forward){ 
    OCR4B  = degree;
    OCR4C  = 0; 
    }
    if(motor_direction == Backward){  
    OCR4B  = 0;
    OCR4C  = degree; 
    }
}
void motor_controlC(int motor_direction, int degree){  

    if(motor_direction == Forward){ 
    OCR2B  = degree;
    OCR2A  = 0; 
    }
    if(motor_direction == Backward){  
    OCR2B  = 0;
    OCR2A  = degree;
    }
}
  


void timecycle(){//ㅡㅡㅡㅡㅡㅡㅡㅡ
  float t;                            
  t = micros() - loop_start_time;
  dt = t;
                                       
}//timecycle
///////==============================================================================/////
void Plate_angle(float Roll, float Pitch, float height){

  Calculate_Roll_Pitch(Roll, Pitch, height);

  
  Motor1.setpoint(theta_x);
  Motor2.setpoint(theta_y);
  Motor3.setpoint(theta_z);
  
}//Plate_angle//



//////==============================================================================//////
void Calculate_Roll_Pitch(float Target_roll, float Target_pitch, float Target_height) {//ㅡㅡㅡㅡㅡㅡ
  // Target_roll = 원하는 롤 값,  Target_pitch = 원하는 피치값                        
  // link_A = 링크 1의 길이 , link_B = 링크2의 길이                                     
                                                                                     
  float link_A1 = 22.5; //
  float link_A2 = 22.5; //
  float link_A3 = 22.5; //
  float link_B1 = 60; //
  float link_B2 = 60; //
  float link_B3 = 60; //
  float height_data = 58 + Target_height; // 

  ///////////////////////////////////////////////
  
  float yaw = 0;     //실험용 yaw

  ///////////////////////////////////////////////
  
  float Length_high, Length_low;

  Length_high = 110; //
  Length_low  = 42.5*2;  //
  
  Length_high = Length_high*sqrt(3)/2;
  Length_low  = Length_low*sqrt(3)/2;
  //이제부터 배열을 사용한다.
  float P[] = {0, 0, height_data}; //T(3 by 1  행렬임)

  float b1[] = { Length_high / sqrt(3),      0 ,              0}; //T
  float b2[] = { -Length_high / (2 * sqrt(3)),  Length_high / 2 ,  0}; //T
  float b3[] = { -Length_high / (2 * sqrt(3)), -Length_high / 2 ,  0}; //T
  float a1[] = { Length_low / sqrt(3),       0 ,              0}; //T
  float a2[] = { -Length_low / (2 * sqrt(3)),   Length_low / 2 ,   0}; //T
  float a3[] = { -Length_low / (2 * sqrt(3)),  -Length_low / 2 ,   0}; //T


  float x = Target_roll  * (PI / 180); // 각도를 라디안으로 변환함
  float y = Target_pitch * (PI / 180);
      yaw = yaw          * (PI / 180);

  float tf_yaw1[3] = {cos(yaw), -sin(yaw),     0} ; //T
  float tf_yaw2[3] = {sin(yaw),  cos(yaw),     0} ; //T
  float tf_yaw3[3] = {       0,         0,     1} ; //T

  x   =   x*cos(yaw) + y*sin(yaw);
  y   =  -x*sin(yaw) + y*cos(yaw);


 


  float R[3][3] = {  {cos(y), sin(x)*sin(y), cos(x)*sin(y)}, {0, cos(x), -sin(x)}, 
  { -sin(y), sin(x)*cos(y), cos(x)*cos(y)}  };

  int i, j, k; // for 문 변수 선언

  float q1[3] = {0, 0, 0} ; //T
  float q2[3] = {0, 0, 0} ; //T
  float q3[3] = {0, 0, 0} ; //T

  for (i = 0; i < 3; i++) {
    for (k = 0; k < 3; k++) {
      q1[i] += R[i][k] * b1[k]; // q1 = R*b1
      q2[i] += R[i][k] * b2[k]; // q2 = R*b2
      q3[i] += R[i][k] * b3[k]; // q3 = R*b3
    }
  }
  q1[2] += height_data;
  q2[2] += height_data;
  q3[2] += height_data;
  ////////////////////////////first//////////////////////////////////
  float x1 = q1[0] - a1[0];
  float x2 = q1[1] - a1[1];
  float x3 = q1[2] - a1[2];
  float x_data_1, y_data_1;
  y_data_1 = x3;
  x_data_1 = sqrt(pow(x1, 2) + pow(x2, 2));
  int dot_data_1 = x1 * a1[0] + x2 * a1[1] + x3 * a1[2]; // 내적
  if (dot_data_1 < 0) x_data_1 = - x_data_1;
  ////////////////////////////second///////////////////////////////////
  float y1 = q2[0] - a2[0];
  float y2 = q2[1] - a2[1];
  float y3 = q2[2] - a2[2];
  float x_data_2, y_data_2;
  y_data_2 = y3;
  x_data_2 = sqrt(pow(y1, 2) + pow(y2, 2));
  int dot_data_2 = y1 * a2[0] + y2 * a2[1] + y3 * a2[2]; // 내적
  if (dot_data_2 < 0) x_data_2 = - x_data_2;
  ////////////////////////////third///////////////////////////////////
  float z1 = q3[0] - a3[0];
  float z2 = q3[1] - a3[1];
  float z3 = q3[2] - a3[2];
  float x_data_3, y_data_3;
  y_data_3 = z3;
  x_data_3 = sqrt(pow(z1, 2) + pow(z2, 2));
  int dot_data_3 = z1 * a3[0] + z2 * a3[1] + z3 * a3[2]; // 내적
  if (dot_data_3 < 0) x_data_3 = - x_data_3;

  // float theta_x ,theta_y, theta_z; // 지역변수 아닌 전역변수로 선언하기 위해 위로 갑니다
    theta_x = length_to_motor_degree(x_data_1, y_data_1, link_A1, link_B1);
    theta_y = length_to_motor_degree(x_data_2, y_data_2, link_A2, link_B3);
    theta_z = length_to_motor_degree(x_data_3, y_data_3, link_A3, link_B3);

    int initial_angle_error_x = 0; 
    int initial_angle_error_y = 0;
    int initial_angle_error_z = 0;

    theta_x += initial_angle_error_x;//초기 모터 위치 값 에러 보정
    theta_y += initial_angle_error_y;
    theta_z += initial_angle_error_z;

    theta_x += 0; //모터가 세워져있기 때문에 
    theta_y += 0;
    theta_z += 0;   
    theta_x = constrain(theta_x, -80, 80); //위치 제한
    theta_y = constrain(theta_y, -80, 80);
    theta_z = constrain(theta_z, -80, 80);
                                                                
}/////////함수 끝//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

float length_to_motor_degree(float x_data, float y_data, float link_A, float link_B) {
                                                                                                                  
  float thb, thbb, tha, thaa;                                                                                       
  float x = x_data;
  float y = y_data;
  float A = link_A;
  float B = link_B;
  float M = ((x * x) + (y * y) - (A * A) - (B * B)) / (2 * A * B); //중간 계산값

  tha = atan2(sqrt(abs(1 - pow(M, 2))), M);
  thb = atan2(y, x) - atan2(B * sin(tha), (A + (B * cos(tha))));
  thaa = atan2(-sqrt(abs(1 - pow(M, 2))), M);
  thbb = atan2(y, x) - atan2(B * sin(thaa), (A + (B * cos(thaa))));

  float theta1 = thb * 180 / PI;
  float theta2 = thbb * 180 / PI;

  // 어떤 각도를 선택할 건지 알고리즘 추가해야함.

  return theta1;                                                                                                   
                                                                                                               
}////////////////////////////////////////////////////////////////////


void doEncoderA1(){ 
  int dir_pos  = (digitalRead(encoderPinA_1)==digitalRead(encoderPinB_1))?1:-1;   // 엔코더 신호 감지, A,B 함수로 모든 EDGE를 잡아 CW ,CCW 판단
  encoder_pos_1 += dir_pos;
}
void doEncoderB1(){ 
  int dir_pos    = (digitalRead(encoderPinA_1)==digitalRead(encoderPinB_1))?-1:1;  //
  encoder_pos_1 += dir_pos;
}

void doEncoderA2(){ 
  int dir_pos     = (digitalRead(encoderPinA_2)==digitalRead(encoderPinB_2))?1:-1;
  encoder_pos_2  += dir_pos;
}
void doEncoderB2(){ 
  int dir_pos    = (digitalRead(encoderPinA_2)==digitalRead(encoderPinB_2))?-1:1;
  encoder_pos_2 += dir_pos;
}

void doEncoderA3(){ 
  int dir_pos     = (digitalRead(encoderPinA_3)==digitalRead(encoderPinB_3))?1:-1;
  encoder_pos_3  += dir_pos;
}
void doEncoderB3(){ 
  int dir_pos    = (digitalRead(encoderPinA_3)==digitalRead(encoderPinB_3))?-1:1;
  encoder_pos_3 += dir_pos;
}


void Calculate_delay( float target_time, float number ){//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
                                                            //target_time (ms)
                                                            //number (횟수)  
  float cycle_end_time = millis();                            //사이클 끝나는 시각
  float cycle_time;                                           //측정된 1 사이클 시간
  float target_delay;                                         //원하는 딜레이 시간
  
  cycle_time = cycle_end_time - cycle_start_time;
  target_delay = (target_time/number)- cycle_time;   

  
  delay(target_delay);                                             
}//Calculate_delay//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
