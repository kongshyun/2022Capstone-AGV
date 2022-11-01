#include <PIDController.h>       // PID 라이브러리  
#include <Wire.h>    
#include <TimerOne.h>            // 타이머 인터럽트 라이브러리

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
#define           MOTOR_1_PWM   5       // pin  5  OCR3A
#define           MOTOR_2_PWM   6       // pin  6  OCR4A
#define           MOTOR_3_PWM   7       // pin  7  OCR4B

#define     MOTOR_1_Direction   8       // pin  8 
#define     MOTOR_2_Direction   9       // pin  9
#define     MOTOR_3_Direction   10      // pin  10
 
#define               Brake_1   11      // pin  11
#define               Brake_2   12      // pin  12
#define               Brake_3   13      // pin  13

#define                Kp_pos   200      // 위치 p
#define                Ki_pos   0.01       // 위치 i
#define                Kd_pos   5000     // 위치 d
 
const float     g =    9.81;
      float av_dt =    0;

const float ratio =    360./139./52.;// 360도,기어비,ppr 34*2 채널



unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임
unsigned long sttime;




void setup() {

   Serial.begin(57600);
   Wire.begin(20);
   //Wire.onReceive(dataReceive);

   pinMode(encoderPinA_1, INPUT_PULLUP);     // 엔코더 핀 0~5
   attachInterrupt(0, doEncoderA1, CHANGE);
   pinMode(encoderPinB_1, INPUT_PULLUP);
   attachInterrupt(1, doEncoderB1, CHANGE);
   pinMode(encoderPinA_2, INPUT_PULLUP);
   attachInterrupt(4, doEncoderA2, CHANGE);
   pinMode(encoderPinB_2, INPUT_PULLUP);
   attachInterrupt(5, doEncoderB2, CHANGE);
   pinMode(encoderPinA_3, INPUT_PULLUP);
   attachInterrupt(2, doEncoderA3, CHANGE);
   pinMode(encoderPinB_3, INPUT_PULLUP);
   attachInterrupt(3, doEncoderB3, CHANGE);

   
   pinMode(        MOTOR_1_Direction, OUTPUT);  
   pinMode(        MOTOR_2_Direction, OUTPUT);
   pinMode(        MOTOR_3_Direction, OUTPUT);  
   pinMode(              MOTOR_1_PWM, OUTPUT);  
   pinMode(              MOTOR_2_PWM, OUTPUT);
   pinMode(              MOTOR_3_PWM, OUTPUT);
   pinMode(                  Brake_1, OUTPUT);
   pinMode(                  Brake_2, OUTPUT);
   pinMode(                  Brake_3, OUTPUT);
   
     
   digitalWrite(  MOTOR_1_Direction,    HIGH);  
   digitalWrite(  MOTOR_2_Direction,    HIGH);
   digitalWrite(  MOTOR_3_Direction,    HIGH);
   digitalWrite(            Brake_1,     LOW);
   digitalWrite(            Brake_2,     LOW);
   digitalWrite(            Brake_3,     LOW);



   TCCR3A = bit(COM3A1) | bit(WGM30);                // pin 5,6,7 - OCR3A, OCR4A, OCR4B
   TCCR3B = bit(WGM32)  | bit(CS31);                 // FAST PWM non-invert
   TCCR4A = bit(COM4A1) | bit(COM4B1) | bit(WGM40);  // prescaler 8 6.5khz
   TCCR4B = bit(WGM42)  | bit(CS41); 

   
   
   Motor1.begin();                           //pid 라이브러리 따라서 선언
   Motor1.tune(Kp_pos, Ki_pos, Kd_pos);      //pid 계수
   Motor1.limit(-255, 255);                  //pid 범위 제한
   Motor1.setpoint(0);                       //pid 목표값
   
   Motor2.begin();    
   Motor2.tune(Kp_pos, Ki_pos, Kd_pos);    
   Motor2.limit(-255, 255);
   Motor2.setpoint(0);

   Motor3.begin();    
   Motor3.tune(Kp_pos, Ki_pos, Kd_pos);    
   Motor3.limit(-255, 255);
   Motor3.setpoint(0);     
   
   Timer1.initialize(10000); // 0.003초마다 실행. in microseconds
   Timer1.attachInterrupt(degree_control);
  
    Serial.print("hello :) ");

   /*
    *  pinMode(5, OUTPUT);
    *  pinMode(6, OUTPUT);
    *  pinMode(7, OUTPUT);
    *  
    *  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41) | _BV(WGM40);
    *  TCCR4B = _BV(CS42);
    */

}

void loop(){

    ///////////////////////////                                                           
    loop_start_time   = micros();
    sttime            = millis();
   ///////////////////////////////조작하는 부분

   
   float theta= Potentiometer(-20, 35);
   

   //float theta =50;
   
   Plate_angle(theta,0,0);
   /*
   Motor1.setpoint(0);
   Motor2.setpoint(0);
   Motor3.setpoint(0);
   */
   //Motor1.setpoint(theta);
   

   Serial.print("Target : ");
   Serial.print(theta);
   Serial.print(",");
   Serial.print("degree1 : ");
   Serial.print(encoder_pos_1*ratio);
   Serial.print(",");
   Serial.print("degree2 : ");
   Serial.print(encoder_pos_2*ratio);
   Serial.print(",");
   Serial.print("degree3 : ");
   Serial.print(encoder_pos_3*ratio);
   Serial.println();
   
   
   /*
   if(Serial.available()){
    CMD=Serial.read();
     if(CMD=='1'){
        delay(2000);
        Serial.println("---------------11111----------------"); //천천히 1m를 간다
        Agv_accel(8,0.4);   //플레이트가 꺾이는 각도 , 이동거리를 넣는다.
        Agv_decel(8,0.4);   //2m를 넣으면 왔다 갔다 4m입니다.
     }
     if(CMD=='2'){
        Serial.println("---------------22222----------------"); //매우 빨리 2m를 간다
        Agv_accel(7,1); //    2초, 2m, 2m/s , 3m/s^2
        Agv_decel(7,1); 
     }
     if(CMD=='3'){
        Serial.println("---------------33333----------------"); // 매우매우빨리 1m를 간다
        Agv_accel(15,0.8);
        Agv_decel(15,0.8);
     }
      if(CMD=='4'){
        Serial.println("---------------44444----------------");   // 천천히 2m를 간다
        Agv_accel(5,1); 
        Agv_decel(5,1);
     }
      if(CMD=='5'){
        Serial.println("---------------5555----------------");    //적당히 1m 를 간다
        Agv_accel(7,0.70);
        Agv_decel(7,0.70);
     }     
   }  //   7, 0.8 등속 0.7

   */
  
  
   timecycle(); // 주기 계산 함수
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
     motor_control(1 ,(dir_1 > 0 )?Forward:Backward, min(abs(motor_value_1),255));  // direction 이 양이면 앞으로, 음이면 뒤로, motor _value 를 절댓값으로 만들어주고 255보다 크지 않게끔(생각해보니까 중복이네)
     motor_control(2 ,(dir_2 > 0 )?Forward:Backward, min(abs(motor_value_2),255));
     motor_control(3 ,(dir_3 > 0 )?Forward:Backward, min(abs(motor_value_3),255));
     
}

int motor_direction(double pid){                 // 모터 방향 결정하는 함수
  int Dir;
  if(pid > 0) Dir =  1;
  if(pid < 0) Dir = -1;
  return Dir;
}

/*
void Agv_accel( float highest_theta, float Length){

  float Vel;
  float Accel;
  float cmd_rpm;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  float theta;

  
 
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, (Length)); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec=0; sec < p_time;  sec += 0.02){
    cycle_start_time = millis();
        
    Vel     =  -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   =  -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    RPM     =  Vel*60/(3.14*0.063);  // 테스트베드 시 0.058 , AGV 0.063

    SetPoint_L = - RPM;             // 가변저항으로 목표값 설정
    SetPoint_R =   RPM; 
    speed_pid_L.setpoint(SetPoint_L); // PID 함수에 목표값 넣기 
    speed_pid_R.setpoint(SetPoint_R);
    //speed_control();

    Serial.print("Target : ");
    Serial.print(RPM);
    Serial.print(",");
    Serial.print("RPM_L : ");
    Serial.print(x_f_L);
    Serial.print(",");
    Serial.print("RPM_R : ");
    Serial.print(x_f_R);
    Serial.println();
    
    Calculate_delay( p_time*1000,  p_time*50 );
  }
  Serial.print("\n");
}

void Agv(float Time){

  av_dt = Time;
  for(float sec=0; sec < Time;  sec += 0.02){
    cycle_start_time = millis();
     
    SetPoint_L = - RPM;             // 가변저항으로 목표값 설정
    SetPoint_R =   RPM; 
    speed_pid_L.setpoint(SetPoint_L); // PID 함수에 목표값 넣기 
    speed_pid_R.setpoint(SetPoint_R);
    //speed_control();

    Serial.print("Target : ");
    Serial.print(RPM);
    Serial.print(",");
    Serial.print("RPM_L : ");
    Serial.print(x_f_L);
    Serial.print(",");
    Serial.print("RPM_R : ");
    Serial.print(x_f_R);
    Serial.println();
    
    Calculate_delay( p_time*1000,  p_time*50 );
  } 
}

void Agv_decel( float highest_theta, float Length){

  float Vel;
  float Accel;
  //float RPM;
  float cmd_rpm;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  float theta;
  
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, (Length)); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec= p_time ; sec > 0;  sec -= 0.02){
    cycle_start_time = millis();
        
    Vel     =  - pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   =  - pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    RPM     =  Vel*60/(3.14*0.063);  // 테스트베드 시 0.058 , AGV 0.063
    
    SetPoint_L = - RPM;             // 가변저항으로 목표값 설정
    SetPoint_R =   RPM; 
    speed_pid_L.setpoint(SetPoint_L); // PID 함수에 목표값 넣기 
    speed_pid_R.setpoint(SetPoint_R);
    //speed_control();

    Serial.print("Target : ");
    Serial.print(RPM);
    Serial.print(",");
    Serial.print("RPM_L : ");
    Serial.print(x_f_L);
    Serial.print(",");
    Serial.print("RPM_R : ");
    Serial.print(x_f_R);
    Serial.println();
    
    Calculate_delay( p_time*1000,  p_time*50 );
    
  }
  Serial.print("\n");
  Serial.print(" 걸린 시간 = ");
  Serial.print(p_time*2+av_dt);
  Serial.print("sec   ");
  Serial.print(",");
  Serial.print(" 이동거리 =");
  Serial.print(p_length*2+(p_velocity*av_dt));
  Serial.print(" m ");
  Serial.print(",");
  Serial.print(" 최고 속도 = ");
  Serial.print(p_velocity);
  Serial.print(" m/s ");
  Serial.print(",");
  Serial.print(" 최고 가속도 = ");
  Serial.print(p_accel);
  Serial.print(" m/s^2 ");
  Serial.print(",");
  Serial.print(" 알파 = ");
  Serial.print(p_alpha/3);
  Serial.print(",");
  Serial.print(" 베타 = ");
  Serial.print(p_beta/2);
  Serial.println();

  av_dt = 0 ;
}

*/

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


void motor_control(int motor_num, int motor_direction, int degree){  //왼쪽모터 오른쪽 모터 돌릴건지 , 모터 방향이랑, 모터 속도

  int stop_pwm = 1;
  int DIR      = 0;
  int BRAKE    = 0;
  degree = min(abs(degree),255);  //속도가 255넘으면 안되게끔 (0~255)

  if(motor_num == 1){
    DIR   = MOTOR_1_Direction;
    OCR4A = 
    BRAKE = Brake_1;
  }
  if(motor_num == 2){
    DIR   = MOTOR_2_Direction;
    OCR4B = degree;
    BRAKE = Brake_2;
  }
  if(motor_num == 3){
    DIR   = MOTOR_3_Direction;
    PWM   = MOTOR_3_PWM;
    BRAKE = Brake_3;
  }
  
  degree = max(degree,5);
  if((degree < stop_pwm)|| motor_direction == Stop ){// stop
    //Serial.println("Motor Stop");
    digitalWrite(            BRAKE,    HIGH);
  }
    if((degree > stop_pwm) && motor_direction == Forward){ 
    digitalWrite(           BRAKE,      LOW);  
    digitalWrite(             DIR,     HIGH);
  }
    if((degree > stop_pwm) && motor_direction == Backward){    
    digitalWrite(            BRAKE,      LOW);
    digitalWrite(              DIR,      LOW);
  }
}
  
float Potentiometer(int number_start , int number_end){     // 가변저항 함수
  
  number_start = 100*number_start;
  number_end   = 100*number_end;
  
  float val_1 = analogRead(A0);
  val_1 = map(val_1, 0, 1023 , number_start, number_end);

  return val_1/100;
}

/*
void profile_maker(float highest_theta, float Length){
  //플레이트 각과 이동거리를 입력하면 최대 가속도, 최대 속도, 걸리는 시간이 나온다
  
  float profile_Time;       //a(t)= 0 이 되는 시각 t
  float t;                  //임시 지역 변수
  float V_length;           //적분된 velocity     = L(t) - L(0) = V(t) - V(0) 
  float A_velocity;         //적분된 acceleration = v(t) - v(0) = A(t) - A(0)
  float highest_accel;      //최대 가속도
  float alpha, beta;        //계수 알파 베타
  float g = 9.81;           //중력 가속도
  float pi = 3.1415;
   
  highest_accel = g*tan((highest_theta*pi)/180);
  
  // (3m/s,   2초) alpha = 0.75 , beta =  2.25
  // (2m/s,   1초) alpha = 4   ,  beta =     6
  // (1m/s, 0.5초) alpha = 16  ,  beta =    12 

  //  V         = -pow(t,4)* (alpha/12)    + pow(t,3)* (beta/6);    // 거리 
  //  A         = -pow(t,3)* (alpha/3)     + pow(t,2)* (beta/2);    // 속도 
  //  a         = -pow(t,2)*  alpha        + pow(t,1)*  beta;       // 가속도
  
  profile_Time  =  sqrt((3*Length)/(highest_accel));
  t             =  profile_Time;
  alpha         =  (4*highest_accel/pow(t,2));
  beta          =  alpha*t;
  V_length      = -pow(t,4)* (alpha/12)  + pow(t,3)* (beta/6);      // 거리 
  A_velocity    = -pow(t,3)* (alpha/ 3)  + pow(t,2)* (beta/2);      // 속도 
  
  p_accel       =  highest_accel;
  p_time        =  profile_Time;
  p_velocity    =  A_velocity;
  p_length      =  V_length;
  p_alpha       =  alpha;
  p_beta        =  beta;
 
  
}






void Calculate_delay( float target_time, float number ){//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
                                                            //target_time (ms)
                                                            //number (횟수)  
  float cycle_end_time = millis();                          //사이클 끝나는 시각
  float cycle_time;                                         //측정된 1 사이클 시간
  float target_delay;                                       //원하는 딜레이 시간
  
  cycle_time = cycle_end_time - cycle_start_time;
  target_delay = ((target_time)/number)- cycle_time;   

  delay(target_delay);
                                                                        
}//Calculate_delay//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

*/


void timecycle(){//ㅡㅡㅡㅡㅡㅡㅡㅡ
  float t;                            
  t = micros() - loop_start_time;
  dt = t;
  
  //Serial.print("loop time : ");
  //Serial.print(dt);
  //Serial.println();
                                       
}//timecycle

void Plate_angle(float Roll, float Pitch, float height){
  
  Calculate_Roll_Pitch(Roll, Pitch, height);
  
  Motor1.setpoint(theta_x);
  Motor2.setpoint(theta_y);
  Motor3.setpoint(theta_z);

}//Plate_angle//




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
