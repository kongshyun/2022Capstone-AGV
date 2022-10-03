//#define USE_USBCON true
#include <PIDController.h>       // PID 라이브러리  
#include <Wire.h>
#include <TimerOne.h>            // 타이머 인터럽트 라이브러리
#include <PWM.h>                 // 아두이노 출력 998HZ PWM을 40KHZ 까지 올려주는 라이브러리 

/////////////////////////// 받은 속도값으로 모터 제어+엔코더 값을 읽어 퍼블리시하는 node
#include <ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include<std_msgs/Float64.h>

#define LOOPTIME  100                         //Looptime in millisecond
//#define SBUF_SIZE 64 // imu
#define ROS
//char sbuf[SBUF_SIZE]; // imu
//signed int sbuf_cnt=0; // imu

ros::NodeHandle  nh;
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter


const double radius = 0.064;  //바퀴 반지름[m] 우리껀0.064
const double width = 0.38;  //두 바퀴 사이 거리[m]
double linear_speed_cmd = 0; //AGV 선형 속도[m/s]
double angular_speed_cmd = 0; //AGV 각속도[rad/s]
double speed_cmd_left ;  //왼쪽 바퀴 속도[rpm]
double speed_cmd_right ; //오른쪽 바퀴 속도[rpm]

//////////////////////////////////////////////////////////

PIDController pos_pid_L;         // PID 함수 이름 선언 (포지션 LEFT)
PIDController speed_pid_L;       // 속도 PID LEFT
PIDController pos_pid_R;
PIDController speed_pid_R;

volatile long int encoder_pos_L = 0;   // LEFT 엔코더 신호 저장 변수
volatile long int encoder_pos_R = 0;   // 오른쪽
int                motor_mode = 2;     // 0 - none, 1 - position control, 2 speed control //지금은 속도 모드만 가능
long int          last_millis = 0;

long int   last_encoder_pos_L = 0;
int             motor_value_L = 0;     // PID 함수에서 나오는 변수. 최종적으로 모터조작함수에 들어감
double             SetPoint_L = 0;     // 목표값

long int   last_encoder_pos_R = 0;
int             motor_value_R = 0;
double             SetPoint_R = 0;

int                     dir_L = 0;     // 방향 결정
int                     dir_R = 0;
float                   RPM_L = 0;     // encoder_pos로 RPM 계산해서 넣는 변수
float                   RPM_R = 0;

int32_t              pwm_freq = 20000;  // 모터 제어 주파수

#define               Forward   1
#define              Backward   2
#define                  Stop   0
#define                  LEFT   1
#define                 RIGHT   2


#define         encoderPinA_R   21      // pin  21 
#define         encoderPinB_R   20      // pin  20
#define         encoderPinA_L   2       // pin  2 
#define         encoderPinB_L   3       // pin  3
#define           MOTOR_L_PWM   5       // pin  5 
#define           MOTOR_R_PWM   6       // pin  6 
#define     MOTOR_L_Direction   7       // pin  7 
#define     MOTOR_R_Direction   8       // pin  8 
#define               Brake_L   9       // pin  9
#define               Brake_R   10      // pin  10

#define                Kp_pos   20      // 위치 p
#define                Ki_pos   0.02    // 위치 i
#define                Kd_pos   2       // 위치 d 

#define                Kp_rpm   10  // 속도 p 20일때 허용범위 안 정상오차20,0.2,0
#define                Ki_rpm   0.4     // 속도 i 5 0.4 1
#define                Kd_rpm   4       // 속도 d 

const float     g =    9.81;
float av_dt =    0;
const float ratio =    360. / 5. / 68.; // 360도,기어비,ppr 34*2 채널

//---
static long analogPinTimer_L = 0;
// Set the sampling time
#define ANALOG_PIN_TIMER_INTERVAL 2 // milliseconds
unsigned long thisMillis_old_L;

int    fc_L      = 2; // cutoff frequency 5~10 Hz 정도 사용해보시기 바랍니다
double dt_L      = ANALOG_PIN_TIMER_INTERVAL / 1000.0; // sampling time
double lambda_L  = 2 * PI * fc_L * dt_L;
double x_L       = 0.0;
double x_f_L     = 0.0;
double x_fold_L  = 0.0;

static long analogPinTimer_R = 0;
// Set the sampling time
#define ANALOG_PIN_TIMER_INTERVAL 2 // milliseconds
unsigned long thisMillis_old_R;
int    fc_R       = 2; // cutoff frequency 5~10 Hz 정도 사용해보시기 바랍니다
double dt_R       = ANALOG_PIN_TIMER_INTERVAL / 1000.0; // sampling time
double lambda_R   = 2 * PI * fc_R * dt_R;
double x_R        = 0.0;
double x_f_R      = 0.0;

float  p_time;                          //---------------------------
float  p_velocity;                      //
float  p_length;                        // 프로파일 계산 함수 전역변수 선언
float  p_alpha;                         //
float  p_beta;                          //---------------------------
float  p_accel;

char  CMD;

double x_fold_R   = 0.0;                // 로우패스필터 변수 왼쪽 오른쪽

unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임
unsigned long sttime;
float RPM;

void MotorL(double Left_v);
void MotorR(double Right_v);

float publlish_encoder();

//속도를 어디에서받아오는지 구독할 노드를 정의한다.
std_msgs::Float64 str_msg;
ros::Publisher enc_pub("encoder_data", &str_msg);
ros::Publisher lp_time("lp_time", &str_msg);
// cmd_vel 콜백 함수
void AGVcontrol_cmd (const std_msgs :: Float64& cmd_vel) {
  //모터속도(왼,오)rpm
  speed_cmd_left = cmd_vel.data;
  MotorL(speed_cmd_left);
  publish_encoder();
  
}

// subscriber
ros::Subscriber<std_msgs::Float64> cmd_vel("cmd_vel", AGVcontrol_cmd);



void Motors_init();
void MotorL(double Left_v);
void MotorR(double Right_v);



void setup() {
  Motors_init();

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_vel);
  nh.advertise(enc_pub);
  nh.advertise(enc_pub);
  nh.advertise(lp_time);

}

void loop() {
  unsigned long stttime=millis();
  nh.spinOnce();//rosserial을 통해 계속해서 키보드 값을 받는다.
  speed_control();
  unsigned long L_T=millis()-stttime;
  str_msg.data=L_T;
  lp_time.publish(&str_msg);
  
}

void Motors_init() {
  Serial.begin(57600);
  MotorL(0);
  MotorR(0);
  
  Wire.begin(20);
  pinMode(encoderPinA_L, INPUT_PULLUP);     // 엔코더 핀 0~3
  attachInterrupt(0, doEncoderAL, CHANGE);
  pinMode(encoderPinB_L, INPUT_PULLUP);
  attachInterrupt(1, doEncoderBL, CHANGE);
  pinMode(encoderPinA_R, INPUT_PULLUP);
  attachInterrupt(2, doEncoderAR, CHANGE);
  pinMode(encoderPinB_R, INPUT_PULLUP);
  attachInterrupt(3, doEncoderBR, CHANGE);


  pinMode(        MOTOR_L_Direction, OUTPUT);
  pinMode(        MOTOR_R_Direction, OUTPUT);
  pinMode(              MOTOR_L_PWM, OUTPUT);
  pinMode(              MOTOR_R_PWM, OUTPUT);
  pinMode(                  Brake_L, OUTPUT);
  pinMode(                  Brake_R, OUTPUT);


  digitalWrite(  MOTOR_L_Direction,    HIGH);
  digitalWrite(  MOTOR_R_Direction,    HIGH);
  digitalWrite(            Brake_L,     LOW);
  digitalWrite(            Brake_R,     LOW);

  pos_pid_L.begin();                           //pid 라이브러리 따라서 선언
  pos_pid_L.tune(Kp_pos, Ki_pos, Kd_pos);      //pid 계수
  pos_pid_L.limit(-255, 255);                  //pid 범위 제한
  pos_pid_L.setpoint(0);                       //pid 목표값

  pos_pid_R.begin();
  pos_pid_R.tune(Kp_pos, Ki_pos, Kd_pos);
  pos_pid_R.limit(-255, 255);
  pos_pid_R.setpoint(0);

  speed_pid_L.begin();
  speed_pid_L.tune(Kp_rpm, Ki_rpm, Kd_rpm);
  speed_pid_L.limit(-255, 255);

  speed_pid_R.begin();
  speed_pid_R.tune(Kp_rpm, Ki_rpm, Kd_rpm);
  speed_pid_R.limit(-255, 255);

  bool success_1 = SetPinFrequencySafe(MOTOR_L_PWM, pwm_freq);   // PWM 핀에서 20KHZ 나오게
  bool success_2 = SetPinFrequencySafe(MOTOR_R_PWM, pwm_freq);

}


void MotorL(double Left_v) {
  SetPoint_L =  Left_v;
  speed_pid_L.setpoint(SetPoint_L);


}
void MotorR(double Right_v) {
  SetPoint_R =  -Right_v;
  speed_pid_R.setpoint(SetPoint_R);

}
void degree_control() {                                         // 각도 제어
  float motorDeg_L = float(encoder_pos_L) * ratio;           // 현재 각도 계산, 각도는 엔코더 신호 * 기어비 * PPR
  float motorDeg_R = float(encoder_pos_R) * ratio;
  motor_value_L = pos_pid_L.compute(motorDeg_L);             //현재 각도를 PID 함수에 넣고 motor_value로 계산된 pid를 꺼내온다.
  motor_value_R = pos_pid_R.compute(motorDeg_R);
  dir_L = motor_direction(motor_value_L);                    //motor_value의 부호를 보고 방향을 판단한다.
  dir_R = motor_direction(motor_value_R);
  motor_control(LEFT , (dir_L > 0 ) ? Forward : Backward, min(abs(motor_value_L), 255)); // direction 이 양이면 앞으로, 음이면 뒤로, motor _value 를 절댓값으로 만들어주고 255보다 크지 않게끔(생각해보니까 중복이네)
  motor_control(RIGHT, (dir_R > 0 ) ? Forward : Backward, min(abs(motor_value_R), 255));
}

void speed_control() {
  RPM_L  = ((float(encoder_pos_L) - float(last_encoder_pos_L)) / 340) * 60.0 * (1000 / (millis() - last_millis)); // 현재 RPM을 계산한다. 지나간 시간과 그동안 감지한 엔코더 신호를 파악해서 계산함 (거리/시간)
  RPM_R  = ((float(encoder_pos_R) - float(last_encoder_pos_R)) / 340) * 60.0 * (1000 / (millis() - last_millis));
  RPM_L  = low_pass_filter_L(RPM_L);  // 엔코더 신호를 시간으로 나눈 값이라 가끔 튀는 값이 있어서 로우패스 필터로 한번 걸러준다.
  RPM_R  = low_pass_filter_R(RPM_R);
  motor_value_L = speed_pid_L.compute(RPM_L);  // 속도 PID 함수에 넣어주고 motor_value를 받는다.
  motor_value_R = speed_pid_R.compute(RPM_R);
  dir_L = motor_direction(motor_value_L);      //  모터밸류 변수의 부호로 방향을 결정함
  dir_R = motor_direction(motor_value_R);
  motor_control( LEFT, (dir_L > 0 ) ? Forward : Backward, min(abs(motor_value_L), 255)); // 모터 컨트롤 함수에 넣는다. dir 변수의 부호 따라서 앞,뒤.
  motor_control(RIGHT, (dir_R > 0 ) ? Forward : Backward, min(abs(motor_value_R), 255));
  last_encoder_pos_L = encoder_pos_L;          // 루프마다 초기화해서 계속 루프시간당 엔코더 구할 수 있게
  last_encoder_pos_R = encoder_pos_R;
  last_millis = millis();
  delayMicroseconds(300);                      //PID 에서 적분시간 Ti를 결정하는 delay
}

int motor_direction(double pid) {                // 모터 방향 결정하는 함수
  int Dir;
  if (pid > 0) Dir =  1;
  if (pid < 0) Dir = -1;
  return Dir;
}

void Agv_accel( float highest_theta, float Length) {

  float Vel;
  float Accel;
  float cmd_rpm;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  float theta;



  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, (Length)); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////

  for (float sec = 0; sec < p_time;  sec += 0.02) {
    cycle_start_time = millis();

    Vel     =  -pow(sec, 3) * (p_alpha / 3) + pow(sec, 2) * (p_beta / 2); // 속도
    Accel   =  -pow(sec, 2) *  p_alpha    + pow(sec, 1) *  p_beta; // 가속도
    theta   =  atan(Accel / g) * 180 / 3.14;
    RPM     =  Vel * 60 / (3.14 * 0.063); // 테스트베드 시 0.058 , AGV 0.063

    SetPoint_L = - RPM;             // 가변저항으로 목표값 설정
    SetPoint_R =   RPM;
    speed_pid_L.setpoint(SetPoint_L); // PID 함수에 목표값 넣기
    speed_pid_R.setpoint(SetPoint_R);
    //speed_control();
    /*
        Serial.print("Target : ");
        Serial.print(RPM);
        Serial.print(",");
        Serial.print("RPM_L : ");
        Serial.print(x_f_L);
        Serial.print(",");
        Serial.print("RPM_R : ");
        Serial.print(x_f_R);
        Serial.println();
    */
    Calculate_delay( p_time * 1000,  p_time * 50 );
  }
  //Serial.print("\n");
}

void Agv(float Time) {

  for (float sec = 0; sec < p_time;  sec += 0.02) {
    cycle_start_time = millis();

    SetPoint_L = - RPM;             // 가변저항으로 목표값 설정
    SetPoint_R =   RPM;
    speed_pid_L.setpoint(SetPoint_L); // PID 함수에 목표값 넣기
    speed_pid_R.setpoint(SetPoint_R);
    //speed_control();
    /*
        Serial.print("Target : ");
        Serial.print(RPM);
        Serial.print(",");
        Serial.print("RPM_L : ");
        Serial.print(x_f_L);
        Serial.print(",");
        Serial.print("RPM_R : ");
        Serial.print(x_f_R);
        Serial.println();
    */
    Calculate_delay( p_time * 1000,  p_time * 50 );
  }
}

void Agv_decel( float highest_theta, float Length) {

  float Vel;
  float Accel;
  //float RPM;
  float cmd_rpm;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  float theta;

  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, (Length)); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////

  for (float sec = p_time ; sec > 0;  sec -= 0.02) {
    cycle_start_time = millis();

    Vel     =  - pow(sec, 3) * (p_alpha / 3) + pow(sec, 2) * (p_beta / 2); // 속도
    Accel   =  - pow(sec, 2) *  p_alpha    + pow(sec, 1) *  p_beta; // 가속도
    theta   =  atan(Accel / g) * 180 / 3.14;
    RPM     =  Vel * 60 / (3.14 * 0.063); // 테스트베드 시 0.058 , AGV 0.063

    SetPoint_L = - RPM;             // 가변저항으로 목표값 설정
    SetPoint_R =   RPM;
    speed_pid_L.setpoint(SetPoint_L); // PID 함수에 목표값 넣기
    speed_pid_R.setpoint(SetPoint_R);
    //speed_control();
    /*
        Serial.print("Target : ");
        Serial.print(RPM);
        Serial.print(",");
        Serial.print("RPM_L : ");
        Serial.print(x_f_L);
        Serial.print(",");
        Serial.print("RPM_R : ");
        Serial.print(x_f_R);
        Serial.println();
    */
    Calculate_delay( p_time * 1000,  p_time * 50 );

  }
  /*
    Serial.print("\n");
    Serial.print(" 걸린 시간 = ");
    Serial.print(p_time*2);
    Serial.print("sec   ");
    Serial.print(",");
    Serial.print(" 이동거리 =");
    Serial.print(p_length*2);
    Serial.print(" m ");
    Serial.print(",");
    Serial.print(" 최고 속도 = ");
    Serial.print(p_velocity);
    Serial.print(" m/s ");
    Serial.print(",");
    Serial.print(" 최고 가속도 = ");
    Serial.print(p_accel);
    Serial.print(" m/s^2 ");
    Serial.println();*/
}

void doEncoderAL() {
  int dir_pos  = (digitalRead(encoderPinA_L) == digitalRead(encoderPinB_L)) ? 1 : -1; // 엔코더 신호 감지, A,B 함수로 모든 EDGE를 잡아 CW ,CCW 판단
  encoder_pos_L += dir_pos;
}
void doEncoderBL() {
  int dir_pos    = (digitalRead(encoderPinA_L) == digitalRead(encoderPinB_L)) ? -1 : 1; //
  encoder_pos_L += dir_pos;
}

void doEncoderAR() {
  int dir_pos     = (digitalRead(encoderPinA_R) == digitalRead(encoderPinB_R)) ? 1 : -1;
  encoder_pos_R  += dir_pos;
}
void doEncoderBR() {
  int dir_pos    = (digitalRead(encoderPinA_R) == digitalRead(encoderPinB_R)) ? -1 : 1;
  encoder_pos_R += dir_pos;
}

void motor_control(int motor_num, int motor_direction, int velocity) { //왼쪽모터 오른쪽 모터 돌릴건지 , 모터 방향이랑, 모터 속도

  int stop_pwm = 1;
  int DIR      = 0;
  int PWM      = 0;
  int BRAKE    = 0;
  velocity = min(abs(velocity), 255); //속도가 255넘으면 안되게끔 (0~255)

  if (motor_num == LEFT) {
    DIR   = MOTOR_L_Direction;
    PWM   = MOTOR_L_PWM;
    BRAKE = Brake_L;
  }
  if (motor_num == RIGHT) {
    DIR   = MOTOR_R_Direction;
    PWM   = MOTOR_R_PWM;
    BRAKE = Brake_R;
  }

  velocity = max(velocity, 5);
  if ((velocity < stop_pwm) || motor_direction == Stop ) { // stop
    //Serial.println("Motor Stop");
    digitalWrite(            Brake_L,    HIGH);
    digitalWrite(            Brake_R,    HIGH);
  }

  if ((velocity > stop_pwm) && motor_direction == Forward) {
    digitalWrite(           BRAKE,      LOW);
    digitalWrite(             DIR,     HIGH);
    digitalWrite(             PWM,     HIGH);
    analogWrite (             PWM, velocity);
  }

  if ((velocity > stop_pwm) && motor_direction == Backward) {
    digitalWrite(            BRAKE,      LOW);
    digitalWrite(              DIR,      LOW);
    digitalWrite(              PWM,     HIGH);
    analogWrite (              PWM, velocity);
  }
}

int Potentiometer(int number_start , int number_end) {    // 가변저항 함수

  number_start = number_start;
  number_end   = number_end;

  int val_1 = analogRead(A0);
  val_1 = map(val_1, 0, 1023 , number_start, number_end);

  return val_1;
}


void profile_maker(float highest_theta, float Length) {
  //플레이트 각과 이동거리를 입력하면 최대 가속도, 최대 속도, 걸리는 시간이 나온다

  float profile_Time;       //a(t)= 0 이 되는 시각 t
  float t;                  //임시 지역 변수
  float V_length;           //적분된 velocity     = L(t) - L(0) = V(t) - V(0)
  float A_velocity;         //적분된 acceleration = v(t) - v(0) = A(t) - A(0)
  float highest_accel;      //최대 가속도
  float alpha, beta;        //계수 알파 베타
  float g = 9.81;           //중력 가속도
  float pi = 3.1415;

  highest_accel = g * tan((highest_theta * pi) / 180);

  // (3m/s,   2초) alpha = 0.75 , beta =  2.25
  // (2m/s,   1초) alpha = 4   ,  beta =     6
  // (1m/s, 0.5초) alpha = 16  ,  beta =    12

  //  V         = -pow(t,4)* (alpha/12)    + pow(t,3)* (beta/6);    // 거리
  //  A         = -pow(t,3)* (alpha/3)     + pow(t,2)* (beta/2);    // 속도
  //  a         = -pow(t,2)*  alpha        + pow(t,1)*  beta;       // 가속도

  profile_Time  =  sqrt((3 * Length) / (highest_accel));
  t             =  profile_Time;
  alpha         =  (4 * highest_accel / pow(t, 2));
  beta          =  alpha * t;
  V_length      = -pow(t, 4) * (alpha / 12)  + pow(t, 3) * (beta / 6); // 거리
  A_velocity    = -pow(t, 3) * (alpha / 3)  + pow(t, 2) * (beta / 2); // 속도

  p_accel       =  highest_accel;
  p_time        =  profile_Time;
  p_velocity    =  A_velocity;
  p_length      =  V_length;
  p_alpha       =  alpha;
  p_beta        =  beta;


}




void Calculate_delay( float target_time, float number ) { //ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
  //target_time (ms)
  //number (횟수)
  float cycle_end_time = millis();                          //사이클 끝나는 시각
  float cycle_time;                                         //측정된 1 사이클 시간
  float target_delay;                                       //원하는 딜레이 시간

  cycle_time = cycle_end_time - cycle_start_time;
  target_delay = ((target_time) / number) - cycle_time;

  delay(target_delay);

}//Calculate_delay//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


float low_pass_filter_L(float raw_data) {                 // 로우 패스 필터

  unsigned long deltaMillis = 0; // clear last result
  unsigned long thisMillis = millis();
  if (thisMillis != thisMillis_old_L) {
    deltaMillis = thisMillis - thisMillis_old_L;
    thisMillis_old_L = thisMillis;
  }

  analogPinTimer_L -= deltaMillis;

  if (analogPinTimer_L <= 0) {
    analogPinTimer_L += ANALOG_PIN_TIMER_INTERVAL;

    // sensing loop start!!
    x_L = raw_data; // 아날로그값 읽기
    x_f_L = lambda_L / (1 + lambda_L) * x_L + 1 / (1 + lambda_L) * x_fold_L; //필터된 값
    x_fold_L = x_f_L; // 센서 필터 이전값 업데이트

    return x_f_L;
  }
}

float low_pass_filter_R(float raw_data) {

  unsigned long deltaMillis = 0; // clear last result
  unsigned long thisMillis = millis();
  if (thisMillis != thisMillis_old_R) {
    deltaMillis = thisMillis - thisMillis_old_R;
    thisMillis_old_R = thisMillis;
  }

  analogPinTimer_R -= deltaMillis;

  if (analogPinTimer_R <= 0) {
    analogPinTimer_R += ANALOG_PIN_TIMER_INTERVAL;

    // sensing loop start!!
    x_R = raw_data; // 아날로그값 읽기
    x_f_R = lambda_R / (1 + lambda_R) * x_R + 1 / (1 + lambda_R) * x_fold_R; //필터된 값
    x_fold_R = x_f_R; // 센서 필터 이전값 업데이트

    return x_f_R;
  }
}

float publish_encoder(){
  str_msg.data=x_f_L;
  enc_pub.publish(&str_msg);
}
