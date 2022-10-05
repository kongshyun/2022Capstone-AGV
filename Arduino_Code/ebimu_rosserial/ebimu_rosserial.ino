

/*
 *    9.27  아두이노 나노에서 제어
 * 
 */
#include <ros.h>
#include<std_msgs/Float64.h>

const float pi = 3.141592;              // 파이
const float g = 9.81;                   // 중력가속도

#include<Servo.h> //Servo 라이브러리를 추가
Servo servo1;      //Servo 클래스로 servo객체 생성
Servo servo2;
Servo servo3;


#define servoPin_1  5               // 모터 1
#define servoPin_2  6               // 모터 2
#define servoPin_3  7               // 모터 3
 
#define SBUF_SIZE 128

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

float theta_x , theta_y, theta_z;   // 역기구학 통과한 모터1,2,3 각도

float ang_x;
float ang_y;
float ang_z;
float accel_x;
float accel_y;
float accel_z;
float accel_x_no_g;
float accel_y_no_g;
float accel_z_no_g;
float old_accel_x_no_g;
float old_accel_y_no_g;
float old_accel_z_no_g;

float angle_x;                          // pitch
float angle_y;                          // roll
float angle_z;                          // yaw

float the_x, the_y, the_z;                     // 보정각도 프로파일 세타

unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long loop_dt;                       // 1 루프 타임
unsigned long sttime;

float Gimbal_Roll;
float Gimbal_Pitch;

const int numReadings_x = 10;
const int numReadings_y = 10;
const int numReadings_z = 10;

int  readings_x[numReadings_x];        // the readings from the analog input
int readIndex_x = 0;                   // the index of the current reading
int     total_x = 0;                   // the running total
int   average_x = 0; 
// the average
int  readings_y[numReadings_y];        // the readings from the analog input
int readIndex_y = 0;                   // the index of the current reading
int     total_y = 0;                   // the running total
int   average_y = 0;                   // the average

int  readings_z[numReadings_z];        // the readings from the analog input
int readIndex_z = 0;                   // the index of the current reading
int     total_z = 0;                   // the running total
int   average_z = 0;                   // the average


static long         analogPinTimer = 0; 
// Set the sampling time
#define  ANALOG_PIN_TIMER_INTERVAL   2 // milliseconds
unsigned long thisMillis_old_x;
unsigned long thisMillis_old_y;
unsigned long thisMillis_old_z;

int fc = 20; // cutoff frequency 5~10 Hz 정도 사용해보시기 바랍니다
double dt_x = ANALOG_PIN_TIMER_INTERVAL/1000.0; // sampling time
double lambda_x = 2*PI*fc*dt_x;
double x_x = 0.0;
double x_f_x = 0.0;
double x_fold_x = 0.0;

double dt_y = ANALOG_PIN_TIMER_INTERVAL/1000.0; // sampling time
double lambda_y = 2*PI*fc*dt_y;
double x_y = 0.0;
double x_f_y = 0.0;
double x_fold_y = 0.0;

double dt_z = ANALOG_PIN_TIMER_INTERVAL/1000.0; // sampling time
double lambda_z = 2*PI*fc*dt_z;
double x_z = 0.0;
double x_f_z = 0.0;
double x_fold_z = 0.0;
double degree;

ros::NodeHandle  nh;

// cmd_vel 콜백 함수
void Stabilizer_cmd (const std_msgs :: Float64& cmd_servo) {
  //모터속도(왼,오)rpm
  degree = cmd_servo.data;
  IMU_Sensing(); 
  servo1.write(degree);
}

// subscriber
ros::Subscriber<std_msgs::Float64> cmd_servo("cmd_servo", Stabilizer_cmd);


void setup() {
  servo_init();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_servo);
}

void loop(){
  nh.spinOnce();//rosserial을 통해 계속해서 키보드 값을 받는다.
  delay(1);
}

void servo_init(){
  
      Serial.begin(57600);
      
      servo1.attach(servoPin_1);  
      servo2.attach(servoPin_2);
      servo3.attach(servoPin_3);

      for (int thisReading_x = 0; thisReading_x < numReadings_x; thisReading_x++) {
           readings_x[thisReading_x] = 0;
      }
      for (int thisReading_y = 0; thisReading_y < numReadings_y; thisReading_y++) {
           readings_y[thisReading_y] = 0;
      }
      for (int thisReading_z = 0; thisReading_z < numReadings_z; thisReading_z++) {
           readings_z[thisReading_z] = 0;
      }
}
void IMU_Sensing(){

   float euler[9];
    if(EBimuAsciiParser(euler, 9))
    {
      angle_x      = euler[0];
      angle_y      = euler[1];
      angle_z      = euler[2];//메가로 줄 YAW값
      accel_x_no_g = euler[3];
      accel_y_no_g = euler[4];      
      accel_z_no_g = euler[5];
      //acc_y        = euler[6];
      //acc_z        = euler[7];
      //gyr_x        = euler[8];
    }
      Gimbal_Pitch      =  angle_x + (  -5  ); 
      Gimbal_Roll       =  angle_y + (  -5  );
    
      float alpha = 0.8;
      accel_x = ( (1-alpha) * accel_x_no_g ) + ( alpha * old_accel_x_no_g);
      accel_y = ( (1-alpha) * accel_y_no_g ) + ( alpha * old_accel_y_no_g);
      accel_z = ( (1-alpha) * accel_z_no_g ) + ( alpha * old_accel_z_no_g);
        
      
      the_x   = atan(accel_x)*180/3.14;
      the_y   = atan(accel_y)*180/3.14;
      the_x = moving_x(the_x);
      the_y = moving_y(the_y);      
      the_x = LPF_x(the_x);
      the_y = LPF_y(the_y);
      the_z = LPF_z(accel_z);

      old_accel_x_no_g = accel_x;
      old_accel_y_no_g = accel_y;
      
      old_accel_z_no_g = accel_z;
      Serial.println(angle_x);
  }



void timecycle(){//ㅡㅡㅡㅡㅡㅡㅡㅡ
  float t;                            
  t = millis() - loop_start_time;
  loop_dt = t;
  
                                       
}//timecycle


int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;

         // Serial.print("\n\r");
         // for(i=0;i<number_of_item;i++)  {  Serial.print(item[i]);  Serial.print(" "); }
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}


void Plate_angle(float Roll, float Pitch, float height){
  
  Calculate_Roll_Pitch(Roll, Pitch, height);
  
  servo1.write(theta_x);
  servo2.write(theta_y);
  servo3.write(theta_z);

}//Plate_angle//




void Calculate_Roll_Pitch(float Target_roll, float Target_pitch, float Target_height) {//ㅡㅡㅡㅡㅡㅡ
  // Target_roll = 원하는 롤 값,  Target_pitch = 원하는 피치값                        
  // link_A = 링크 1의 길이 , link_B = 링크2의 길이                                     
                                                                                     
  float link_A1 = 40; //
  float link_A2 = 40; //
  float link_A3 = 40; //
  float link_B1 = 60; //
  float link_B2 = 60; //
  float link_B3 = 60; //
  float height_data = 67 + Target_height; // 

  ///////////////////////////////////////////////
  
  float yaw = 0;     //실험용 yaw

  ///////////////////////////////////////////////
  
  float Length_high, Length_low;

  Length_high = 110; //
  Length_low  = 35;  //
  
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


  float x = Target_roll  * (pi / 180); // 각도를 라디안으로 변환함
  float y = Target_pitch * (pi / 180);
      yaw = yaw          * (pi / 180);

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

    int initial_angle_error_x = -1.5; 
    int initial_angle_error_y = -10.5;
    int initial_angle_error_z = 0;

    theta_x += initial_angle_error_x;//초기 모터 위치 값 에러 보정
    theta_y += initial_angle_error_y;
    theta_z += initial_angle_error_z;

    theta_x += 90; //모터가 세워져있기 때문에 
    theta_y += 90;
    theta_z += 90;   

    theta_x = constrain(theta_x, 30, 160); //위치 제한
    theta_y = constrain(theta_y, 30, 160);
    theta_z = constrain(theta_z, 30, 160);
                                                                
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

  float theta1 = thb * 180 / pi;
  float theta2 = thbb * 180 / pi;

  // 어떤 각도를 선택할 건지 알고리즘 추가해야함.

  return theta1;                                                                                                   
                                                                                                               
}////////////////////////////////////////////////////////////////////


float LPF_x(float input) {
unsigned long deltaMillis = 0; // clear last result
unsigned long thisMillis = millis();  
if (thisMillis != thisMillis_old_x) { 
  deltaMillis = thisMillis-thisMillis_old_x; 
  thisMillis_old_x = thisMillis;   
} 

analogPinTimer -= deltaMillis; 

if (analogPinTimer <= 0) {  
  analogPinTimer += ANALOG_PIN_TIMER_INTERVAL; 

  // sensing loop start!! 
  x_x = input; // 아날로그값 읽기
  x_f_x = lambda_x/(1+lambda_x)*x_x+1/(1+lambda_x)*x_fold_x; //필터된 값
  x_fold_x = x_f_x; // 센서 필터 이전값 업데이트

  return x_f_x;
  
 }

}

float LPF_y(float input) {
unsigned long deltaMillis = 0; // clear last result
unsigned long thisMillis = millis();  
if (thisMillis != thisMillis_old_y) { 
  deltaMillis = thisMillis-thisMillis_old_y; 
  thisMillis_old_y = thisMillis;   
} 

analogPinTimer -= deltaMillis; 

if (analogPinTimer <= 0) {  
  analogPinTimer += ANALOG_PIN_TIMER_INTERVAL; 

  // sensing loop start!! 
  x_y = input; // 아날로그값 읽기
  x_f_y = lambda_y/(1+lambda_y)*x_y+1/(1+lambda_y)*x_fold_y; //필터된 값
  x_fold_y = x_f_y; // 센서 필터 이전값 업데이트

  return x_f_y;
  
 }
}

float LPF_z(float input) {
unsigned long deltaMillis = 0; // clear last result
unsigned long thisMillis = millis();  
if (thisMillis != thisMillis_old_z) { 
  deltaMillis = thisMillis-thisMillis_old_z; 
  thisMillis_old_z = thisMillis;   
} 

analogPinTimer -= deltaMillis; 

if (analogPinTimer <= 0) {  
  analogPinTimer += ANALOG_PIN_TIMER_INTERVAL; 

  // sensing loop start!! 
  x_z = input; // 아날로그값 읽기
  x_f_z = lambda_z/(1+lambda_z)*x_z+1/(1+lambda_z)*x_fold_z; //필터된 값
  x_fold_z = x_f_z; // 센서 필터 이전값 업데이트

  return x_f_z;
  
 }

}

float moving_x(float input) {  
  total_x     -= readings_x[readIndex_x];
  readings_x[readIndex_x]      = input;
  total_x     += readings_x[readIndex_x];
  readIndex_x += 1;

  if (readIndex_x >= numReadings_x) {
    readIndex_x = 0;
  }
  average_x = total_x / numReadings_x;
  return average_x;
}

float moving_y(float input) {  
  total_y     -= readings_y[readIndex_y];
  readings_y[readIndex_y]      = input;
  total_y     += readings_y[readIndex_y];
  readIndex_y += 1;

  if (readIndex_y >= numReadings_y) {
    readIndex_y = 0;
  }
  average_y = total_y / numReadings_y;
  return average_y;
  
}

float moving_z(float input) {  
  total_z     -= readings_z[readIndex_z];
  readings_z[readIndex_z]      = input;
  total_z     += readings_z[readIndex_z];
  readIndex_z += 1;

  if (readIndex_z >= numReadings_z) {
    readIndex_z = 0;
  }
  average_z = total_z / numReadings_z;
  return average_z;
}
