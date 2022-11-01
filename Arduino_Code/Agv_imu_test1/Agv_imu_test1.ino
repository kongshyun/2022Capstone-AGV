/*
 * Serial_node.py
 * AGV + Stabil dual rosserial test1
 * DC모터 구동과 yaw방향제어 및 ebimu센서값 Stabil노드로 pub 
 */
/////////////////////////// 받은 속도값으로 모터 제어+엔코더 값을 읽어 퍼블리시하는 node
#include <ros.h>
#include<std_msgs/Int16.h>

#include <geometry_msgs/PointStamped.h>
#include<ros/time.h>


const float pi = 3.141592;              // 파이
const float g = 9.81;                   // 중력가속도

// 프로파일 계산 함수 전역변수 선언
float  p_time; float  p_velocity;float  p_length;float  p_alpha;float  p_beta;float  p_accel;    

float a;                                // accel 현재 가속도
float theta;                            // 보정각도 프로파일 세타

unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임

char  CMD;

#define SBUF_SIZE 64

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

float theta_x , theta_y, theta_z;   // 역기구학 통과한 모터1,2,3 각도

float ang_x;float ang_y;float ang_z;float accel_x;float accel_y;float accel_z;

//pitch, roll, yaw
float angle_x;float angle_y;float angle_z;                         

float the_x, the_y, the_z;                     // 보정각도 프로파일 세타

unsigned long loop_dt;                       // 1 루프 타임
unsigned long sttime;

float Gimbal_Roll;float Gimbal_Pitch;

float RPM;
float av_dt = 0;

float distance = 0.365/2;
float RPM_yaw;
float target_yaw;

const float ratio =    17;

ros::NodeHandle  nh;


// cmd_vel 콜백 함수(Key_profile.py노드에서 받은 data로 agv dc모터 제어)
void AGVcontrol_cmd (const std_msgs :: Int16& cmd_vel) {
  
  int key=cmd_vel.data;//명령 받은 키
    
  ///////////////////////////                                                           
    loop_start_time = millis();
    ///////////////////////////
    sttime          = millis(); 
     
  //ros 명령 받으면 1ms속도 시나리오 
  if (key >5 )
  {
    Agv_accel(7,0.70);
    Agv(0.7);
    Agv_decel(7,0.70);
    delay(2000);
    Agv_turn(0.5,0.05);
    delay(2000);
    Agv_accel(5,0.5);
    Agv_decel(5,0.5);
  }
  
  else if ( key>0 && key<4)
  {
    Serial3.println("co1=0");Serial3.println("co2=0");
    Serial3.println("mvc=0,0");
  }
  timecycle(); // -주기 계산 함수
}
// subscriber
ros::Subscriber<std_msgs::Int16> cmd_vel("cmd_vel", AGVcontrol_cmd);

//publisher (roll, pitch값을 /imu토픽으로 pub)
geometry_msgs::PointStamped imu;
ros::Publisher imu_pub("imu", &imu);

void setup() {
  Serial.begin(57600);
  Serial2.begin(57600);
  Serial3.begin(57600);
  
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(imu_pub);

}

void loop() {
  nh.spinOnce();//rosserial을 통해 계속해서 키보드 값을 받는다.
  publish_imu_accel();
  delay(1);
}//loop//-----------------------------------------------------------------------------------------



//ebimu roll, pitch 값을 serial_node2로 publish
void publish_imu_accel(){
  float euler[3];
  
  if(EBimuAsciiParser(euler, 3))
  {
     imu.point.x=euler[0];//Roll
     imu.point.y=euler[1];//Pitch
     imu.header.stamp=nh.now();
     imu_pub.publish(&imu);
  }
}

void IMU_Sensing(){

   float euler[3];
    if(EBimuAsciiParser(euler, 3))
    {
      angle_x      = euler[0];//roll
      angle_y      = euler[1];//pitch
      angle_z      = euler[2];//yaw
      
      Gimbal_Pitch      =  angle_x + (  0  ); 
      Gimbal_Roll       =  angle_y + (  0  );
  }

}  


int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial2.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial2.read();
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

void Agv_turn(float highest_theta, float Length){
  
  float Vel;
  float Accel;
  float cmd_rpm;  //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어

  Serial3.println("co1=1");Serial3.println("co2=1");
 
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec=0; sec < p_time;  sec += 0.02){
    cycle_start_time = millis();

    IMU_Sensing();
            
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    RPM     =  Vel*60/(3.14*0.064);
    
    cmd_rpm =  RPM;
    cmd_str =  "mvc="+String(-cmd_rpm)+","+String(-cmd_rpm);
    Serial3.println(cmd_str);                                     //모터에명령통신

    //Plate_angle(0,0);
    
    Calculate_delay( p_time*1000,  p_time*50 );
  }
    for(float sec= p_time ; sec > 0;  sec -= 0.02){
    cycle_start_time = millis();

    IMU_Sensing();
        
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    RPM     =  Vel*60/(3.14*0.064);
    cmd_rpm =  RPM;
    cmd_str =  "mvc="+String(-cmd_rpm)+","+String(-cmd_rpm);
    Serial3.println(cmd_str);                                     //모터에명령통신

    
    //Plate_angle(0,0);
    //Serial.print("theta=  "); Serial.println(theta);
    
    Calculate_delay( p_time*1000,  p_time*50 );
  }
}


void Agv_accel( float highest_theta, float Length){

  float Vel;
  float Accel;
  float cmd_rpm;  //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어

  Serial3.println("co1=1");Serial3.println("co2=1");
 
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec=0; sec < p_time;  sec += 0.02){
    cycle_start_time = millis();

    IMU_Sensing();
    RPM_yaw   = (target_yaw - angle_z) * distance * ratio /PI/0.064;
        
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    RPM     =  Vel*60/(3.14*0.064);
    
    cmd_rpm =  RPM;
    cmd_str =  "mvc="+String(cmd_rpm)+","+String(-cmd_rpm);
    Serial3.println(cmd_str);                                     //모터에명령통신

    Calculate_delay( p_time*1000,  p_time*50 );
  }
}

void Agv(float Time){
  
  float cmd_rpm;                   //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  
  av_dt = Time;
  for(float sec=0; sec < Time;  sec += 0.02){
    cycle_start_time = millis();
    
    IMU_Sensing();

    cmd_rpm =  RPM;
    cmd_str =  "mvc="+String(cmd_rpm)+","+String(-cmd_rpm);
    Serial3.println(cmd_str);                                     //모터에명령통신
   
    Calculate_delay( p_time*1000,  p_time*50 );
  } 
}

void Agv_decel( float highest_theta, float Length){

  float Vel;
  float Accel;
  float RPM;
  float cmd_rpm;  //모터에 부여할 RPM
  String cmd_str;                  //모터에 줄 속도 명령어
  
  //////////////////////////////////////////////////////////////////////////////////
  profile_maker( highest_theta, Length); // 시간, 속도, 최대 가속도, 계수 알파 베타를 반환함
  //////////////////////////////////////////////////////////////////////////////////
  
  for(float sec= p_time ; sec > 0;  sec -= 0.02){
    cycle_start_time = millis();

    IMU_Sensing();
        
    Vel     = -pow(sec,3) * (p_alpha/3) + pow(sec,2)* (p_beta/2); // 속도 
    Accel   = -pow(sec,2) *  p_alpha    + pow(sec,1)*  p_beta;    // 가속도
    theta   =  atan(Accel/g)*180/3.14;    
    RPM     =  Vel*60/(3.14*0.064);
    cmd_rpm =  RPM;
    
   
    cmd_str =  "mvc="+String(cmd_rpm)+","+String(-cmd_rpm);
    Serial3.println(cmd_str);                                     //모터에명령통신

    
    Calculate_delay( p_time*1000,  p_time*50 );
    
  }

  av_dt = 0 ;
}


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


void Plate_angle(float Roll, float Pitch){
  
  Calculate_Roll_Pitch(Roll, Pitch);

  
  
}//Plate_angle//

void timecycle(){//ㅡㅡㅡㅡㅡㅡㅡㅡ
  float t;                            
  t = millis() - loop_start_time;
  dt = t;
  
  //Serial.print("loop time : ");
  //Serial.print(dt);
  //Serial.println();
                                       
}//timecycle

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


void Calculate_Roll_Pitch(float Target_roll, float Target_pitch) {//ㅡㅡㅡㅡㅡㅡ
  // Target_roll = 원하는 롤 값,  Target_pitch = 원하는 피치값                        
  // link_A = 링크 1의 길이 , link_B = 링크2의 길이                                     
                                                                                     
  float link_A1 = 40;
  float link_A2 = 40;
  float link_A3 = 40;
  float link_B1 = 61;
  float link_B2 = 61;
  float link_B3 = 61;
  float height_data = 67; 

  float Length_high, Length_low;

  Length_high = 110; // 윗면의 삼각형 한 변의 길이
  Length_low  = 35; // 아랫면의 삼각형 한 변의 길이
  
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

  float x = Target_roll * (pi / 180); // 각도를 라디안으로 변환함
  float y = Target_pitch * (pi / 180);

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

    theta_x += 90; //모터가 세워져있기 때문에 
    theta_y += 90 + 5;
    theta_z += 90 -50;   

    theta_x = constrain(theta_x, 30, 160); //위치 제한
    theta_y = constrain(theta_y, 30, 160);
    theta_z = constrain(theta_z, 30, 160);
                                                                
}/////////함수 끝//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


float length_to_motor_degree(float x_data, float y_data, float link_A,
float link_B) {
                                                                                                                  
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
