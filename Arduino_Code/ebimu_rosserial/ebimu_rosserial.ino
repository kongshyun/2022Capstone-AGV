/*
 * DC모터를 제어하고, ebimu값을 받아와 방향제어와 다른 아두이노 노드로 pub하는 노드
 * ax,ay,az,roll,pitch값을 Stabilizer노드로 pub.
 * yaw값은 여기서 받아 방향제어 함.
 */

 
#include<ros.h>
#include<Wire.h>


const float pi = 3.141592;              // 파이
const float g = 9.81;                   // 중력가속도

float  p_time;                          //---------------------------
float  p_velocity;                      //
float  p_length;                        // 프로파일 계산 함수 전역변수 선언
float  p_alpha;                         //
float  p_beta;                          //---------------------------
float  p_accel;

float a;                                // accel 현재 가속도
float theta;                            // 보정각도 프로파일 세타


unsigned long loop_start_time;          //루프 시작 시각 저장 변수
unsigned long cycle_start_time;         //for 루프 1 cycle 시작 시각 저장 변수
unsigned long dt;                       // 1 루프 타임

float sec=0.00;
float Back_time=500;
char  CMD;

float scale_3ms=0.02;
float scale_2ms=0.02;
float scale_1ms=0.01;


#define SBUF_SIZE 64

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

float theta_x , theta_y, theta_z;   // 역기구학 통과한 모터1,2,3 각도

float ang_x;
float ang_y;
float ang_z;
float accel_x;
float accel_y;
float accel_z;

float angle_x;                          // pitch
float angle_y;                          // roll
float angle_z;                          // yaw

float the_x, the_y, the_z;                     // 보정각도 프로파일 세타

unsigned long loop_dt;                       // 1 루프 타임
unsigned long sttime;

float Gimbal_Roll;
float Gimbal_Pitch;

float RPM;
float av_dt = 0;

float distance = 0.365/2;
float RPM_yaw;
float target_yaw;

const float ratio =    17;


////////////////////////////////////////////////////////////////////
/////////////////////// EBIMU FUNCTION /////////////////////////////
#define SBUF_SIZE 64

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

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
/////////////////////// EBIMU FUNCTION /////////////////////////////
////////////////////////////////////////////////////////////////////




//publisher
ros::Publisher imu_pub("imu_data", &imu);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);//ebimu
  Serial3.begin(115200);//agv
  Wire.begin();

  Serial.println("hello");

  //agv_set
  Serial3.println("co1=1");
  Serial3.println("co2=1");

  //ros_set
  nh.initNode();
  nh.advertise(imu_pub);//imu데이터를 Stabilizer노드로 pub함.
  
}

void loop() {
  nh.spinOnce();
  publish_imu();
  publish_encoder();
  
}

//imu데이터를 "imu_data"토픽으로 publish
void publish_imu(){
  float euler[6];
  
  if(EBimuAsciiParser(euler, 6))
  {
     Serial.print("\n\r");
     Serial.print(euler[0]);   Serial.print(" ");//Roll 
     Serial.print(euler[1]);   Serial.print(" ");//Pitch
     Serial.print(euler[2]);   Serial.print(" ");//Yaw
     
     Serial.print(euler[3]);   Serial.print(" ");//ax
     Serial.print(euler[4]);   Serial.print(" ");//ay
     Serial.print(euler[5]);   Serial.print(" ");//az
  }
}
