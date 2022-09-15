// AGV_teleop_key.py에서 Twist를 받아 속도명령

// Hello도 발행
//AGV_teleop_key 노드에서 키보드제어를 받고, "Hello"문자열을 발행하는 노드
#include <ros.h>
#include <Wire.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include<std_msgs/String.h>

#define LOOPTIME  100                         //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

const double radius = 0.032;  //바퀴 반지름[m] 우리껀0.064
const double width = 0.372;  //두 바퀴 사이 거리[m]
double linear_speed_cmd = 0; //AGV 선형 속도[m/s]
double angular_speed_cmd = 0; //AGV 각속도[rad/s]
double speed_cmd_left = 0;  //왼쪽 바퀴 속도[rpm]
double speed_cmd_right = 0; //오른쪽 바퀴 속도[rpm]



void publlish_encoder();

ros::NodeHandle  nh;

// cmd_vel 콜백 함수
void AGVcontrol_cmd (const geometry_msgs::Twist& cmd_vel){
  noCommLoops = 0;   //Reset the counter for number of main loops without communication

  // cmd_vel에서 속도 추출
  linear_speed_cmd = cmd_vel.linear.x;
  angular_speed_cmd = cmd_vel.angular.z;
  
  speed_cmd_left = (linear_speed_cmd - (angular_speed_cmd * width / 2)) * 60/(2*3.14)/radius;
  speed_cmd_right = (linear_speed_cmd + (angular_speed_cmd * width / 2)) * 60/(2*3.14)/radius;

  // 모터제어기 명령 command
  String mvc_cmd = "mvc=";
  String comma = ",";

  mvc_cmd += String(speed_cmd_right) + comma + String(speed_cmd_left);
  Serial.println("Hello");
  
  Serial.println(mvc_cmd); // 속도명령을 내리고 엔코더값을 읽어옴.

  publlish_encoder();// 엔코더 값을 읽어 퍼블리시하는 추가 함수
}

// subscriber
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", AGVcontrol_cmd);
// publisher

//geometry_msgs::PointStamped wheel;

std_msgs::String wheel;
ros::Publisher enc_pub("encoder_data", &wheel);


void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_vel);
  nh.advertise(enc_pub);

}

void loop()
{
  
  nh.spinOnce();
//  noCommLoops++;
//  if (noCommLoops == 65535)
//  {
//    noCommLoops = noCommLoopMax;
//  }
  delay(200);
}

char hi[10]="HELLO";
void publlish_encoder(){

  wheel.data=hi;
  enc_pub.publish(&wheel);
}
