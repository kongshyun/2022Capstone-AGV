#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//initializing all the variables

#define LOOPTIME  100                          //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
volatile double v1,v2;                        //v1 = Right velocity, v2 = Left velocity
volatile double velocity_Right,velocity_Left; 
void extract_velocity_Right();
void extract_velocity_Left();
void publishSpeed(double time);

unsigned long lastMilli = 0;


const double radius = 0.032;                 //Wheel radius, in m 우리껀0.064
const double wheelbase = 0.17;              //Wheelbase, in m 우리건 0.17

///////////////////
double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s

double speed_act_left1 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left1 = 0;                    //Command speed for left wheel in m/s


double speed_act_left2 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left2 = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s

double speed_act_right1 = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right1 = 0;                   //Command speed for right wheel in m/s


double speed_act_right2 = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right2 = 0;                   //Command speed for right wheel in m/s

const double max_speed = 1.34;
//const double min_speed = 0.85;                //Max speed in m/s


void Serial_Event();

ros::NodeHandle nh;

//function that will be called when receiving command from host

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;   //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message
  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

  //@@@@@@@@@@@@@@@@@@@@@@@@추가한 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
  //m/s --> rad/s 단위환산 필요
  // 입력받은 command를 모터제어기로
  String cmd_str = "mvc=";  //command to moonwalker string variable
  String buf_velocity_R = ""; // temp buf for cmd_str
  String buf_velocity_L = ","; // temp buf for cmd_str
  
  //right
  speed_req_right *= 60/(2*3.14)/radius; //m/s를 rad/s로 단위 환산
  buf_velocity_R += String(speed_req_right);
  
  //left
  speed_req_left *= 60/(2*3.14)/radius;
  buf_velocity_L += String(speed_req_left);
  
  cmd_str += buf_velocity_R + buf_velocity_L;
  Serial3.println(cmd_str);
  //@@@@@@@@@@@@@@@@@@@@@@@@추가한 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

void setup() {
  Serial3.begin(115200);                   //Moonwalker 통신
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
}

void loop() {

  nh.spinOnce();
    //----------------------------------

    if ((millis() - lastMilli) >= LOOPTIME) {     //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    //----------------------------------

    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }
    extract_velocity_Right(); 
    extract_velocity_Left();
    publishSpeed(LOOPTIME);

}
//---------------------------------------------------------------------------------------------------------------------

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = velocity_Left;  //left wheel speed (in m/s)
  speed_msg.vector.y = velocity_Right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
void extract_velocity_Right() {
    String recv_str3 = "";
    String str_v1, str_v2;
    bool positive;
    Serial3.println("v");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v1 = positive ? v1_1 + v1_2 : v1_1 - v1_2;
    velocity_Right = v1*radius*2*3.141592/60;
}
void extract_velocity_Left(){
    String recv_str3 = "";
    String str_v1, str_v2;
    bool positive;
    Serial3.println("v2");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v2 = positive ? v1_1 + v1_2 : v1_1 - v1_2;
    velocity_Left = v2*radius*2*3.141592/60;
  }
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
