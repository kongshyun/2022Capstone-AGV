/*
 * rosserial-> serial_node2
 * imu데이터를 /imu 토픽에서 sub 테스트 하는 코드
 */

#include <ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include<std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include<ros/time.h>

float roll;
float pitch;
float theta;
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
  imu_data.publish(&roll_pitch_theta);
}

// subscriber
ros::Subscriber<geometry_msgs::PointStamped> imu("imu",EBimu_callback);



void setup() {
  
  nh.initNode();
  
  nh.subscribe(imu);
  nh.advertise(imu_data);

}

void loop() {
  nh.spinOnce();
  delay(1);

}
