#include <ros.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>

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

ros::NodeHandle  nh;

geometry_msgs::PointStamped wheel;
ros::Publisher mpu_pub("imu_data", &wheel);


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


float compX;


void setup() {
  
  Wire.begin();
  Serial.begin(57600);
  
  accelgyro.initialize();
  accelgyro.setDLPFMode(MPU9250_DLPF_BW_20);  // 저역통과필터
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(mpu_pub);

}

void publish_imu(){
  wheel.point.x=Axyz[0];
  wheel.point.y=Axyz[1];
  wheel.point.z=Axyz[2];
  mpu_pub.publish(&wheel);
}
void loop() {

   
    getAccel_Data();
    getGyro_Data();
    /*
    Serial.print("ax = ");
    Serial.println(Axyz[0]);
    Serial.print("ay = ");
    Serial.println(Axyz[1]);
    Serial.print("az = ");
    Serial.println(Axyz[2]);
*/
    nh.spinOnce();
    delay(200);
}
