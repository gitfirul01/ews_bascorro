/*
  Range   : Roll  : +/- 180 deg/sec
            Pitch : +/- 180 deg/sec
            Yaw   : +/- 180 deg/sec
  Scale   : Roll  : 1 = 1 deg/sec
            Pitch : 1 = 1 deg/sec
            Yaw   : 1 = 1 deg/sec
 */

#include <IMU.h>
#include <ros.h>
#include <std_msgs/String.h>


cIMU    IMU;

uint8_t   err_code;
uint8_t   led_tog = 0;
uint8_t   led_pin = 13;

ros::NodeHandle  nh;

std_msgs::String imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);    // imu_pub: node, imu: topic


void setup()
{
  nh.initNode();
  nh.advertise(imu_pub);

  IMU.begin();

  pinMode( led_pin, OUTPUT );
}





void loop()
{
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;


  if( (millis()-tTime[0]) >= 500 )
  {
    tTime[0] = millis();

    digitalWrite( led_pin, led_tog );
    led_tog ^= 1;
  }

  tTime[2] = micros();
  if( IMU.update() > 0 ) imu_time = micros()-tTime[2];



  if( (millis()-tTime[1]) >= 50 )
  {
    tTime[1] = millis();

    String imu = String(IMU.rpy[0]) + " " + String(IMU.rpy[1]) + " " + String(IMU.rpy[2]);

    char buff[20];
    imu.toCharArray(buff, sizeof(buff));
    
    imu_msg.data = buff;
    imu_pub.publish(&imu_msg);
    nh.spinOnce();
  }
}
