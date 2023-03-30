#include <ros.h>
#include <std_msgs/Byte.h>


ros::NodeHandle nh;

std_msgs::Byte button_msg;
ros::Publisher pub_button("button", &button_msg);



void setup() {
  nh.initNode();
  nh.advertise(pub_button);

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
}


void loop() {
  uint8_t reading = 0;
  static uint32_t pre_time;


  if (digitalRead(BDPIN_PUSH_SW_1) == HIGH) {
    reading |= 0x01;
  }
  if (digitalRead(BDPIN_PUSH_SW_2) == HIGH) {
    reading |= 0x02;
  }

  if (millis()-pre_time >= 50) {
    button_msg.data = reading;
    pub_button.publish(&button_msg);
    pre_time = millis();
  }

  nh.spinOnce();
}
