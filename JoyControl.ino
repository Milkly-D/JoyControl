#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <LSM303.h>
#include <Zumo32U4.h>
#include <geometry_msgs/Twist.h>

long timer = 0;
int vright = 0;
int vleft = 0;
int basespeed = 100;
int16_t positionLeft = 0;
int16_t positionRight = 0;
int16_t newLeft = 0;
int16_t newRight = 0;
float linear_x = 0.0f;
float angle_z = 0.0f;
float threshold_val = 0.8f;
std_msgs::String str_msg;

LSM303 compass;
L3G gyro;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
ros::NodeHandle nh;

void motorcontrol(const geometry_msgs::Twist& twist)
{
  linear_x = twist.linear.x;
  angle_z = twist.angular.z;

  if((abs(linear_x)<=threshold_val)&&(abs(angle_z)<=threshold_val)){
    motors.setSpeeds(0,0);
    delay(2);
  }else if(linear_x != 0.0f){
    if(linear_x > 0){
      motors.setSpeeds(0,0);
      delay(2);
      vleft = basespeed;
      vright = basespeed;
      motors.setSpeeds(vleft,vright);
    }else if(linear_x <0){
      motors.setSpeeds(0,0);
      delay(2);
      vleft = -1*basespeed;
      vright = -1*basespeed;
      motors.setSpeeds(vleft,vright);
    }else{
      motors.setSpeeds(0,0);
      delay(2);
    }
  }else{
    if(angle_z > 0.0f){
      motors.setSpeeds(0,0);
      delay(2);
      vleft = -1*(basespeed+120);
      vright = (basespeed+120);
      motors.setSpeeds(vleft,vright);
    }else if(angle_z < 0.0f){
      motors.setSpeeds(0,0);
      delay(2);
      vleft = (basespeed+120);
      vright = -1*(basespeed+120);
      motors.setSpeeds(vleft,vright);
   }else{
      motors.setSpeeds(0,0);
      delay(2);
   }
 }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motorcontrol);
ros::Publisher chatter("/sensorval", &str_msg);

void setup() {
  Wire.begin();

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  nh.initNode();              // Init ROS Node
  nh.advertise(chatter);      // Init ROS Publisher
  nh.subscribe(sub);          // Init ROS Subscriber

  compass.init();             // Init magnetmeter
  compass.enableDefault();

  gyro.init();                // Init gyrometer
  gyro.enableDefault();
}

void loop() {
  compass.read();
  gyro.read();
  timer = millis();
  newLeft = encoders.getCountsAndResetLeft();
  newRight = encoders.getCountsAndResetRight();
  if(!(encoders.checkErrorLeft()) && !(encoders.checkErrorRight())){
    positionLeft = newLeft;
    positionRight = newRight;
  }
  String s = "";
  s += timer;
  s += ',';
  s += compass.a.x;
  s += ',';
  s += compass.a.y;
  s += ',';
  s += compass.a.z;
  s += ',';
  s += compass.m.x;
  s += ',';
  s += compass.m.y;
  s += ',';
  s += compass.m.z;
  s += ',';
  s += vleft;
  s += ',';
  s += vright;
  s += ',';
  s += positionLeft;
  s += ',';
  s += positionRight;
  s += ',';
  s += gyro.g.x;
  s += ',';
  s += gyro.g.y;
  s += ',';
  s += gyro.g.z;
  
  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
 }
