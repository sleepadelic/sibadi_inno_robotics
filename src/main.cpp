#include <Arduino.h>
#include "robot_config.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "WheelEncoder.h"
#include "GyverPID.h"
#include <EnableInterrupt.h>
#include "Generator.h"
#include <std_srvs/Empty.h>


WheelEncoder LR_enc(LR_ENCODER_PIN);
WheelEncoder RR_enc(RR_ENCODER_PIN);
void leftWheelSetValue(int value);
void rightWheelSetValue(int value);

double w_r=0, w_l=0;

void LR_enc_ISR(){
  if(w_l>=0){
    LR_enc.odom++;
  }
  else
  {
    LR_enc.odom--;
  }
}

void RR_enc_ISR(){
  if(w_r>=0){
    RR_enc.odom++;
  }
  else
  {
    RR_enc.odom--;
  }
}

unsigned long outDel = 0;
void SendCurrentSpeedToROS();
void SendOdomToROS();
//ROS_INIT
using std_srvs::Empty;
ros::NodeHandle nh;
std_msgs::Float32 LR_speed_msg;
ros::Publisher LR_SpeedPublisher("LR_speed", &LR_speed_msg);
std_msgs::Float32 RR_speed_msg;
ros::Publisher RR_SpeedPublisher("RR_speed", &RR_speed_msg);
std_msgs::Float32 LR_odom_msg;
ros::Publisher LR_OdomPublisher("LR_ticks", &LR_odom_msg);
std_msgs::Float32 RR_odom_msg;
ros::Publisher RR_OdomPublisher("RR_ticks", &RR_odom_msg);
std_msgs::Float32 LR_pwr_msg;
ros::Publisher LR_PwrPublisher("LR_pwr", &LR_pwr_msg);
std_msgs::Float32 RR_pwr_msg;
ros::Publisher RR_PwrPublisher("RR_pwr", &RR_pwr_msg);

Generator gen;
// Generator start service
void gen_start(const Empty::Request & req, Empty::Response & res)
{
  gen.start();
}
ros::ServiceServer<Empty::Request, Empty::Response> gen_starter_service("gen_start", &gen_start);

// Generator stop service
void gen_stop(const Empty::Request & req, Empty::Response & res)
{
  gen.stop();
}
ros::ServiceServer<Empty::Request, Empty::Response> gen_stopper_service("gen_stop", &gen_start);


GyverPID L_regulator(100, 10, 0, 10);
GyverPID R_regulator(100, 10, 0, 10);

void SpeedMessageCallback( const geometry_msgs::Twist& msg){

  w_r = msg.linear.x + (msg.angular.z/2.0);
  w_l = msg.linear.x - (msg.angular.z/2.0);
  L_regulator.setpoint = abs(w_l);
  R_regulator.setpoint = abs(w_r);
  
  if (w_l ==0 && w_r==0){
    digitalWrite(RELAY_PWR_SUPPLY, LOW);
    digitalWrite(RELAY_CONTROLLER, LOW);
  }
  else
  {
    digitalWrite(RELAY_PWR_SUPPLY, HIGH);
    digitalWrite(RELAY_CONTROLLER, HIGH);
  }
} 

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &SpeedMessageCallback); //Command topic subsciber

void ConfigPins(){
  // DRIVE CONTROL
  pinMode(R_DR_DIR, OUTPUT);
  pinMode(L_DR_DIR, OUTPUT);
  pinMode(R_DR_PWM, OUTPUT);
  pinMode(L_DR_PWM, OUTPUT);
  // MAIN CONTROL
  pinMode(RELAY_CONTROLLER, OUTPUT);
  pinMode(RELAY_PWR_SUPPLY, OUTPUT);
  // GEN CONTROL
  pinMode(RELAY_GEN_STARTER, OUTPUT);
  pinMode(RELAY_GEN_STOPPER, OUTPUT);
  digitalWrite(RELAY_PWR_SUPPLY, HIGH);
  digitalWrite(RELAY_CONTROLLER, HIGH);
  
}
void ConfigROS(){
  nh.initNode();
  nh.advertise(LR_SpeedPublisher);
  nh.advertise(RR_SpeedPublisher);
  nh.advertise(LR_OdomPublisher);
  nh.advertise(RR_OdomPublisher);
  nh.advertise(LR_PwrPublisher);
  nh.advertise(RR_PwrPublisher);
  nh.subscribe(sub);

  nh.advertiseService(gen_starter_service);
  nh.advertiseService(gen_stopper_service);
}
void ConfigPIDs(){
  L_regulator.setLimits(0,  255);
  L_regulator.setpoint = 0;
  R_regulator.setLimits(0,  255);
  R_regulator.setpoint = 0;
}
void setup() {
  ConfigPins();
  ConfigROS();
  ConfigPIDs();
  enableInterrupt(RR_ENCODER_PIN, RR_enc_ISR, RISING);
  enableInterrupt(LR_ENCODER_PIN, LR_enc_ISR, RISING);
}


void loop() {

  LR_enc.tick();
  RR_enc.tick();

  L_regulator.input = abs(LR_enc.getLinearSpeed(WHEELRADIUS));
  R_regulator.input = abs(RR_enc.getLinearSpeed(WHEELRADIUS));
  RR_pwr_msg.data = R_regulator.getResultTimer();
  RR_PwrPublisher.publish(&RR_pwr_msg);
  LR_pwr_msg.data = L_regulator.getResultTimer();
  LR_PwrPublisher.publish(&LR_pwr_msg);
  
  if (w_l !=0){ 
    leftWheelSetValue(L_regulator.getResultTimer());
  }
  else
  {
    leftWheelSetValue(0);
  }
  
  if (w_r !=0){
    rightWheelSetValue(R_regulator.getResultTimer());
  }
  else
  {
    rightWheelSetValue(0);
  }
  SendCurrentSpeedToROS();
  nh.spinOnce();
}

void SendCurrentSpeedToROS(){
  if (millis()-outDel >= 100)
  {
    LR_speed_msg.data = LR_enc.getLinearSpeed(WHEELRADIUS);
    RR_speed_msg.data = RR_enc.getLinearSpeed(WHEELRADIUS);

    LR_SpeedPublisher.publish(&LR_speed_msg);
    RR_SpeedPublisher.publish(&RR_speed_msg);
    SendOdomToROS();
    outDel = millis();
  }
}
void SendOdomToROS(){

  LR_odom_msg.data = LR_enc.odom;
  RR_odom_msg.data = RR_enc.odom;

    LR_OdomPublisher.publish(&LR_odom_msg);
    RR_OdomPublisher.publish(&RR_odom_msg);
}
void leftWheelSetValue(int value){
  if (w_l>=0)
  {
    analogWrite(L_DR_PWM, abs(value));
    digitalWrite(L_DR_DIR, LOW);
  }else
  {
    analogWrite(L_DR_PWM, abs(value));
    digitalWrite(L_DR_DIR, HIGH);
  }
}
void rightWheelSetValue(int value){
  if (w_r>=0)
  {
    analogWrite(R_DR_PWM, abs(value));
    digitalWrite(R_DR_DIR, LOW);
  }else
  {
    analogWrite(R_DR_PWM, abs(value));
    digitalWrite(R_DR_DIR, HIGH);
  }
}