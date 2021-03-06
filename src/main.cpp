#include <Arduino.h>
#include "robot_config.h"
#include "WheelEncoder.h"
#include <EnableInterrupt.h>
#include "ros_init.h"
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <rosserial_arduino/Test.h>

WheelEncoder LR_enc(LR_ENCODER_PIN);
WheelEncoder RR_enc(RR_ENCODER_PIN);
void leftWheelSetValue(int value);
void rightWheelSetValue(int value);

unsigned long cmd_WatchdogMillis = 0;
boolean debug_pin_state;
int ov_L;
int ov_R;
int w_r = 0, w_l = 0;

// motor odometry interrupts
void LR_enc_ISR()
{
  if (w_l >= 0)
  {
    LR_enc.odom++;
  }
  else
  {
    LR_enc.odom--;
  }
}

void RR_enc_ISR()
{
  if (w_r >= 0)
  {
    RR_enc.odom++;
  }
  else
  {
    RR_enc.odom--;
  }
}

void cmd_velWatchdog()
{
  if (millis() - cmd_WatchdogMillis > 400)
  {
    w_l = 0;
    w_r = 0;
    ov_L = 0;
    ov_R = 0;
  }
}
void cmd_velWatchdogReset()
{
  cmd_WatchdogMillis = millis();
}

unsigned long outDel = 0; // ros message sent delay

void SendCurrentSpeedToROS();
void SendOdomToROS();

void SpeedMessageCallback(const geometry_msgs::Twist &msg)
{
  cmd_velWatchdogReset();
  w_r = (msg.linear.x + (msg.angular.z / 2.0)) * 1000;
  w_l = (msg.linear.x - (msg.angular.z / 2.0)) * 1000;
  w_r = constrain(w_r, -1000, 1000);
  w_l = constrain(w_l, -1000, 1000);
  ov_L = map(abs(w_l), 0, 1000, DR_PWM_MIN, DR_PWM_MAX - 50);
  ov_R = map(abs(w_r), 0, 1000, DR_PWM_MIN, DR_PWM_MAX - 50);
  ov_L = constrain(ov_L, DR_PWM_MIN, DR_PWM_MAX - 50);
  ov_R = constrain(ov_R, DR_PWM_MIN, DR_PWM_MAX - 50);

  if (w_l == 0 && w_r == 0)
  {
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

Generator gen;
// Generator start service
void gen_start(const std_msgs::Bool &msg)
{
  if (msg.data == true && gen.isStarted == false)
  {
    gen.start();
    gen.isStarted = true;
  }
  if (msg.data == false)
  {
    gen.stop();
    gen.isStarted = false;
  }
}
ros::Subscriber<std_msgs::Bool> gen_sub("/gen_start_stop", &gen_start);

void ConfigROS()
{
  nh.initNode();
  nh.advertise(LR_SpeedPublisher);
  nh.advertise(RR_SpeedPublisher);
  nh.advertise(LR_OdomPublisher);
  nh.advertise(RR_OdomPublisher);
  nh.advertise(LR_PwrPublisher);
  nh.advertise(RR_PwrPublisher);
  nh.subscribe(sub);
  nh.subscribe(gen_sub);
}

void SendCurrentSpeedToROS()
{

  LR_speed_msg.data = LR_enc.getLinearSpeed(WHEELRADIUS);
  RR_speed_msg.data = RR_enc.getLinearSpeed(WHEELRADIUS);

  LR_SpeedPublisher.publish(&LR_speed_msg);
  RR_SpeedPublisher.publish(&RR_speed_msg);

  RR_pwr_msg.data = w_l;
  RR_PwrPublisher.publish(&RR_pwr_msg);
  LR_pwr_msg.data = w_r;
  LR_PwrPublisher.publish(&LR_pwr_msg);

  SendOdomToROS();
}
void SendOdomToROS()
{

  LR_odom_msg.data = LR_enc.odom;
  RR_odom_msg.data = RR_enc.odom;

  LR_OdomPublisher.publish(&LR_odom_msg);
  RR_OdomPublisher.publish(&RR_odom_msg);
}

void ConfigPins()
{
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

  pinMode(DEBUG_PIN, OUTPUT);

  digitalWrite(RELAY_PWR_SUPPLY, HIGH);
  digitalWrite(RELAY_CONTROLLER, HIGH);
}

void setup()
{
  ConfigPins();
  ConfigROS();
  enableInterrupt(RR_ENCODER_PIN, RR_enc_ISR, RISING);
  enableInterrupt(LR_ENCODER_PIN, LR_enc_ISR, RISING);
}

void loop()
{
  cmd_velWatchdog();
  
  if (DEBUG_IS_ENABLE)
  {
    debug_pin_state ^= true;
    digitalWrite(DEBUG_PIN, debug_pin_state);
  }

  LR_enc.tick();
  RR_enc.tick();

  if (w_l != 0)
  {
    leftWheelSetValue(ov_L);
  }
  else
  {
    leftWheelSetValue(0);
  }

  if (w_r != 0)
  {
    rightWheelSetValue(ov_R);
  }
  else
  {
    rightWheelSetValue(0);
  }
  if (millis() - outDel >= 100)
  {
    outDel = millis();
    SendCurrentSpeedToROS();
  }
  nh.spinOnce();
}

void leftWheelSetValue(int value)
{
  if (w_l >= 0)
  {
    analogWrite(L_DR_PWM, abs(value));
    digitalWrite(L_DR_DIR, LOW);
  }
  else
  {
    analogWrite(L_DR_PWM, abs(value));
    digitalWrite(L_DR_DIR, HIGH);
  }
}
void rightWheelSetValue(int value)
{
  if (w_r >= 0)
  {
    analogWrite(R_DR_PWM, abs(value));
    digitalWrite(R_DR_DIR, LOW);
  }
  else
  {
    analogWrite(R_DR_PWM, abs(value));
    digitalWrite(R_DR_DIR, HIGH);
  }
}