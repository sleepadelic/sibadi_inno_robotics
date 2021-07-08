#include "robot_config.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include "Generator.h"

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


