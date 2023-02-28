
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace to_twist
{
class joyToTwsistStamped
{
public:
  joyToTwsistStamped() : spinner_(1)
  {
    joy_sub_ = n_.subscribe("joy", 1, &joyToTwsistStamped::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("twiststaped_out", 1);
    
    spinner_.start();
    ros::waitForShutdown();
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::AsyncSpinner spinner_;

  // Convert incoming joy commands to TwistStamped
  
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = msg->axes[1];
    twist.twist.linear.y = msg->axes[0];
    twist.twist.linear.z = ((msg->axes[2] * -0.5) + 0.5) * msg->axes[7];    // [Y = -0.5X + 0.5] * sign 
    twist.twist.angular.x = ((msg->axes[5] * -0.5) + 0.5) * msg->axes[7];
    twist.twist.angular.y = msg->axes[4];
    twist.twist.angular.z = msg->axes[3];

    twist_pub_.publish(twist);
    
  }
};
}  // end to_twist namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_to_twsiststamped");

  to_twist::joyToTwsistStamped to_twist;

  return 0;
}