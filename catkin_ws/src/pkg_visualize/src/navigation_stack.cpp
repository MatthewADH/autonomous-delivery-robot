#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"


void cb (const std_msgs::Empty mess) {}


int main (int argc, char **argv) {
  ros::init(argc, argv, "navigation_stack");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Subscriber sub1 = nh.subscribe("scan", 10, cb);
  ros::Subscriber sub2 = nh.subscribe("odometry/filtered", 10, cb);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
