#include "ros/ros.h"

#include "std_msgs/Empty.h"


void cb (const std_msgs::Empty mess) {}


int main (int argc, char **argv) {
  ros::init(argc, argv, "ekf_localization_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Empty>("odometry/filtered", 10);

  ros::Subscriber sub1 = nh.subscribe("scan", 10, cb);
  ros::Subscriber sub2 = nh.subscribe("imu", 10, cb);
  ros::Subscriber sub3 = nh.subscribe("cmd_vel", 10, cb);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
