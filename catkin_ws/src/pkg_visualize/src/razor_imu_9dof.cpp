#include "ros/ros.h"

#include "std_msgs/Empty.h"


int main (int argc, char **argv) {
  ros::init(argc, argv, "razor_imu_9dof");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Empty>("imu", 10);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
