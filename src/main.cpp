#include "trajectory_action2topic.hpp"

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_action2topic");
  ros::NodeHandle m_rosNode;

  TrajectoryAction2Topic action2topic(ros::this_node::getName());

  ROS_INFO("Spinning");
  ros::spin();

  return 0;
}
