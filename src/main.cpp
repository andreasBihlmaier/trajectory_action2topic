#include "trajectory_action2topic.hpp"

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_action2topic");
  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandlePrivate("~");

  TrajectoryAction2Topic action2topic("joint_trajectory_action");

  std::string rename_joints_for;
  if (nodeHandlePrivate.getParam("rename_joints_for", rename_joints_for)) {
    std::string robot_description;
    if (!nodeHandle.getParam("robot_description", robot_description)) {
      ROS_ERROR("rename_joints_for is set, but no robot_description available. Quitting.");
      return 1;
    }
    action2topic.rename_joints(rename_joints_for, robot_description);
  }

  ROS_INFO("Spinning");
  ros::spin();

  return 0;
}
