#include "trajectory_action2topic.hpp"

// system includes
#include <numeric>

// library includes
#include <ahbstring.h>
#include <urdf/model.h>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
std::vector<double>
TrajectoryAction2Topic::reorderJoints(JointMapType jointMap, std::vector<double> input)
{
  std::vector<double> output;

  for (size_t jointIdx = 0; jointIdx < input.size(); jointIdx++) {
    output.push_back(input[jointMap[jointIdx]]);
  }

  return output;
}

TrajectoryAction2Topic::JointMapType
TrajectoryAction2Topic::permutationMap(const std::vector<std::string>& targetOrder, const std::vector<std::string>& origOrder)
{
  JointMapType jointMap;

  for (size_t nameIdx = 0; nameIdx < origOrder.size(); nameIdx++) {
    std::vector<std::string>::const_iterator foundIt = std::find(targetOrder.begin(), targetOrder.end(), origOrder[nameIdx]);
    if (foundIt == targetOrder.end()) {
      return JointMapType();
    }
    jointMap[foundIt - targetOrder.begin()] = nameIdx;
  }

  return jointMap;
}

TrajectoryAction2Topic::TrajectoryAction2Topic(const std::string& actionName)
  :m_actionName(actionName),
   m_actionServer(m_nh, actionName, boost::bind(&TrajectoryAction2Topic::onGoal, this, _1), false),
   m_rename_joints(false)
{
  m_jointPub = m_nh.advertise<sensor_msgs::JointState>("set_joint", 1);
  m_jointSub = m_nh.subscribe<sensor_msgs::JointState>("get_joint", 1, boost::bind(&TrajectoryAction2Topic::onJoint, this, _1));
  m_actionServer.start();
}

void
TrajectoryAction2Topic::onJoint(const sensor_msgs::JointState::ConstPtr& joints)
{
  m_currJointState = *joints;
}

void
TrajectoryAction2Topic::onGoal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  ROS_INFO_STREAM("onGoal():\n" << *goal << "---");

  // TODO INVALID_GOAL

  // sort joints according to m_currJointState
  JointMapType jointMap;
  if (!m_currJointState.name.empty()) {
    if (m_rename_joints) {
      jointMap = permutationMap(m_renamed_curr_joints, goal->trajectory.joint_names);
    } else {
      jointMap = permutationMap(m_currJointState.name, goal->trajectory.joint_names);
    }
    if (jointMap.size() != m_currJointState.name.size()) {
      m_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      m_actionServer.setSucceeded(m_result);
      return;
    }
    ROS_INFO_STREAM("m_currJointState.name: " << ahb::string::toString(m_currJointState.name) << " goal->trajectory.joint_names: " << ahb::string::toString(goal->trajectory.joint_names) << " jointMap: " << ahb::string::toString(jointMap));
  } else {
    for (size_t jointIdx = 0; jointIdx < goal->trajectory.points[0].positions.size(); jointIdx++) {
      jointMap[jointIdx] = jointIdx;
    }
  }

  bool success = true;
  m_targetJointState.name = m_currJointState.name;
  m_feedback.joint_names = m_currJointState.name;
  ros::Time startTime = ros::Time::now();
  for (size_t pointIdx = 0; pointIdx < goal->trajectory.points.size(); pointIdx++) {
    const trajectory_msgs::JointTrajectoryPoint& currPoint = goal->trajectory.points[pointIdx];
    ROS_INFO_STREAM("Point " << pointIdx << ":\n" << currPoint);

    m_targetJointState.header.seq++;
    m_targetJointState.position = reorderJoints(jointMap, currPoint.positions);
    m_targetJointState.velocity = reorderJoints(jointMap, currPoint.velocities);

    if (m_actionServer.isPreemptRequested()) {
      ROS_INFO("Preempted at point %zd", pointIdx);
      m_actionServer.setPreempted();
      success = false;
      break;
    }

    m_targetJointState.header.stamp = ros::Time::now();
    m_jointPub.publish(m_targetJointState);

    // wait until the time at which the point should be reached
    ros::Duration sinceStart = ros::Time::now() - startTime;
    ros::Duration sleepDuration = currPoint.time_from_start - sinceStart;
    if (sleepDuration.toSec() > 0.0) {
      ROS_INFO_STREAM("sinceStart=" << sinceStart << " will sleeping for " << sleepDuration);
      sleepDuration.sleep();
    }

    m_feedback.desired = currPoint;
    m_feedback.actual.positions = m_currJointState.position;
    m_feedback.actual.velocities = m_currJointState.velocity;
    sinceStart = ros::Time::now() - startTime;
    m_feedback.actual.time_from_start = sinceStart;
    m_feedback.error.positions = m_targetJointState.position - m_feedback.actual.positions;
    m_feedback.error.velocities = m_targetJointState.velocity - m_feedback.actual.velocities;
    m_actionServer.publishFeedback(m_feedback);

    //double accumulated_position_error = std::accumulate(m_feedback.error.positions.begin(), m_feedback.error.positions.end(), 0.0, plusabs<double>);
    bool limit_exceeded = false;
    for (size_t jointIdx = 0; jointIdx < m_feedback.error.positions.size(); jointIdx++) { // replace by std::any_of once C++11 is available ...
      if (m_feedback.error.positions[jointIdx] > m_max_position_error) {
        limit_exceeded = true;
        break;
      }
    }
    if (limit_exceeded) {
      ROS_ERROR_STREAM("Too high position error: " << ahb::string::toString(m_feedback.error.positions));
      m_result.error_code = m_result.PATH_TOLERANCE_VIOLATED;
      m_actionServer.setAborted(m_result);
      success = false;
      break;
    }
  }

  //TODO GOAL_TOLERANCE_VIOLATED

  if (success) {
    m_result.error_code = m_result.SUCCESSFUL;
    m_actionServer.setSucceeded(m_result);
  }
}

void
TrajectoryAction2Topic::rename_joints(const std::string& robot_name, const std::string& robot_description)
{
  //std::cout << robot_description << std::endl;
  urdf::Model model;
  if (!model.initString(robot_description)) {
    ROS_ERROR("Failed to parse robot_description URDF. Aborting.");
    return;
  }

  ROS_INFO("Waiting to receive joint message");
  sensor_msgs::JointStateConstPtr joint_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("get_joint", m_nh);

  // TODO see joint_topic_merger.py:get_target_joint_names()
  for (std::vector<std::string>::const_iterator origIter = joint_msg->name.begin(); origIter != joint_msg->name.end(); ++origIter) {
    //std::cout << *origIter << std::endl;
    std::string fullOrigName = robot_name + "__" + *origIter;
    std::vector<std::string> targetNames;
    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator jointIter = model.joints_.begin(); jointIter != model.joints_.end(); ++jointIter) {
      //std::cout << jointIter->first << std::endl;
      if (ahb::string::endswith(jointIter->first, fullOrigName)) {
        targetNames.push_back(jointIter->first);
      }
    }
    if (targetNames.size() != 1) {
      ROS_ERROR("Unable to rename joint %s. Aborting.", fullOrigName.c_str());
      return;
    }

    m_renamed_curr_joints.push_back(targetNames[0]);
  }
  ROS_INFO_STREAM("renamed joints: " << ahb::string::toString(m_renamed_curr_joints));

  m_rename_joints = true;
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
