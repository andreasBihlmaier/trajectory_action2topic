#include "trajectory_action2topic.hpp"

// system includes

// library includes

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
    jointMap[nameIdx] = foundIt - targetOrder.begin();
  }

  return jointMap;
}

TrajectoryAction2Topic::TrajectoryAction2Topic(const std::string& actionName)
  :m_actionName(actionName),
   m_actionServer(m_nh, actionName, boost::bind(&TrajectoryAction2Topic::onGoal, this, _1), false)
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
    jointMap = permutationMap(m_currJointState.name, goal->trajectory.joint_names);
    if (jointMap.size() != m_currJointState.name.size()) {
      m_result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      m_actionServer.setSucceeded(m_result);
      return;
    }
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
    ROS_INFO_STREAM("sinceStart=" << sinceStart << " will sleeping for " << sleepDuration);
    sleepDuration.sleep();

    m_feedback.desired = currPoint;
    m_feedback.actual.positions = m_currJointState.position;
    m_feedback.actual.velocities = m_currJointState.velocity;
    sinceStart = ros::Time::now() - startTime;
    m_feedback.actual.time_from_start = sinceStart;
    //TODO m_feedback.error
    m_actionServer.publishFeedback(m_feedback);

    //TODO PATH_TOLERANCE_VIOLATED
  }

  if (success) {
    //TODO GOAL_TOLERANCE_VIOLATED
    m_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    m_actionServer.setSucceeded(m_result);
  }
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
