#include "trajectory_action2topic.hpp"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TrajectoryAction2Topic::TrajectoryAction2Topic(const std::string& actionName)
  :m_actionName(actionName),
   m_actionServer(m_nh, actionName, boost::bind(&TrajectoryAction2Topic::onGoal, this, _1), false)
{
  m_actionServer.start();
}

void
TrajectoryAction2Topic::onGoal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  ROS_INFO_STREAM("onGoal():\n" << *goal << "---");

  // TODO
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
