#ifndef _TRAJECTORY_ACTION2TOPIC_H_
#define _TRAJECTORY_ACTION2TOPIC_H_

// system includes

// library includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// custom includes


// forward declarations



class TrajectoryAction2Topic
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    TrajectoryAction2Topic(const std::string& actionName);

    // overwritten methods

    // methods
    void onGoal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

    // variables


  protected:
    // methods

    // variables
    ros::NodeHandle m_nh;
    std::string m_actionName;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> m_actionServer;

    control_msgs::FollowJointTrajectoryFeedback m_feedback;
    control_msgs::FollowJointTrajectoryResult m_result;


  private:
    // methods

    // variables


};

#endif // _TRAJECTORY_ACTION2TOPIC_H_
