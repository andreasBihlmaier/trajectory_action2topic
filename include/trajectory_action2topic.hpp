#ifndef _TRAJECTORY_ACTION2TOPIC_H_
#define _TRAJECTORY_ACTION2TOPIC_H_

// system includes

// library includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

// custom includes


// forward declarations



class TrajectoryAction2Topic
{
  public:
    // enums

    // typedefs
    typedef std::map<int,int> JointMapType;

    // const static member variables
 
    // static utility functions
    static std::vector<double> reorderJoints(JointMapType jointMap, std::vector<double> input);
    static JointMapType permutationMap(const std::vector<std::string>& targetOrder, const std::vector<std::string>& origOrder);


    // constructors
    TrajectoryAction2Topic(const std::string& actionName);

    // overwritten methods

    // methods
    void onGoal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void onJoint(const sensor_msgs::JointState::ConstPtr& joints);

    // variables


  protected:
    // methods

    // variables
    ros::NodeHandle m_nh;
    std::string m_actionName;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> m_actionServer;
    ros::Subscriber m_jointSub;
    ros::Publisher m_jointPub;

    control_msgs::FollowJointTrajectoryFeedback m_feedback;
    control_msgs::FollowJointTrajectoryResult m_result;
    sensor_msgs::JointState m_currJointState;
    sensor_msgs::JointState m_targetJointState;


  private:
    // methods

    // variables


};

#endif // _TRAJECTORY_ACTION2TOPIC_H_
