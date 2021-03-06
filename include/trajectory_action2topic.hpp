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


template <typename T>
std::vector<T> operator-(const std::vector<T>& lhs, const std::vector<T>& rhs)
{
  std::vector<T> res;
  res.reserve(lhs.size());

  for (size_t idx = 0; idx < lhs.size(); idx++) {
    res.push_back(lhs[idx] - rhs[idx]);
  }

  return res;
}

template <typename T>
T plusabs(const T& lhs, const T& rhs)
{
  return std::abs(lhs) + std::abs(rhs);
}


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
    void rename_joints(const std::string& robot_name, const std::string& robot_description);

    // variables
    static const double m_max_position_error = 0.1;


  protected:
    // methods

    // variables
    ros::NodeHandle m_nh;
    std::string m_actionName;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> m_actionServer;
    ros::Subscriber m_jointSub;
    ros::Publisher m_jointPub;
    bool m_rename_joints;
    std::vector<std::string> m_renamed_curr_joints;

    control_msgs::FollowJointTrajectoryFeedback m_feedback;
    control_msgs::FollowJointTrajectoryResult m_result;
    sensor_msgs::JointState m_currJointState;
    sensor_msgs::JointState m_targetJointState;


  private:
    // methods

    // variables


};

#endif // _TRAJECTORY_ACTION2TOPIC_H_
