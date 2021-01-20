#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

class URManagerGroup
{
public:
    URManagerGroup();
    ~URManagerGroup();
    void run();

private:

    void callback_state(const sensor_msgs::JointState& msg);
    void callback_ur_aux(const trajectory_msgs::JointTrajectory& msg);
    bool active;
    ros::NodeHandle nh;
    trajectory_msgs::JointTrajectory trajectory;
    ros::Publisher command_publisher;
    ros::Subscriber command_subscriber, state_subscriber,  sub_ur_aux;
    std::string positon_controller_topic;
    std_msgs::Float64MultiArray command_array;

};
