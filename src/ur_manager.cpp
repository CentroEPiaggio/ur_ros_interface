#include "ur_interface_pkg/ur_manager.h"

URManager::URManager()
{
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.positions.push_back(0.0);
    point.time_from_start = ros::Duration(0.01);
    active = false;
    trajectory.joint_names.push_back("shoulder_pan_joint");
    trajectory.joint_names.push_back("shoulder_lift_joint");
    trajectory.joint_names.push_back("elbow_joint");
    trajectory.joint_names.push_back("wrist_1_joint");
    trajectory.joint_names.push_back("wrist_2_joint");
    trajectory.joint_names.push_back("wrist_3_joint");

    trajectory.points.push_back(point);
    
    nh.getParam("positon_controller_topic",positon_controller_topic);
    sub_ur_aux = nh.subscribe("/ur_command",1,&URManager::callback_ur_aux,this);
  

    state_subscriber = nh.subscribe("/joint_states",1,&URManager::callback_state,this);

    command_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(positon_controller_topic.c_str(), 1);

}


void URManager::callback_ur_aux(const trajectory_msgs::JointTrajectory& msg)
{
    trajectory.points.at(0).positions = msg.points.at(0).positions;  
    std_msgs::Header empty_header;
    //trajectory.header.stamp = ros::Time::now();
    trajectory.header = empty_header;	
    command_publisher.publish(trajectory);
}



void URManager::callback_state(const sensor_msgs::JointState& msg)
{
    if(active) return;

    trajectory.points.at(0).positions.at(0) = msg.position.at(2);
    trajectory.points.at(0).positions.at(1) = msg.position.at(1);
    trajectory.points.at(0).positions.at(2) = msg.position.at(0);
    trajectory.points.at(0).positions.at(3) = msg.position.at(3);
    trajectory.points.at(0).positions.at(4) = msg.position.at(4);
    trajectory.points.at(0).positions.at(5) = msg.position.at(5);
    std_msgs::Header empty_header;
    //trajectory.header.stamp = ros::Time::now();
    trajectory.header = empty_header;	
    command_publisher.publish(trajectory);
 
    active = true;
}

void URManager::run()
{
    ros::Rate r(100);



    while(!active && ros::ok()) 
    {
        ROS_INFO("Waiting for UR arm ... ");
        ros::spinOnce();
        r.sleep();

    } 
    while(ros::ok())
    {
	    r.sleep();
	    ros::spinOnce();
    }

}

URManager::~URManager()
{

}
