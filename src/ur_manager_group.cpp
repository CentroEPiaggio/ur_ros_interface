#include "ur_interface_pkg/ur_manager_group.h"

URManagerGroup::URManagerGroup()
{
    
    nh.getParam("positon_controller_topic",positon_controller_topic);
    sub_ur_aux = nh.subscribe("/ur_command",1,&URManagerGroup::callback_ur_aux,this);
  

    state_subscriber = nh.subscribe("/joint_states",1,&URManagerGroup::callback_state,this);

    command_publisher = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
    command_array.data.resize(6);
}


void URManagerGroup::callback_ur_aux(const trajectory_msgs::JointTrajectory& msg)
{
    for(int i=0;i<6;i++)
        command_array.data.at(i) = msg.points.at(0).positions.at(i);  

    command_publisher.publish(command_array);
}



void URManagerGroup::callback_state(const sensor_msgs::JointState& msg)
{
    if(active) return;

    command_array.data.at(0) = msg.position.at(2);
    command_array.data.at(1) = msg.position.at(1);
    command_array.data.at(2) = msg.position.at(0);
    command_array.data.at(3) = msg.position.at(3);
    command_array.data.at(4) = msg.position.at(4);
    command_array.data.at(5) = msg.position.at(5);
    command_publisher.publish(command_array);
 
    active = true;
}

void URManagerGroup::run()
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

URManagerGroup::~URManagerGroup()
{

}