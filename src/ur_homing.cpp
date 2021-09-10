#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>

bool received_state = false;
std::vector<double> q0(6);
std::vector<double> homing_default{0 ,  -1.88,    -1.5708 ,  -2.89 ,  -1.5708   ,      -3.1414};
trajectory_msgs::JointTrajectory trajectory;

void state_callback(const sensor_msgs::JointState& msg)
{
	if(received_state) return;
    
    // for(int i = 0;i<6;i++)
		q0.at(0) = (msg.position.at(2));
 		q0.at(1) = (msg.position.at(1));
		q0.at(2) = (msg.position.at(0));
		q0.at(3) = (msg.position.at(3));
		q0.at(4) = (msg.position.at(4));
		q0.at(5) = (msg.position.at(5));

	received_state=true;
		// std::cout<<"Enter"<<std::endl;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "single_lwr_homing");
	ros::NodeHandle n_;
	double rateHZ = 50;

	std::vector<double> homing(6);
    
    n_.param("homing",homing,homing_default);
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);

    trajectory.joint_names.push_back("shoulder_pan_joint");
    trajectory.joint_names.push_back("shoulder_lift_joint");
    trajectory.joint_names.push_back("elbow_joint");
    trajectory.joint_names.push_back("wrist_1_joint");
    trajectory.joint_names.push_back("wrist_2_joint");
    trajectory.joint_names.push_back("wrist_3_joint");

    trajectory.points.push_back(point);

	ros::Subscriber sub = n_.subscribe("/joint_states",1,&state_callback);

	ros::Publisher pub_command_homing = n_.advertise<trajectory_msgs::JointTrajectory>("/ur_command", 1);

	ros::Rate r(rateHZ);

	bool active=false;

	while(!received_state && ros::ok())
	{
		r.sleep();
		ros::spinOnce();
	}

	double alpha=0.0;

	while(ros::ok())
	{
		for(int i=0; i<6; i++) {
		trajectory.points.at(0).positions.at(i) = alpha*homing.at(i) + (1-alpha)*q0.at(i);
		}

		pub_command_homing.publish(trajectory);

		r.sleep();
		ros::spinOnce();

		alpha+=0.0025;
		if(alpha>1) break;
	}

	std::cout<<"Homing: DONE"<<std::endl;

	return 0;
}
