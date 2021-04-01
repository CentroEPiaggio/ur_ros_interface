#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen3/Eigen/Eigen>
#include <skew_symmetric.h>
#include <pseudo_inversion.h>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>

bool received_state = false;
std::vector<double> q0(6);
std::vector<double> homing_default{0 ,  -1.88,    -1.5708 ,  -2.89 ,  -1.5708   ,      -3.1414};
KDL::JntArray  q_;            // Joint positions
Eigen::VectorXd q_eig_;
Eigen::Vector3d pos_d_;
Eigen::Quaterniond quat_d_;
	ros::Publisher pub_current_pose;
    
    geometry_msgs::Pose hand_pose;

trajectory_msgs::JointTrajectory trajectory;

void state_callback(const sensor_msgs::JointState& msg)
{
	if(received_state) return;
    
    // for(int i = 0;i<6;i++)
		q_(0) = (msg.position.at(2));
 		q_(1) = (msg.position.at(1));
		q_(2) = (msg.position.at(0));
		q_(3) = (msg.position.at(3));
		q_(4) = (msg.position.at(4));
		q_(5) = (msg.position.at(5));

		q_eig_(0) = q_(0);
 		q_eig_(1) = q_(1);
		q_eig_(2) = q_(2);
		q_eig_(3) = q_(3);
		q_eig_(4) = q_(4);
		q_eig_(5) = q_(5);

	received_state=true;
		// std::cout<<"Enter"<<std::endl;

}

void callback_des(const geometry_msgs::Pose& msg)
{
	pos_d_(0) = msg.position.x;
	pos_d_(1) = msg.position.y;
	pos_d_(2) = msg.position.z;

	quat_d_.w() = msg.orientation.w;
	quat_d_.x() = msg.orientation.x;
	quat_d_.y() = msg.orientation.y;
	quat_d_.z() = msg.orientation.z;	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_inv_kin");
	ros::NodeHandle n_;
	double rateHZ = 100;
	double dt = 1/rateHZ;

	ros::Subscriber sub = n_.subscribe("/joint_states",1,&state_callback);
	ros::Subscriber sub_pose_d = n_.subscribe("/pose_des",1,&callback_des);

	ros::Publisher pub_command_homing = n_.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
	pub_current_pose = n_.advertise<geometry_msgs::Pose>("/hand_pose", 1);
	sensor_msgs::JointState joint_command;
	joint_command.position.resize(6);
	ros::Rate r(rateHZ);

	bool first_quat_ = true;

	KDL::Tree robot_kin;
   	std::string robot_desc_string;
   	n_.param("robot_description", robot_desc_string, std::string());
   	if (!kdl_parser::treeFromString(robot_desc_string, robot_kin)){
      ROS_ERROR("Failed to parse kdl tree");
      return false;
   }	

   KDL::Chain kdl_chain_;
	if(!robot_kin.getChain("base", "hand", kdl_chain_)) // Check ee frame 
	{
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

	Eigen::Vector3d pos_, e1_, e2_;
	Eigen::Matrix3d orient_, orient_d_;
	Eigen::Quaterniond quat_, quat_old_;
	Eigen::VectorXd e_;
	Eigen::VectorXd old_pos_;
	Eigen::MatrixXd Jac_, Jac_pinv_, k_, Jac_W_;
	Eigen::VectorXd q0_, qdot_, defl_;                                                                                                                             
	KDL::Jacobian  J_;            // Jacobian
	KDL::Frame     x_;            // Tip pose                                                                                                                                       
	KDL::Frame     xd_;           // Tip desired pose                                                                                                                               
	KDL::Frame     x0_;           // Tip initial pose 
	Eigen::Matrix3d skew_;
	KDL::Vector quat_d_vec;

	// Resize (pre-allocate) the variables in non-realtime.                                                                                                                         
	q_.resize(kdl_chain_.getNrOfJoints());
	J_.resize(kdl_chain_.getNrOfJoints());
	KDL::SetToZero(q_);
	pos_.Zero();
	// pos_d_ << -0.348, 0.0 ,0.084;
	pos_d_ << 0, 0.0 ,0.0;
	quat_d_.w() = 1.0;
	quat_d_.vec() << 0.0, 0.0, 0.0;
	e1_.Zero();
	e2_.Zero();
	orient_.Zero();
	e_.resize(6);
	Jac_.resize(6, kdl_chain_.getNrOfJoints());
	Jac_pinv_.resize(kdl_chain_.getNrOfJoints(), 6);
	k_ = Eigen::Matrix<double, 6, 6>::Identity() * 8.0;
	qdot_.resize(kdl_chain_.getNrOfJoints());
	q_eig_.resize(kdl_chain_.getNrOfJoints());
	q_eig_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
	q0_.resize(kdl_chain_.getNrOfJoints());

	tf::TransformListener listener;

    tf::StampedTransform transform;
    try{
      listener.lookupTransform("base", "hand",  
                               ros::Time(0), transform); // check ee frame
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    hand_pose.position.x = transform.getOrigin().x();
	hand_pose.position.y = transform.getOrigin().y();
	hand_pose.position.z = transform.getOrigin().z();
	hand_pose.orientation.x = transform.getRotation().x();
	hand_pose.orientation.y = transform.getRotation().y();
	hand_pose.orientation.z = transform.getRotation().z();
	hand_pose.orientation.w = transform.getRotation().w();
    pub_current_pose.publish(hand_pose);


	bool active=false;
    
	while(!received_state && ros::ok())
	{
		std::cout<< "Waiting State"<< std::endl;
		r.sleep();
		ros::spinOnce();
	}

	jnt_to_pose_solver_->JntToCart(q_, x_);
	//converto posizione, matrice di rot e jac da kdl in Eigen
	for(int i = 0; i < 3 ; i++)
	{
		pos_d_(i) = x_.p(i);
	}


	for(int i = 0; i < 3 ; i++)
	{
		for(int j = 0; j < 3 ; j++)
		{
			orient_d_(i, j) = x_.M(i, j);

		}
		
	}
	quat_d_ = orient_d_;
	double alpha=0.0;


    // pub_current_pose.publish(hand_pose);
	while(ros::ok())
	{
		// pub_curr_pose();
		// // Compute the forward kinematics and Jacobian (at this location).
		jnt_to_pose_solver_->JntToCart(q_, x_);
		jnt_to_jac_solver_->JntToJac(q_, J_);


		//converto posizione, matrice di rot e jac da kdl in Eigen
		for(int i = 0; i < 3 ; i++)
		{
			pos_(i) = x_.p(i);
		}

		// std::cout<<"position: "<< pos_(0)<<" "<< pos_(1)<<" "<<pos_(2)<< std::endl;


		for(int i = 0; i < 3 ; i++)
		{
			for(int j = 0; j < 3 ; j++)
			{
				orient_(i, j) = x_.M(i, j);

			}
			
		}

		for(int i = 0; i < 6 ; i++)
		{
			for(int j = 0; j < kdl_chain_.getNrOfJoints() ; j++)
			{
				Jac_(i, j) = J_(i, j);

			}
			
		}

		//da matrice di rot a quat
		quat_ = orient_;
		quat_.normalize();

		// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
		if(first_quat_)
		{
			first_quat_= false;
			quat_old_ = quat_;
		} 

		double sign_check = quat_.w() * quat_old_.w() + quat_.x() * quat_old_.x() + quat_.y() * quat_old_.y() + quat_.z() * quat_old_.z();
		if(sign_check < 0.0)
		{
			quat_.w() = quat_.w() * (-1); 
			quat_.vec() = quat_.vec() * (-1); 
		}

		quat_old_ = quat_;

		double sign_check_des = quat_.w() * quat_d_.w() + quat_.x() * quat_d_.x() + quat_.y() * quat_d_.y() + quat_.z() * quat_d_.z();
		if(sign_check_des < 0.0)
		{
			quat_d_.w() = quat_d_.w() * (-1); 
			quat_d_.vec() = quat_d_.vec() * (-1); 
		}



		
		quat_d_vec(0) = quat_d_.x();
		quat_d_vec(1) = quat_d_.y();
		quat_d_vec(2) = quat_d_.z();

		
		skew_symmetric(quat_d_vec, skew_);
		e1_ = pos_d_ - pos_;
		e2_ = (quat_.w() * quat_d_.vec()) - (quat_d_.w() * quat_.vec()) - (skew_ * quat_.vec()); 

		e_ << e1_, e2_;

		// std::cout << e_ << "\n" << std::endl;


		if(e_.norm() > 0.01)
		// if(true)
		{
			pseudo_inverse(Jac_ , Jac_pinv_,true);
		
			qdot_ = Jac_pinv_ * (k_ * e_);

			q_eig_ += qdot_ * dt;

		}
		std_msgs::Float64MultiArray command_array;
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		for(int i=0; i<6; i++)
		{
			command_array.data.push_back(q_eig_(i));
			q_(i) = q_eig_(i);
		}

		joint_command.header.stamp = ros::Time::now();

		pub_command_homing.publish(command_array);

		    try{
      listener.lookupTransform("base", "hand",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    hand_pose.position.x = transform.getOrigin().x();
	hand_pose.position.y = transform.getOrigin().y();
	hand_pose.position.z = transform.getOrigin().z();
	hand_pose.orientation.x = transform.getRotation().x();
	hand_pose.orientation.y = transform.getRotation().y();
	hand_pose.orientation.z = transform.getRotation().z();
	hand_pose.orientation.w = transform.getRotation().w();
    pub_current_pose.publish(hand_pose);

		r.sleep();
		ros::spinOnce();

	}

	return 0;
}
