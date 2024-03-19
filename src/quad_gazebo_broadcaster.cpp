#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "gazebo_msgs/ModelState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <quad_control/linear_message.h>
#include <quad_control/angular_message.h>
#include <quad_control/trajectory_message.h>
#include <quad_control/uav_message.h>

float theta_p = 0;
float phi_p = 0;
float psi_p = 0;
float theta = 0;
float phi = 0;
float psi = 0;
float x = 0;
float y = 0;
float z = 0;
float x_p = 0;
float y_p = 0;
float z_p = 0;

void uavMessageCallback(const quad_control::uav_message::ConstPtr& uav_msg)
{
    theta_p = uav_msg->theta_p;
    phi_p = uav_msg->phi_p;
    psi_p = uav_msg->psi_p;
	x_p = uav_msg->x_p;
    y_p = uav_msg->y_p;
    z_p = uav_msg->z_p;
    x = uav_msg->x;
    y = uav_msg->y;
    z = uav_msg->z;
    theta = uav_msg->theta;
    phi = uav_msg->phi;
    psi = uav_msg->psi;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "quad_gazebo_broadcaster");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber pos = nh.subscribe("uav_message",100, &uavMessageCallback);
	
	gazebo_msgs::ModelState states;
	ros::Publisher gazebo_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",100);

	states.model_name = "F450";
	tf2::Quaternion myQuaternion;
	tf2::Quaternion q;
	tf2::Quaternion q_rot;
	
	while(ros::ok())
	{
		
		myQuaternion.setRPY(phi,theta,psi);
		q_rot.setRPY(3.141592,0,0);
		q = q_rot * myQuaternion;
		q.normalize();
		
		states.pose.position.x = x;
		states.pose.position.y = -y;
		states.pose.position.z = -z;
		
		states.pose.orientation.x = q.x();
		states.pose.orientation.y = q.y();
		states.pose.orientation.z = q.z();
		states.pose.orientation.w = q.w();
		
		gazebo_pub.publish(states);
		
		ros::spinOnce();
		loop_rate.sleep();
		
		
	}
	
	
}
