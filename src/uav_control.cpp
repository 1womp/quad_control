#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <quad_control/linear_message.h>
#include <quad_control/angular_message.h>
#include <quad_control/trajectory_message.h>
#include <quad_control/uav_message.h>

float x_p = 0;          //necesito mandar
float y_p = 0;          //necesito mandar
float z_p = 0;          //necesito mandar
float x = 0;            //necesito mandar
float y = 0;            //necesito mandar
float z = 0;            //necesito mandar
float theta_p = 0;      //necesito mandar
float phi_p = 0;        //necesito mandar
float psi_p = 0;        //necesito mandar
float theta = 0;        //necesito mandar
float phi = 0;          //necesito mandar
float psi = 0;          //necesito mandar
float thrust = 0;       //me llega de linear_control
float torque_theta = 0; //me llega de angular_control
float torque_phi = 0;   //me llega de angular_control
float torque_psi = 0;   //me llega de angular_control
float Jxx = 0.0411;     //constante
float Jyy = 0.0478;     //constante
float Jzz = 0.0599;     //constante
float mass = 2;         //constante
float g = 9.81;         //constante

Eigen::Vector3f p_p;
Eigen::Vector3f v;
Eigen::Vector3f p;
Eigen::Vector3f v_p;
Eigen::Vector3f att;
Eigen::Vector3f w_p;
Eigen::Vector3f att_p;
Eigen::Vector3f w_pp;
Eigen::Vector3f torque;
Eigen::Vector3f f(0,0,0);
Eigen::Vector3f e3(0,0,1);
Eigen::Matrix3f J;

Eigen::Matrix3f R(){
    Eigen::Matrix3f r;
    r << cos(att(2))*cos(att(1)), cos(att(2))*sin(att(0))*sin(att(1)) - cos(att(0))*sin(att(2)), sin(att(2))*sin(att(0)) + cos(att(2))*cos(att(0))*sin(att(1)),
        cos(att(1))*sin(att(2)), cos(att(2))*cos(att(0)) + sin(att(2))*sin(att(0))*sin(att(1)), cos(att(0))*sin(att(2))*sin(att(1)) - cos(att(2))*sin(att(0)),
        -sin(att(1)), cos(att(1))*sin(att(0)), cos(att(0))*cos(att(1));
    return r;
}

Eigen::Matrix3f R2(){
    Eigen::Matrix3f r2;
    r2 <<1, (sin(att(0))*tan(att(1))), (cos(att(0))*tan(att(1))),
            0, cos(att(0)), -sin(att(0)),
		    0, (sin(att(0))/cos(att(1))), (cos(att(0))/cos(att(1)));
    return r2;
}

void linearMessageCallback(const quad_control::linear_message::ConstPtr& linear_msg)
{
    thrust = linear_msg->thrust;
}

void angularMessageCallback(const quad_control::angular_message::ConstPtr& angular_msg)
{
    torque(0) = angular_msg->torque_phi;
    torque(1) = angular_msg->torque_theta;
    torque(2) = angular_msg->torque_psi;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control");
    ros::NodeHandle nh;

    ros::Publisher uav_message_pub = nh.advertise<quad_control::uav_message>("uav_message", 100);

    ros::Subscriber linear_sub = nh.subscribe("linear_message", 100, linearMessageCallback);
    ros::Subscriber angular_sub = nh.subscribe("angular_message", 100, angularMessageCallback);

    ros::Rate loop_rate(100);
    
    J <<Jxx, 0, 0,
		0, Jyy, 0,
		0, 0, Jzz;
	
	p << 0,0,0;
    p_p << 0,0,0;
	att << 0,0,0;
    w_p << 0,0,0;
    w_pp << 0,0,0;
    att_p << 0,0,0;
    v << 0,0,0;
    v_p << 0,0,0;

    while (ros::ok())
    {
        quad_control::uav_message uav_message_msg;
        
        //angular

        w_pp = J.inverse() * (torque - (w_p.cross(J*w_p)));

        for(int i = 0; i <= 2; i++)
		{
			w_p(i) = w_p(i) + 0.01 * w_pp(i);
		}
        att_p = R2() * w_p;

        for(int i = 0; i <= 2; i++)
		{
			att(i) = att(i) + 0.01 * att_p(i);
		}

        // https://msl.cs.uiuc.edu/planning/node102.html

        //linear
        f = thrust*e3 + (R()).transpose()*(mass*g*e3);

        v_p = f/mass - w_p.cross(v);

		v(0) = v(0) + 0.01 * v_p(0);
        v(1) = v(1) + 0.01 * v_p(1);
        v(2) = v(2) + 0.01 * v_p(2);

        p_p = R() * v;

		p(0) = p(0) + 0.01 * p_p(0);
        p(1) = p(1) + 0.01 * p_p(1);
        p(2) = p(2) + 0.01 * p_p(2);

        uav_message_msg.x = p(0);
        uav_message_msg.y = p(1);
        uav_message_msg.z = p(2);
        uav_message_msg.x_p = p_p(0);
        uav_message_msg.y_p = p_p(1);
        uav_message_msg.z_p = p_p(2);
        uav_message_msg.phi = att(0);
        uav_message_msg.theta = att(1);
        uav_message_msg.psi = att(2);
        uav_message_msg.phi_p = att_p(0);
        uav_message_msg.theta_p = att_p(1);
        uav_message_msg.psi_p = att_p(2);

        uav_message_pub.publish(uav_message_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
