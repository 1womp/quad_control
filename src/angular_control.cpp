#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <quad_control/linear_message.h>
#include <quad_control/angular_message.h>
#include <quad_control/trajectory_message.h>
#include <quad_control/uav_message.h>

float theta = 0;        //recibo de uav_control
float phi = 0;          //recibo de uav_control
float psi = 0;          //recibo de uav_control
float theta_p = 0;      //recibo de uav_control
float phi_p = 0;        //recibo de uav_control
float psi_p = 0;        //recibo de uav_control
float destheta = 0;     //recibo de linear_control
float desphi = 0;       //recibo de linear_control
float despsi = 0;       //recibo de linear_control
float destheta_p = 0;   //recibo de linear_control
float desphi_p = 0;     //recibo de linear_control
float despsi_p = 0;     //recibo de linear_control
float torque_theta = 0; //mando a uav_control
float torque_phi = 0;   //mando a uav_control
float torque_psi = 0;   //mando a uav_control
float error_theta = 0;  //calculo
float error_phi = 0;    //calculo
float error_psi = 0;    //calculo
float error_theta_p = 0;//calculo
float error_phi_p = 0;  //calculo
float error_psi_p = 0;  //calculo
float theta_u = 0;      //calculo
float phi_u = 0;        //calculo
float psi_u = 0;        //calculo
float Jxx = 0.0411;     //constante
float Jyy = 0.0478;     //constante
float Jzz = 0.0599;     //constante

float kptheta = 2.1;    //constante
float kdtheta = 3.2;    //constante
float kpphi = 3.8;      //constante
float kdphi = 1.8;      //constante
float kppsi = 0.5;      //constante
float kdpsi = 1;        //constante

void uavMessageCallback(const quad_control::uav_message::ConstPtr& uav_msg)
{
    theta_p = uav_msg->theta_p;
    phi_p = uav_msg->phi_p;
    psi_p = uav_msg->psi_p;
    theta = uav_msg->theta;
    phi = uav_msg->phi;
    psi = uav_msg->psi;
}

void linearMessageCallback(const quad_control::linear_message::ConstPtr& linear_msg)
{
    destheta = linear_msg->destheta;
    destheta_p = linear_msg->destheta_p;
    desphi = linear_msg->desphi;
    desphi_p = linear_msg->desphi_p;
}

void trajectoryMessageCallback(const quad_control::trajectory_message::ConstPtr& trajectory_msg)
{
    despsi = trajectory_msg->despsi;
    despsi_p = trajectory_msg->despsi_p;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "angular_control");
    ros::NodeHandle nh;

    ros::Publisher angular_message_pub = nh.advertise<quad_control::angular_message>("angular_message", 100);

    ros::Subscriber linear_sub = nh.subscribe("linear_message", 100, linearMessageCallback);
    ros::Subscriber uav_sub = nh.subscribe("uav_message", 100, uavMessageCallback);
    ros::Subscriber trajectory_sub = nh.subscribe("trajectory_message", 100, trajectoryMessageCallback);

    ros::Rate loop_rate(100);


    while (ros::ok())
    {
        quad_control::angular_message angular_message_msg;

        error_theta = destheta - theta;
        error_phi = desphi - phi;
        error_psi = despsi - psi;

        error_theta_p = destheta_p - theta_p;
        error_phi_p = desphi_p - phi_p;
        error_psi_p = despsi_p - psi_p;

        theta_u = kptheta*error_theta + kdtheta*error_theta_p;
        phi_u = kpphi*error_phi + kdphi*error_phi_p;
        psi_u = kppsi*error_psi + kdpsi*error_theta_p;

        torque_phi = Jxx*(((Jzz-Jyy)/(Jxx))*theta_p*psi_p + phi_u);
        torque_theta = Jyy*(((Jxx-Jzz)/(Jyy))*phi_p*psi_p + theta_u);
        torque_psi = Jzz*(((Jyy-Jxx)/(Jzz))*phi_p*theta_p + psi_u);

        angular_message_msg.torque_theta = torque_theta;
        angular_message_msg.torque_phi = torque_phi;
        angular_message_msg.torque_psi = torque_psi;
        angular_message_msg.error_psi = error_psi;

        angular_message_pub.publish(angular_message_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
