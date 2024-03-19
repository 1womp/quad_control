#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <quad_control/linear_message.h>
#include <quad_control/angular_message.h>
#include <quad_control/trajectory_message.h>
#include <quad_control/uav_message.h>

float x_p = 0;         //recibo de uav_control
float y_p = 0;         //recibo de uav_control
float z_p = 0;         //recibo de uav_control
float x = -5;          //recibo de uav_control
float y = 0;           //recibo de uav_control
float z = 0;           //recibo de uav_control
float theta = 0;       //recibo de uav_control
float phi = 0;         //recibo de uav_control
float psi = 0;         //recibo de uav_control
float desx = 0;        //recibo de trajectory_control
float desy = 0;        //recibo de trajectory_control
float desz = 0;        //recibo de trajectory_control
float desx_p = 0;      //recibo de trajectory_control
float desy_p = 0;      //recibo de trajectory_control
float desz_p = 0;      //recibo de trajectory_control
float thrust = 0;      //mando a uav_control
float destheta = 0;    //mando a angular_control
float desphi = 0;      //mando a angular_control
float despsi = 0;      //mando a angular_control (recivo de trayectory_control)
float destheta_p = 0;  //mando a angular_control (constante)
float desphi_p = 0;    //mando a angular_control (constante)
float despsi_p = 0;    //mando a angular_control (recibo de trajectory_control)
float desz_pp = 0;     //(permanece en zero)

float uvx = 0;         //calculo
float uvy = 0;         //calculo
float uvz = 0;         //calculo

float error_x = 0;     //calculo
float error_x_p = 0;   //calculo
float error_y = 0;     //calculo
float error_y_p = 0;   //calculo
float error_z = 0;     //calculo
float error_z_p = 0;   //calculo

float kpuvx = 0.1;    //constante
float kduvx = 0.8;     //constante
float kpuvy = 0.1;    //constante
float kduvy = 0.8;     //constante
float kpuvz = 0.1;    //constante
float kduvz = 1.2;     //constante

float mass = 2;        //constante
float g = 9.81;        //constante

void trajectoryMessageCallback(const quad_control::trajectory_message::ConstPtr& trajectory_msg)
{
    desx = trajectory_msg->desx;
    desy = trajectory_msg->desy;
    desz = trajectory_msg->desz;
    desx_p = trajectory_msg->desx_p;
    desy_p = trajectory_msg->desy_p;
    desz_p = trajectory_msg->desz_p;
    despsi = trajectory_msg->despsi;
}

void uavMessageCallback(const quad_control::uav_message::ConstPtr& uav_msg)
{
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_control");
    ros::NodeHandle nh;

    ros::Publisher linear_message_pub = nh.advertise<quad_control::linear_message>("linear_message", 100);

    ros::Subscriber trajectory_sub = nh.subscribe("trajectory_message", 100, trajectoryMessageCallback);
    ros::Subscriber uav_sub = nh.subscribe("uav_message", 100, uavMessageCallback);

    ros::Rate loop_rate(100);


    while (ros::ok())
    {
        quad_control::linear_message linear_message_msg;

        error_x = x - desx;
        error_x_p = x_p - desx_p;

        error_y = y - desy;
        error_y_p = y_p - desy_p;

        error_z = z - desz;
        error_z_p = z_p - desz_p;

        uvx = -kpuvx*error_x - kduvx*error_x_p;
        uvy = -kpuvy*error_y - kduvy*error_y_p;
        uvz = -kpuvz*error_z - kduvz*error_z_p;

        thrust = ((mass/(cos(phi)*cos(theta)))*(desz_pp - g + uvz));

        desphi = asin((mass/thrust)*(sin(despsi)*uvx - cos(despsi)*uvy));

        destheta = asin(((mass/thrust)*uvx - sin(despsi)*sin(desphi))/(cos(despsi)*cos(desphi)));

        linear_message_msg.thrust = thrust;
        linear_message_msg.destheta = destheta;
        linear_message_msg.destheta_p = destheta_p;
        linear_message_msg.desphi = desphi;
        linear_message_msg.desphi_p = desphi_p;

        linear_message_pub.publish(linear_message_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
