#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <quad_control/linear_message.h>
#include <quad_control/angular_message.h>
#include <quad_control/trajectory_message.h>
#include <quad_control/uav_message.h>

float desx = 0;     //mando a linear_control
float desy = 0;     //mando a linear_control
float desz = 0;     //mando a linear_control
float despsi = 0;   //mando a linear_control
float desx_p = 0;   //mando a linear_control
float desy_p = 0;   //mando a linear_control
float desz_p = 0;   //mando a linear_control
float despsi_p = 0; //mando a linear_control
float t = 0;        //tiempo

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_control");
    ros::NodeHandle nh;

    ros::Publisher trajectory_message_pub = nh.advertise<quad_control::trajectory_message>("trajectory_message", 100);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        quad_control::trajectory_message trajectory_message_msg;

        if (t >= 0 && t < 5){
            desx_p = 0;
            desy_p = 0;
            desz_p = -0.5;
            despsi_p = 0;
        }
        else if (t >= 5 && t <= 65){
            desx_p = 0.5*sin(0.1*(t-5));
            desy_p = 0.5*cos(0.1*(t-5));
            desz_p = 0;
            despsi_p = 0.1;
        }

        desx = desx + 0.01*desx_p;
        desy = desy + 0.01*desy_p;
        desz = desz + 0.01*desz_p;
        despsi = despsi + 0.01*despsi_p;

        t = t + 0.01;

        trajectory_message_msg.desx = desx;
        trajectory_message_msg.desy = desy;
        trajectory_message_msg.desz = desz;
        trajectory_message_msg.desx_p = desx_p;
        trajectory_message_msg.desy_p = desy_p;
        trajectory_message_msg.desz_p = desz_p;
        trajectory_message_msg.despsi = despsi;
        trajectory_message_msg.despsi_p = despsi_p;

        trajectory_message_pub.publish(trajectory_message_msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
