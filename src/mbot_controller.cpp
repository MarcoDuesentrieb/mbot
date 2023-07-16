//
// Created by Marco Dittmann on 16.07.23.
//

#include <iostream>
#include <sstream>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mbot_msgs/Motor.h>
#include <mbot_msgs/EMG.h>

float emg01 = 0;
float emg02 = 0;
ros::Time last_emg_update;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void emg_callback(mbot_msgs::EMG msg)
{
    emg01 = msg.ch1;
    emg02 = msg.ch2;
    last_emg_update = ros::Time::now();
}


int main(int argc, char** argv)
{

    //  ### INITIALISATION ###
    ros::init(argc, argv, "mbot_controller_node");
    ros::NodeHandle nh;

    // ROS publishers
    ros::Publisher motor_pub = nh.advertise<mbot_msgs::Motor>("/mbot/motor", 10);

    // ROS subscribers
    ros::Subscriber emg_sub = nh.subscribe("/mbot/emg", 10, emg_callback);

    // ROS messages
    mbot_msgs::Motor motor;

    // emg voltage
    float emg01_min, emg01_max;
    float emg02_min, emg02_max;
    nh.param<float>("v_emg01_min", emg01_min, 0.0);
    nh.param<float>("v_emg01_max", emg01_max, 3.3);
    nh.param<float>("v_emg02_min", emg02_min, 0.0);
    nh.param<float>("v_emg02_max", emg02_max, 3.3);

    ROS_INFO_STREAM("emg01_min: " << emg01_min);
    ROS_INFO_STREAM("emg01_max: " << emg01_max);
    ROS_INFO_STREAM("emg02_min: " << emg02_min);
    ROS_INFO_STREAM("emg02_max: " << emg02_max);


    // Read rate parameter
    double rate;
    nh.param<double>("rate", rate, 20.0); // Default rate: 10 Hz
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {

        if((ros::Time::now() - last_emg_update).sec < 0.2)
        {
            // generate motor command
            float t = map(emg01, emg01_min, emg01_max, -1.0, 1.0);
            float r = map(emg02, emg02_min, emg02_max, -1.0, 1.0);

            // Calculate left and right motor speeds based on translational and rotational speed
            int ml = (t - r) * 255;
            int mr = (t + r) * 255;
            int cmd_l = std::min(std::max(ml, -255), 255);
            int cmd_r = std::min(std::max(mr, -255), 255);
            motor.left = cmd_l;
            motor.right = cmd_r;
        }
        else
        {
            motor.left = 0;
            motor.right = 0;
        }


        motor_pub.publish(motor);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Cleanup

    return 0;
}