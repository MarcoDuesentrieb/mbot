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


float map(float x, float in_min, float in_max, float out_min, float out_max) {
    if(in_max == in_min) return (out_min + out_max) / 2.0;
    else return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


class ControllerNode
{
public:
    ControllerNode()
    {
        ROS_INFO("mbot_controller_node started");

        // get ROS parameters
        nh.param<float>("ultrasonic_safety_distance", ultrasonic_safety_distance, 0.10);
        nh.param<double>("rate", rate, 20.0); // Default rate: 10 Hz
        // get emg related parameters
        nh.param<double>("emg_timeout", emg_timeout, 0.2);
        nh.param<float>("v_emg01_min", emg01_min, 0.0);
        nh.param<float>("v_emg01_max", emg01_max, 3.3);
        nh.param<float>("v_emg02_min", emg02_min, 0.0);
        nh.param<float>("v_emg02_max", emg02_max, 3.3);

        // ROS publishers
        motor_pub = nh.advertise<mbot_msgs::Motor>("/mbot/motor", 10);

        // ROS subscribers
        ultrasonic_sub = nh.subscribe("/mbot/ultrasonic", 10, &ControllerNode::ultrasonic_callback, this);
        emg_sub = nh.subscribe("/mbot/emg", 10, &ControllerNode::emg_callback, this);
    }

    void run() // main loop
    {
        while(ros::ok())
        {
            ros::Rate loop_rate(rate);
            generateMotorCommand();
            motor_pub.publish(motor_msg);
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO("Controller node running");
        }
    }
private:
    ros::NodeHandle nh;
    ros::Publisher motor_pub;
    ros::Subscriber ultrasonic_sub;
    ros::Subscriber emg_sub;
    ros::Time last_emg_update;
    mbot_msgs::Motor motor_msg;
    double rate = 20.0;
    double emg_timeout;
    float ultrasonic_safety_distance;
    float ultrasonic_distance = 0.f;
    float emg01 = 0.f;
    float emg02 = 0.f;
    float emg01_min, emg01_max;
    float emg02_min, emg02_max;

    void generateMotorCommand()
    {
        if((ros::Time::now() - last_emg_update).toSec() < emg_timeout)
        {
            // Constrain emg between min and max
            emg01 = std::clamp(emg01, emg01_min, emg01_max);
            emg02 = std::clamp(emg02, emg02_min, emg02_max);

            // Normalize 
            float l = map(emg01, emg01_min, emg01_max, 0.0, 1.0);
            float r = map(emg02, emg02_min, emg02_max, 0.0, 1.0);

            // Calculate left and right motor speeds based on translational and rotational speed
            int cmd_l = static_cast<int>(round(l * 255));
            int cmd_r = static_cast<int>(round(r * 255));

            if(ultrasonic_distance < ultrasonic_safety_distance)
            {
                // only allow moving backwards since there is an obstacle in front of the robot
                if(cmd_l > 0 || cmd_r > 0)
                {
                    cmd_l = 0;
                    cmd_r = 0;
                }
            }

            motor_msg.left = cmd_l;
            motor_msg.right = cmd_r;
        }
        else
        {
            // No recent EMG sensor data available -> stop the robot
            motor_msg.left = 0;
            motor_msg.right = 0;
        }

    }

    void ultrasonic_callback(std_msgs::Float32 msg)
    {
        ultrasonic_distance = msg.data;
    }

    void emg_callback(mbot_msgs::EMG msg)
    {
        emg01 = msg.ch1;
        emg02 = msg.ch2;
        last_emg_update = ros::Time::now();
    }
};





int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbot_controller_node");
    ControllerNode controller_node;
    controller_node.run();

    return 0;
}