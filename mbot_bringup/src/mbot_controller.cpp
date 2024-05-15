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


class ControllerNode
{
public:
    ControllerNode()
    {
        ROS_INFO("mbot_controller_node started");

        // Get ROS parameters
        nh.param<float>("ultrasonic_safety_distance", ultrasonic_safety_distance, 0.10);
        nh.param<double>("rate", rate, 20.0); // Default rate: 10 Hz
        
        // TODO: Get EMG related parameters


        // TODO: Add ROS publisher for the motor command
        

        // TODO: Add ROS subscriber for the EMG signals


        ultrasonic_sub = nh.subscribe("/mbot/ultrasonic", 10, &ControllerNode::ultrasonic_callback, this);
    }

    void run() // main loop
    {
        while(ros::ok())
        {
            generateMotorCommand();
            // TODO: Process publish motor command


            ros::Rate loop_rate(rate);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber ultrasonic_sub;
    double rate;
    float ultrasonic_safety_distance; // distance at which the robot might stop to prevent collision
    float ultrasonic_distance = 0.f;  // mbot ultrasonic sensor data
    float emg01 = 0.f;  // EMG sensor 1 voltage 
    float emg02 = 0.f;  // EMG sensor 2 voltage 
    float emg01_min, emg01_max; // EMG sensor 1 voltage lower and upper limit
    float emg02_min, emg02_max;  // // EMG sensor 1 voltage lower and upper limit
    
    // TODO: Add member variables for ROS publishers, subscribers or other data if needed


    void generateMotorCommand()
    {
        // TODO: Implement control logic


    }

    // TODO: Add subscriber callbacks


    void ultrasonic_callback(std_msgs::Float32 msg)
    {
        // OPTIONAL: Make use of the ultrasonic sensor
        ultrasonic_distance = msg.data;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbot_controller_node");
    ControllerNode controller_node;
    controller_node.run();

    return 0;
}