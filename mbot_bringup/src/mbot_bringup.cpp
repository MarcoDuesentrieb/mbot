#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <simpleble/SimpleBLE.h>
#include <cstring>


/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mbot_msgs/EMG.h>
#include <mbot_msgs/Motor.h>


void reverseBytes(std::string &s)
// for converting between Little-Endian and Big-Endian
{
    auto copy = s;
    size_t len = s.length();

    for(size_t i = 0; i < len; ++i)
        s[i] = copy[len-1-i];
}



int main(int argc, char** argv)
{
    //  ### INITIALISATION ###
    ros::init(argc, argv, "mbot_bringup_node");
    ros::NodeHandle nh;

    // Read peripheral_address parameter
    std::string peripheral_address;
    nh.param<std::string>("peripheral_address", peripheral_address, "");
    std::vector<std::pair<SimpleBLE::BluetoothUUID, SimpleBLE::BluetoothUUID>> uuids;
    SimpleBLE::Peripheral arduino;
    bool connected = false;
    bool found = false;

    // ROS publishers
    ros::Publisher emg_pub = nh.advertise<mbot_msgs::EMG>("/mbot/emg", 10);
    ros::Publisher motor_pub = nh.advertise<mbot_msgs::Motor>("/mbot/motor", 10);

    // ROS messages
    mbot_msgs::EMG emg;
    mbot_msgs::Motor motor;

    // Read rate parameter
    double rate;
    nh.param<double>("rate", rate, 20.0); // Default rate: 10 Hz
    ros::Rate loop_rate(rate);

    // Check if the systems bluetooth adapter is enabled
    if (!SimpleBLE::Adapter::bluetooth_enabled()) {
        ROS_ERROR("Bluetooth is not enabled");
        return 1;
    }

    // Get a list of all available adapters
    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty()) {
        ROS_ERROR("No Bluetooth adapters found");
        return 1;
    }

    // Use the first adapter
    auto adapter = adapters[0];

    // Print adapter info
    ROS_INFO_STREAM("Adapter identifier: " << adapter.identifier());
    ROS_INFO_STREAM("Adapter address: " << adapter.address());


    //  ### Adapter CALLBACK DEFINITIONS ###

    // Set the callback to be called when the scan starts
    adapter.set_callback_on_scan_start([&peripheral_address]()
    {
        ROS_INFO_STREAM("Scanning for peripherals... ");
        ROS_INFO_STREAM("Searching for: " << peripheral_address);
    });

    // Set the callback to be called when the scan stops
    adapter.set_callback_on_scan_stop([]()
    {
        ROS_INFO("Scan stopped");
    });

    // Set the callback to be called when the scan finds a new peripheral
    adapter.set_callback_on_scan_found([&adapter, &arduino, &peripheral_address, &found](SimpleBLE::Peripheral peripheral)
    {
//        std::cout << "Peripheral found: " << peripheral.identifier() << std::endl;
//        std::cout << "Peripheral address: " << peripheral.address() << std::endl;
//        ROS_INFO_STREAM("Found peripheral: " << peripheral.address());
        if(peripheral.address() == peripheral_address)
        {
            ROS_INFO("Desired peripheral found!");
            adapter.scan_stop();
            arduino = peripheral;
            found = true;
        }
    });

    //  ### START ###

    // Start scanning for peripherals
    auto start = std::chrono::system_clock::now();
    adapter.scan_start();


    // Get the list of peripherals found
    // std::vector<SimpleBLE::Peripheral> peripherals = adapter.scan_get_results();

    while(!arduino.initialized()) std::this_thread::sleep_for(std::chrono::milliseconds (1000));
    arduino.set_callback_on_connected([&]()
    {
        connected = true;
        std::cout << "connected!\n\n";
        // Store all service and characteristic uuids in a vector.

        for (auto service: arduino.services())
        {
          for (auto characteristic: service.characteristics())
          {
              uuids.push_back(std::make_pair(service.uuid(), characteristic.uuid()));
          }
        }

        ROS_INFO("The following services and characteristics were found:");
        for (size_t i = 0; i < uuids.size(); i++)
            ROS_INFO_STREAM("[" << i << "] " << uuids[i].first << " " << uuids[i].second);


        // Subscribe to the characteristic.
        arduino.notify(uuids[1].first, uuids[1].second, [&](SimpleBLE::ByteArray rx_data)
        {
            float f1, f2;
            size_t n_bytes = rx_data.length();
            uint8_t rx_int[n_bytes];
            for(size_t i = 0; i < n_bytes; ++i)
                rx_int[i] =  (uint8_t)rx_data[i];

            memcpy(&f1, &rx_int[0], 4);
            memcpy(&f2, &rx_int[4], 4);

            // publish EMG data
            emg.ch1 = f1;
            emg.ch2 = f2;
            emg_pub.publish(emg);
        });
    });



    arduino.set_callback_on_disconnected([&connected, &uuids]()
    {
        connected = false;
        uuids.clear();
        ROS_WARN_STREAM("Peripheral disconnected.");
    });


    // Establish a connection to the Arduino device
    std::cout << "Connecting to Arduino ... ";
    if(arduino.is_connectable())
        arduino.connect();

    while (ros::ok())
    {
        if(connected)
        {



        }
        else if(!adapter.scan_is_active() && !found)
        {
            ROS_INFO_ONCE("Trying to connect again ...");
            adapter.scan_start();
        }
        else if(found)
        {
            arduino.connect();
            while(!arduino.is_connected())
            {
                loop_rate.sleep();
            }
        }

        // motor_pub.publish(motor);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Cleanup
    ROS_INFO("Disconnecting peripheral ...");
    arduino.disconnect();
    while (arduino.is_connected())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ROS_INFO("Peripheral disconnected.");

    return 0;
}