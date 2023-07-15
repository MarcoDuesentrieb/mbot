#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <simpleble/SimpleBLE.h>
#include <cstring>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <mbot_msgs/Motor.h>


void reverseBytes(std::string &s)
// for converting between Little-Endian and Big-Endian
{
    auto copy = s;
    size_t len = s.length();

    for(size_t i = 0; i < len; ++i)
        s[i] = copy[len-1-i];
}

float hexStringToFloat(const std::string& hexString) {

    std::stringstream ss;

    for(auto c : hexString)
        ss << std::hex << (int)c;

    // Convert the stripped string to an integer
    //std::stringstream ss;
    //ss << std::hex << hexString;
    unsigned int intValue;
    ss >> intValue;

    // Interpret the integer as a float
    float floatValue;
    std::memcpy(&floatValue, &intValue, sizeof(float));

    return floatValue;
}


int main(int argc, char** argv)
{

    //  ### INITIALISATION ###
    ros::init(argc, argv, "mbot_bringup_node");
    ros::NodeHandle nh;

    // Read peripheral_address parameter
    std::string peripheral_address;
    nh.param<std::string>("peripheral_address", peripheral_address, "");
    SimpleBLE::Peripheral arduino;

    // ROS publishers
    ros::Publisher emg01_pub = nh.advertise<std_msgs::Float32>("/mbot/emg01", 10);
    ros::Publisher emg02_pub = nh.advertise<std_msgs::Float32>("/mbot/emg02", 10);
    ros::Publisher motor_pub = nh.advertise<mbot_msgs::Motor>("/mbot/motor", 10);

    // ROS messages
    std_msgs::Float32 emg01;
    std_msgs::Float32 emg02;
    mbot_msgs::Motor motor;

    // Read rate parameter
    double rate;
    nh.param<double>("rate", rate, 20.0); // Default rate: 10 Hz
    ros::Rate loop_rate(rate);

    // Check if the systems bluetooth adapter is enabled
    if (!SimpleBLE::Adapter::bluetooth_enabled()) {
        std::cout << "Bluetooth is not enabled" << std::endl;
        return 1;
    }

    // Get a list of all available adapters
    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty()) {
        std::cout << "No Bluetooth adapters found" << std::endl;
        return 1;
    }

    // Use the first adapter
    auto adapter = adapters[0];

    // Print adapter info
    std::cout << "Adapter identifier: " << adapter.identifier() << "\n";
    std::cout << "Adapter address: " << adapter.address() << "\n\n";




    //  ### Adapter CALLBACK DEFINITIONS ###

    // Set the callback to be called when the scan starts
    adapter.set_callback_on_scan_start([]()
    {
        std::cout << "Scanning for peripherals... " << std::endl;
    });

    // Set the callback to be called when the scan stops
    adapter.set_callback_on_scan_stop([]()
    {
        std::cout << "Scan stopped" << std::endl;
    });

    // Set the callback to be called when the scan finds a new peripheral
    adapter.set_callback_on_scan_found([&adapter, &arduino, &peripheral_address](SimpleBLE::Peripheral peripheral)
    {
        // std::cout << "Peripheral found: " << peripheral.identifier() << std::endl;
        // std::cout << "Peripheral address: " << peripheral.address() << std::endl;
        if(peripheral.address() == peripheral_address)
        {
            std::cout << "YES we found the desired peripheral!\n";
            adapter.scan_stop();
            arduino = peripheral;
        }
    });


    //  ### START ###

    // Start scanning for peripherals
    adapter.scan_start();

    // Get the list of peripherals found
    // std::vector<SimpleBLE::Peripheral> peripherals = adapter.scan_get_results();

    while(!arduino.initialized()) std::this_thread::sleep_for(std::chrono::milliseconds (100));
    arduino.set_callback_on_connected([&arduino, &emg01]()
    {

        std::cout << "connected!\n\n";
        // Store all service and characteristic uuids in a vector.
        std::vector<std::pair<SimpleBLE::BluetoothUUID, SimpleBLE::BluetoothUUID>> uuids;
        for (auto service: arduino.services())
        {
          for (auto characteristic: service.characteristics())
          {
              uuids.push_back(std::make_pair(service.uuid(), characteristic.uuid()));
          }
        }

        std::cout << "The following services and characteristics were found:" << std::endl;
        for (size_t i = 0; i < uuids.size(); i++)
            std::cout << "[" << i << "] " << uuids[i].first << " " << uuids[i].second << "\n";

            SimpleBLE::ByteArray rx_data = arduino.read(uuids[1].first, uuids[1].second);

            // the Arduino Nano 33 IOT uses big endian, so we have to reverse the bytes
            reverseBytes(rx_data);

            std::stringstream ss;
            for(auto c : rx_data) ss << std::hex << (int)c;

            float rx_data_f = hexStringToFloat(rx_data);
            std::cout << "\nCharacteristic content is: 0x";
            std:: cout << ss.str() << std::endl;
            std:: cout << "Float: " << rx_data_f << std::endl;

            emg01.data = rx_data_f;

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    // Establish a connection to the Arduino device
    std::cout << "Connecting to Arduino ... ";
    if(arduino.is_connectable())
        arduino.connect();

    while (ros::ok())
    {
        emg01_pub.publish(emg01);
        // motor_pub.publish(motor);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //while(!arduino.is_connected()) std::this_thread::sleep_for(std::chrono::milliseconds(100));


    return 0;
}