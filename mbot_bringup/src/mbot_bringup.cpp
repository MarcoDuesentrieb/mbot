#include <iostream>
#include <sstream>
#include <chrono>
#include <signal.h>
#include <thread>
#include <simpleble/SimpleBLE.h>
#include <cstring>
#include <functional> // For std::function

/* ROS includes */
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <mbot_bringup/dynrecConfig.h>
#include <std_msgs/Float32.h>
#include <mbot_msgs/EMG.h>


void reverseBytes(std::string &s)
// for converting between Little-Endian and Big-Endian
{
    auto copy = s;
    size_t len = s.length();

    for(size_t i = 0; i < len; ++i)
        s[i] = copy[len-1-i];
}

std::string getHostname()
{
    // Get the hostname
    char hostname[25];
    if (gethostname(hostname, sizeof(hostname)) != 0)
    {
        // TODO: Error handling
        return "unknown";
    }
    else
    {
        // Ensure the hostname is null-terminated
        hostname[sizeof(hostname) - 1] = '\0';
        return std::string(hostname);
    }
}


class MbotBringupNode
{
public:
    MbotBringupNode()
    {
        // Indicate connection state as ROS parameter
        ros::param::set("peripheral_connected", false);

        // List of MAC-Addresses of all available EMG sensors is stored in mbot_parameters.yaml
        while(true)
        {
            static int id = 1;
            std::string param_name = "emg_sensor_address_";
            if(id < 10) param_name += "0" + std::to_string(id);
            else param_name += std::to_string(id);
            std::string address;
            if (ros::param::has(param_name))
            {
                ros::param::get(param_name, address);
                sensorMap.emplace(id++, address);
                ROS_INFO_STREAM("Added address " << address << " to list of peripherals");
            }
            else break;
        }

        // Setup Dynamic Reconfigure 
        f = boost::bind(&MbotBringupNode::dynRecCallback, this, _1, _2);
        server.setCallback(f);

        // The correct sensor number is retrieved from evaluating the hostname (last 2 characters)
        // E.g. hostname: mbot-0x --> sensor_param_name: "emg_sensor_address_0x"
        auto hostname = getHostname();

        // Extract the last two characters (this is the sensor number)
        emg_sensor_id = stoi(hostname.substr(hostname.length() - 2));

        try
        {
            peripheral_address = sensorMap.at(emg_sensor_id);
            ros::param::set("emg_sensor_id", emg_sensor_id);
            ROS_INFO_STREAM("Set parameter emg_sensor_id to: " << emg_sensor_id);
        }
        catch(std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
        } 

        emg_pub = nh.advertise<mbot_msgs::EMG>("/mbot/emg", 10);
        
        // Read rate parameter
        nh.param<double>("rate", rate, 20.0); // Default rate: 10 Hz

        setupBluetoothAdapter();
        
        // Start scanning for peripherals
        adapter.scan_start();
    }

    ~MbotBringupNode()
    {
        ROS_INFO("Disconnecting peripheral ...");
        arduino.disconnect();
        while (arduino.is_connected())
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ROS_INFO("Peripheral disconnected.");
        ros::shutdown();
    }

    void run()
    {
        while (ros::ok())
        {
            ros::Rate loop_rate(rate);
            if(arduino.initialized())
            {
                if(!arduino.is_connected() && !connecting)
                {
                    ROS_INFO("Trying to reconnect ...");
                    try
                    {
                        connecting = true;
                        arduino.connect();
                        // if(!adapter.scan_is_active())
                        // {
                        //     adapter.scan_start();
                        // }
                    }
                    catch(std::exception& e)
                    {
                        // placeholder
                    }
                }
            }

            

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    // ROS
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<mbot_bringup::dynrecConfig> server;
    dynamic_reconfigure::Server<mbot_bringup::dynrecConfig>::CallbackType f;
    ros::Publisher emg_pub;
    mbot_msgs::EMG emg;
    double rate; // loop rate of the node

    // BLE
    std::map<int, std::string> sensorMap; // associate sensor ID with hardware address strings
    std::string peripheral_address;
    SimpleBLE::Adapter adapter;
    SimpleBLE::Peripheral arduino;
    std::vector<std::pair<SimpleBLE::BluetoothUUID, SimpleBLE::BluetoothUUID>> uuids;
    int emg_sensor_id;
    bool connected = false;
    bool connecting = false;
    bool found = false;


    void dynRecCallback(mbot_bringup::dynrecConfig &config, uint32_t level)
    {
        if(!adapter.initialized()) return;

        auto new_peripheral_address = sensorMap.at(config.emg_sensor_id);

        if (peripheral_address != new_peripheral_address)
        {
            peripheral_address = new_peripheral_address;
            emg_sensor_id = config.emg_sensor_id;
            ROS_INFO_STREAM("Changed EMG sensor ID to: " << config.emg_sensor_id << " (" << new_peripheral_address << ")");
            ros::param::set("emg_sensor_id", config.emg_sensor_id);

            // Stop any ongoing scan
            if(adapter.scan_is_active())
                adapter.scan_stop();

            // Disconnect if currently connected
            if (connected) {
                if(arduino.initialized())
                    arduino.disconnect();
                connected = false;
                uuids.clear();
                found = false;
            }

            // Start scanning for the new peripheral address
            adapter.scan_start();
        }
    }


    int setupBluetoothAdapter()
    {
        // Check if the systems bluetooth adapter is enabled
        if(!SimpleBLE::Adapter::bluetooth_enabled())
        {
            ROS_ERROR("Bluetooth is not enabled");
            return 1;
        }

        // Get a list of all available adapters
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty())
        {
            ROS_ERROR("No Bluetooth adapters found");
            return 1;
        }

        // Use the first adapter
        adapter = adapters[0];

        // Print adapter info
        ROS_INFO_STREAM("Adapter identifier: " << adapter.identifier());
        ROS_INFO_STREAM("Adapter address: " << adapter.address());

        // Set the callback to be called when the scan starts
        adapter.set_callback_on_scan_start([this]()
        {
            ROS_INFO_STREAM("Scanning for peripherals... ");
            ROS_INFO_STREAM("Searching for: " << peripheral_address);
        });

        // Set the callback to be called when the scan stops
        adapter.set_callback_on_scan_stop([this]()
        {
            ROS_INFO("Scan stopped");
        });


        // Set the callback to be called when the scan finds a new peripheral
        adapter.set_callback_on_scan_found([this](SimpleBLE::Peripheral peripheral)
        {
            if(peripheral.address() == peripheral_address)
            {
                ROS_INFO("Requested peripheral found!");
                adapter.scan_stop();
                arduino = peripheral;
                found = true;

                // Setup peripheral event callbacks
                setupBluetoothPeripheral();
            }
        });

        return 0;
    }

    void setupBluetoothPeripheral()
    {
        // Get the list of peripherals found
        // std::vector<SimpleBLE::Peripheral> peripherals = adapter.scan_get_results();

        while(!arduino.initialized()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        arduino.set_callback_on_connected([this]()
        {
            connected = true;
            connecting = false;
            ROS_INFO("Connected!");

            // Set peripheral_connected ROS parameter to true on connection
            ros::param::set("peripheral_connected", true);
                
            // Store all service and characteristic uuids in a vector.
            for (auto service: arduino.services())
                for (auto characteristic: service.characteristics())
                    uuids.push_back(std::make_pair(service.uuid(), characteristic.uuid()));

            if(uuids.size() == 0)
            {
                ROS_WARN("No services and characteristics were found!");
                ROS_WARN("Trying to dis- and reconnect ...");
                arduino.disconnect();
                while (arduino.is_connected())
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                found = false;
                return;
            }

            ROS_INFO("The following services and characteristics were found:");
            for (size_t i = 0; i < uuids.size(); i++)
                ROS_INFO_STREAM("[" << i << "] " << uuids[i].first << " " << uuids[i].second);

            // Subscribe to the characteristic.
            arduino.notify(uuids[1].first, uuids[1].second, [this](SimpleBLE::ByteArray rx_data)
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


        arduino.set_callback_on_disconnected([this]()
        {
            connected = false;
            uuids.clear();
            ros::param::set("peripheral_connected", false);
            ROS_WARN_STREAM("Peripheral disconnected.");
            
        });

        // Establish a connection to the Arduino device
        if(arduino.is_connectable())
        {
            ROS_INFO_STREAM("The peripheral has the following characteristics: " << arduino.services().size());

            ROS_INFO_STREAM("Connecting to EMG sensor " << emg_sensor_id << " (" << peripheral_address << ")");
            connecting = true;
            try
            {

                arduino.connect();
            }
            catch(std::exception& e)
            {
                ROS_WARN("Unable to connect. Is the peripheral powered on?");
            }
            
        }
        else
        {
            ROS_ERROR_STREAM("Peripheral " << peripheral_address << " is not connectable!");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbot_bringup_node");
    MbotBringupNode node;
    
    try
    {
        node.run();
    }
    catch(std::exception& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    
    return 0;
}
