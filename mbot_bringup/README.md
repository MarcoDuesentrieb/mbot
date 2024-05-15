# mbot_bringup

ROS package starting the ROS interface for mBot.

## Nodes

### mbot_battery_monitor

This node establishes a connection with the [x728 UPS](https://wiki.geekworm.com/X728) board through I2C to retrieve information regarding the battery module. The information obtained includes the battery's State of Charge (SoC) and the connection and charging status of a charger. Additionally, it manages the x728's onboard buzzer to alert on low battery conditions.

#### Topics published

- `/mbot/battery (sensor_msgs/Battery)`:  
  Battery status

### mbot_bringup_node

This node initiates a Bluetooth Low Energy (BLE) connection with the Arduino Nano 33 IoT using the [SimpleBLE](https://github.com/OpenBluetoothToolbox/SimpleBLE) C++ library. It sets up the Raspberry Pi to function as a BLE central device. The node subscribes to the Electromyography (EMG) characteristic in notification mode offered by the Arduino Nano 33 IoT peripheral (repository: [BLE-Sensor](https://github.com/MarcoDuesentrieb/BLE-Sensor)). The data acquired, which reflects the voltage measurements from two EMG sensors, is subsequently broadcasted as an `mbot_msgs/EMG` message on the `/mbot/emg` topic.

#### Topics published

- `/mbot/emg (mbot_msgs/EMG)`:  
  EMG voltage from both Sensor 1 and Sensor 2.

### mbot_controller_node

This node is responsible for producing motor command outputs for the mBots' motors. It monitors the `/mbot/emg` topic to gather EMG data from both sensors. Subsequently, it processes these signals to generate motor commands, which are then disseminated as `mbot_msgs/Motor` messages through the `/mbot/motor` topic. These messages are received by the mBots' mCore board, which is integrated via ROSserial, to control the motors' operation. The node utilizes the minimum and maximum EMG voltage parameters defined in the `mbot_parameters.yaml` file, converting the EMG readings from this defined range into motor commands that vary from -255 to 255. Here, -255 signifies full reverse motion, while 255 indicates full forward motion of the motor.

#### Topics subscribed

- `/mbot/ultrasonic (sensor_msgs/Range)`:  
  Distance measured by mBot's utrasonic sensor
  
- `/mbot/emg (mbot_msgs/EMG)`:  
  EMG voltage from both Sensor 1 and Sensor 2.

#### Topics published

- `/mbot/motor (mbot_msgs/Motor)`:  
  Motor commands (-255: full backward, 255: full forward) for mCore to drive the left and right wheel. 

## Parameter files

### battery_parameters.yaml

- `buzzer_alarm` (bool, default: true):  
  Whether to use the x728 UPS onboard buzzer for low-battery alarm.

- `low_bat_threshold` (double, default: 0.05):  
  Threshold level of battery State of Charge (SoC) in percentage, below which the buzzer alarm is activated.

- `buzzer_interval` (double, default: 120):  
  Time interval (seconds) after which the buzzer is triggered if  `buzzer_alarm` is true and battery is less than `low_bat_threshold`.

### mbot_parameters.yaml

- `rate` (double, default: 25):  
  Spin rate (Hz) for `mbot_bringup` and `mbot_controller` node

- `peripheral_address` (string):  
  Device address of the BLE peripheral to connect to.

- `ultrasonic_safety_distance` (float, default: 0.19):  
  Safety distance (meters) of ultrasonic sensor, below which the motors stop moving regardless of motor command sent. 

- `emg_timeout` (double):  
  Timeout (seconds) after which the `mbot_controller` node sends a stop command for the motor as a safety measure, in case there is no current EMG data available.

- `v_emg01_min` (float):  
  Sensor 1 minimum expected EMG voltage.  

- `v_emg01_max` (float):  
  Sensor 1 maximum expected EMG voltage.  

- `v_emg02_min` (float):  
  Sensor 2 minimum expected EMG voltage.  

- `v_emg02_max` (float):  
  Sensor 2 maximum expected EMG voltage.  

### demo_parameters.yaml

This file contains the configuration for the simple demo application. For a detailed description of used parameters, see section [`mbot_parameters.yaml`](#mbot_parameters.yaml).

## Launch files

### mbot_battery.launch

Launches the  

- `mbot_battery_monitor` node

### mbot_bringup.launch

Launches the 

- `mbot_parameters` parameter file
- `mbot_bringup` node
- `mbot_battery` launch file### mbot_bringup.launch

### mbot_controller.launch

Launches the 

- `mbot_parameters` parameter file
- `mbot_controller` node
- `mbot_rosserial` launch file

### mbot_demo.launch

Launches the 

- `demo_parameters` parameter file
- `mbot_controller` node
- `mbot_bringup` launch file
- `mbot_rosserial` launch file

### mbot_rosserial.launch

Launches the 

- `serial_node.py` node of package `rosserial_python` with parameters:  
  - `_port` (string, default: "/dev/ttyS0")  
  - `_baud` (int, default: 115200)  


