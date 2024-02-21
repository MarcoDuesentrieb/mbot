# mbot_messages

ROS package for mBot's custom message types.

## EMG.msg

Message used as a container for EMG voltage of each EMG sensor.  

Layout:  

```c++
float32 ch1     # EMG sensor 1 voltage (in V)
float32 ch2     # EMG sensor 2 voltage (in V)
```

## Motor.msg

Message used as a container for both left and right motor command.  

Layout:  

```c++
int16 left      # Left motor command
int16 right     # Right motor command
```