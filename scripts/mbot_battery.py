#!/usr/bin/python

# This node is for interfacing the x728 UPS for Raspberry Pi.
# It monitors and publishes the UPS battery state of charge and (depending on the parameters set
# in battery_parameters.yaml) activates the x728 onboard buzzer to notify the user of low battery.

import rospy
from time import sleep
from sensor_msgs.msg import BatteryState
import struct
import smbus
import RPi.GPIO as GPIO

# Global settings
# GPIO26 is for x728 V2.1/V2.2/V2.3, GPIO13 is for X728 v1.2/v1.3/V2.0
PLD_PIN       = 6
BUZZER_PIN    = 20  # x728 UPS has a buzzer that can be used to warn the user of low battery
I2C_ADDR      = 0x36
bus = smbus.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

GPIO.setmode(GPIO.BCM)
GPIO.setup(PLD_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setwarnings(True)

def readVoltage(bus):
     read = bus.read_word_data(I2C_ADDR, 2)
     swapped = struct.unpack("<H", struct.pack(">H", read))[0]
     voltage = swapped * 1.25 /1000/16
     return voltage

def readCapacity(bus):
     read = bus.read_word_data(I2C_ADDR, 4)
     swapped = struct.unpack("<H", struct.pack(">H", read))[0]
     capacity = swapped/256
     return capacity

def buzz(duration):
    GPIO.output(BUZZER_PIN, 1)
    sleep(duration)
    GPIO.output(BUZZER_PIN, 0)
    sleep(duration)


class Battery:
    def __init__(self):
        rospy.init_node('mbot_battery_monitor')
        self.battery_state_pub = rospy.Publisher('/mbot/battery', BatteryState, queue_size=10)
        self.battery_state_msg = BatteryState()
        self.rate = 5
        self.timer = None
        self.power_supply_present = False

        # Read parameters from config file
        self.buzzer_alarm = rospy.get_param('buzzer_alarm', False)
        self.buzzer_interval = rospy.get_param('buzzer_interval', 60.0)
        self.low_bat_threshold = rospy.get_param('low_bat_threshold', 0.05)

        # Refer to BatteryState message definition: 
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html
        self.battery_state_msg.capacity = float('nan') #  Capacity in Ah (last full capacity)  (If unmeasured NaN)
        self.battery_state_msg.design_capacity = 2 * 3.120
        self.battery_state_msg.power_supply_status = self.get_power_supply_status()
        self.battery_state_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        self.battery_state_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_state_msg.present = True

        GPIO.add_event_detect(PLD_PIN, GPIO.BOTH, self.pld_callback)  # add rising edge detection on a channel
        # Trigger first event manually to catch low battery state from the beginning
        self.pld_callback(PLD_PIN)
        


    # power loss detection callback
    def pld_callback(self, channel):
        if GPIO.input(PLD_PIN) == GPIO.HIGH:
            self.power_supply_present = False
            rospy.loginfo("Power supply disconnected.")
            if self.buzzer_alarm and self.battery_state_msg.percentage < self.low_bat_threshold:
                self.buzzer_callback(None)
                if self.timer is None or self.timer._shutdown:
                    self.timer = rospy.Timer(rospy.Duration(self.buzzer_interval), self.buzzer_callback)
        else:
            self.power_supply_present = True
            rospy.loginfo("Power supply connected.")
            if self.timer is not None:
                    self.timer.shutdown()
                

    def buzzer_callback(self, event):
        rospy.logwarn("BATTERY LOW")
        buzz(0.1)
        buzz(0.1)

    def get_power_supply_status(self):
        if readCapacity(bus) == 100:
            return BatteryState.POWER_SUPPLY_STATUS_FULL
        elif self.power_supply_present:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

    def update(self):
        self.battery_state_msg.header.stamp = rospy.Time.now()
        self.battery_state_msg.power_supply_status = self.get_power_supply_status()
        self.battery_state_msg.voltage = readVoltage(bus)
        self.battery_state_msg.percentage = readCapacity(bus)/100.0
        self.battery_state_pub.publish(self.battery_state_msg)


    def spin(self):
        rospy.loginfo('Start mbot_battery_monitor')
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
        rospy.spin()


    def shutdown(self):
        rospy.loginfo('Stop mbot_battery_monitor')
        if self.timer is not None:
            self.timer.shutdown()
        GPIO.output(BUZZER_PIN, 0)
        rospy.sleep(0.5)
        GPIO.cleanup()
        rospy.sleep(0.5)


def main():
    battery = Battery()
    battery.spin()


if __name__ == '__main__':
    main()