#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State

def battery_cb(msg):
    rospy.loginfo(f"Battery: {msg.percentage * 100:.2f}% | Voltage: {msg.voltage}V")

def state_cb(msg):
    rospy.loginfo(f"Connected: {msg.connected} | Mode: {msg.mode} | Armed: {msg.armed}")

def main():
    rospy.init_node('telemetry_node', anonymous=True)

    rospy.Subscriber('/mavros/battery', BatteryState, battery_cb)
    rospy.Subscriber('/mavros/state', State, state_cb)

    rospy.spin()

if __name__ == '__main__':
    main()
