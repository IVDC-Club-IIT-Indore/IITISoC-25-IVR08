#!/usr/bin/env python3

import rospy
import json
import math
import os
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

DEG_TO_M = 111139  # meters per degree latitude approx

class DynamicFlightNode:
    def __init__(self):
        rospy.init_node("dynamic_flight_node")

        # File paths
        self.input_path = "/root/catkin_ws/src/geotag/scripts/global_aco_path.json"
        self.z_alt = 2.5
        self.rate = rospy.Rate(10)  # 10Hz

        self.state = State()
        self.current_index = 0
        self.ref_lat = None
        self.ref_lon = None

        rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Wait for services
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Startup sequence
        self.wait_for_connection()
        self.send_initial_setpoints()
        self.set_mode_and_arm()

        # Start main loop
        self.flight_loop()

    def state_cb(self, msg):
        self.state = msg

    def wait_for_connection(self):
        rospy.loginfo("🔌 Waiting for FCU connection...")
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("✅ FCU connected")

    def send_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def send_initial_setpoints(self):
        rospy.loginfo("🌀 Sending initial setpoints...")
        for _ in range(50):
            self.send_pose(0, 0, self.z_alt)
            self.rate.sleep()

    def set_mode_and_arm(self):
        rospy.loginfo("🛫 Attempting OFFBOARD mode + arming...")
        for _ in range(10):
            self.send_pose(0, 0, self.z_alt)
            if self.state.mode != "OFFBOARD":
                self.mode_srv(base_mode=0, custom_mode="OFFBOARD")
            if not self.state.armed:
                self.arm_srv(True)
            if self.state.mode == "OFFBOARD" and self.state.armed:
                rospy.loginfo("✅ OFFBOARD mode set and drone armed")
                return
            self.rate.sleep()
        rospy.logerr("❌ Failed to arm and set OFFBOARD. Aborting.")
        rospy.signal_shutdown("Mode or arm failed")

    def convert_gps_to_local(self):
        if not os.path.exists(self.input_path):
            rospy.logwarn("❌ GPS JSON not found")
            return []

        try:
            with open(self.input_path, 'r') as f:
                gps_data = json.load(f)

            gps_data = list(reversed(gps_data))  # optional fix


            if not gps_data:
                return []

            if self.ref_lat is None or self.ref_lon is None:
                self.ref_lat = gps_data[0]["center_latitude"]
                self.ref_lon = gps_data[0]["center_longitude"]
                rospy.loginfo(f"📍 Using ref lat/lon: {self.ref_lat}, {self.ref_lon}")

            local_waypoints = []
            for wp in gps_data:
                lat = wp["center_latitude"]
                lon = wp["center_longitude"]
                dx = (lon - self.ref_lon) * DEG_TO_M * math.cos(math.radians(self.ref_lat))
                dy = (lat - self.ref_lat) * DEG_TO_M
                local_waypoints.append((dx, dy))

            return local_waypoints

        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to parse GPS JSON: {e}")
            return []

    def flight_loop(self):
        rospy.loginfo("📡 Starting live flight loop...")
        while not rospy.is_shutdown():
            waypoints = self.convert_gps_to_local()

            if self.current_index < len(waypoints):
                x, y = waypoints[self.current_index]
                rospy.loginfo(f"➡️ Sending WP {self.current_index+1}: x={x:.2f}, y={y:.2f}, z={self.z_alt:.2f}")
                for _ in range(30):  # 3 seconds per WP
                    self.send_pose(x, y, self.z_alt)
                    self.rate.sleep()
                self.current_index += 1
            else:
                self.send_pose(x, y, self.z_alt)
                self.rate.sleep()

if __name__ == "__main__":
    try:
        DynamicFlightNode()
    except rospy.ROSInterruptException:
        pass
