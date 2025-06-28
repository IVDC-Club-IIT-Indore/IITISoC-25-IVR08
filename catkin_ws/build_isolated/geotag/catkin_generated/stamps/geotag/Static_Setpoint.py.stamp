#!/usr/bin/env python

import rospy
import json
import time
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import State

class AutoMissionUploader:
    def __init__(self):
        rospy.init_node('auto_mission_uploader')

        self.json_path = '/root/catkin_ws/src/geotag/scripts/global_aco_path.json'
        self.fixed_alt = 20.0

        self.state = State()
        rospy.Subscriber('/mavros/state', State, self.state_cb)

        rospy.wait_for_service('/mavros/mission/push')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')

        self.push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.rate = rospy.Rate(10)
        self.wait_for_connection()
        self.upload_and_run()

    def state_cb(self, msg):
        self.state = msg

    def wait_for_connection(self):
        rospy.loginfo("🔌 Waiting for FCU connection...")
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()
        rospy.loginfo("✅ FCU connected.")

    def create_waypoint(self, lat, lon, alt, is_current=False):
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 16  # NAV_WAYPOINT
        wp.is_current = is_current
        wp.autocontinue = True
        wp.param1 = 0  # Hold time
        wp.param2 = 5  # Acceptance radius
        wp.param3 = 0
        wp.param4 = float('nan')
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        return wp

    def load_waypoints(self):
        with open(self.json_path, 'r') as f:
            data = json.load(f)

        waypoints = []
        for i, item in enumerate(data):
            lat = item['center_latitude']
            lon = item['center_longitude']
            wp = self.create_waypoint(lat, lon, self.fixed_alt, is_current=(i == 0))
            waypoints.append(wp)
        return waypoints

    def upload_and_run(self):
        # 1. Load and push waypoints
        rospy.loginfo("📦 Loading and pushing waypoints...")
        waypoints = self.load_waypoints()
        resp = self.push_srv(start_index=0, waypoints=waypoints)
        if resp.success:
            rospy.loginfo(f"✅ Uploaded {resp.wp_transfered} waypoints.")
        else:
            rospy.logerr("❌ Failed to upload mission.")
            return

        # 2. Set AUTO.MISSION mode
        rospy.loginfo("🛫 Setting mode: AUTO.MISSION...")
        mode_resp = self.set_mode_srv(custom_mode='AUTO.MISSION')
        if mode_resp.mode_sent:
            rospy.loginfo("✅ Mode set to AUTO.MISSION.")
        else:
            rospy.logerr("❌ Failed to set AUTO.MISSION.")
            return

        # 3. Arm the drone
        rospy.loginfo("🔐 Arming drone...")
        arm_resp = self.arm_srv(True)
        if arm_resp.success:
            rospy.loginfo("✅ Drone armed. Mission starting!")
        else:
            rospy.logerr("❌ Failed to arm drone.")

if __name__ == '__main__':
    try:
        AutoMissionUploader()
    except rospy.ROSInterruptException:
        pass
