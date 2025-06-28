#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
import re
import threading
import time
import math
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.path as mplPath
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic

class SingleDroneLawnmowerNode:
    def __init__(self):
        rospy.init_node('single_drone_lawnmower_node', anonymous=True)

        self.state = State()
        self.gps_position = NavSatFix()
        self.local_position = PoseStamped()
        self.geotags = []
        self.geotagging_distance = 25.0
        self.grid_size = 25.0
        self.flight_altitude = 10.0

        self.polygon_coords = []
        self.waypoints = []
        self.mission_active = False

        self.setup_ros_interface()
        self.parse_kml_and_generate_waypoints()

        rospy.loginfo("Single Drone Lawnmower Node Initialized")

    def setup_ros_interface(self):
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pos_callback)

        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.geotag_pub = rospy.Publisher('/geotag_data', String, queue_size=10)

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def state_callback(self, msg):
        self.state = msg

    def gps_callback(self, msg):
        self.gps_position = msg

    def local_pos_callback(self, msg):
        self.local_position = msg

    def parse_kml_and_generate_waypoints(self):
        try:
            kml_path = "/root/catkin_ws/src/geotag/scripts/farm_iitindore.kml"
            if not os.path.exists(kml_path):
                rospy.logerr(f"KML file not found: {kml_path}")
                return

            tree = ET.parse(kml_path)
            root = tree.getroot()
            namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

            coords_text = root.find(".//kml:coordinates", namespace)
            if coords_text is None:
                rospy.logerr("No <coordinates> found in KML")
                return

            coord_string = coords_text.text.strip()
            coords = []
            for line in coord_string.split():
                lon, lat, *_ = map(float, line.split(','))
                coords.append((lat, lon))

            self.polygon_coords = coords
            rospy.loginfo(f"Parsed {len(coords)} polygon coordinates from KML")

            self.generate_lawnmower_pattern()
        except Exception as e:
            rospy.logerr(f"Error parsing KML: {e}")

    def generate_lawnmower_pattern(self):
        lats = [coord[0] for coord in self.polygon_coords]
        lons = [coord[1] for coord in self.polygon_coords]
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)
        avg_lat = (lat_min + lat_max) / 2
        lat_step = self.grid_size / 111000.0
        lon_step = self.grid_size / (111000.0 * math.cos(math.radians(avg_lat)))
        lat_range = np.arange(lat_min, lat_max + lat_step, lat_step)
        lon_range = np.arange(lon_min, lon_max + lon_step, lon_step)
        polygon = np.array(self.polygon_coords)
        path = mplPath.Path(polygon)
        all_waypoints = []
        k = 0
        for j in lat_range:
            row_points = []
            if k % 2 == 0:
                for i in lon_range:
                    if path.contains_points([[j, i]])[0]:
                        row_points.append((j, i))
            else:
                for i in reversed(lon_range):
                    if path.contains_points([[j, i]])[0]:
                        row_points.append((j, i))
            all_waypoints.extend(row_points)
            k += 1
        half = len(all_waypoints) // 2
        self.waypoints = all_waypoints[:half]  # Only one half

    def calculate_distance(self, pos1, pos2):
        return geodesic((pos1[0], pos1[1]), (pos2[0], pos2[1])).meters

    def should_place_geotag(self, current_pos):
        if not self.geotags:
            return True
        last = self.geotags[-1]
        return self.calculate_distance((last['latitude'], last['longitude']), (current_pos.latitude, current_pos.longitude)) >= self.geotagging_distance

    def place_geotag(self, position):
        geotag = {
            'latitude': position.latitude,
            'longitude': position.longitude,
            'altitude': position.altitude,
            'timestamp': rospy.Time.now().to_sec()
        }
        self.geotags.append(geotag)
        self.geotag_pub.publish(String(data=json.dumps(geotag)))

    def create_pose_stamped(self, lat, lon, alt=None):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = (lon - self.polygon_coords[0][1]) * 111000 * math.cos(math.radians(lat))
        pose.pose.position.y = (lat - self.polygon_coords[0][0]) * 111000
        pose.pose.position.z = alt if alt else self.flight_altitude
        pose.pose.orientation.w = 1.0
        return pose

    def execute_mission(self):
        if not self.waypoints:
            rospy.logerr("No waypoints generated")
            return
        self.mission_active = True
        rate = rospy.Rate(10)
        for i, (lat, lon) in enumerate(self.waypoints):
            if not self.mission_active:
                break
            target_pose = self.create_pose_stamped(lat, lon)
            rospy.loginfo(f"Navigating to waypoint {i+1}/{len(self.waypoints)}")
            start_time = time.time()
            while self.mission_active:
                self.pos_pub.publish(target_pose)
                if self.gps_position.latitude != 0 and self.gps_position.longitude != 0:
                    dist = self.calculate_distance((self.gps_position.latitude, self.gps_position.longitude), (lat, lon))
                    if dist < 2.0:
                        if self.should_place_geotag(self.gps_position):
                            self.place_geotag(self.gps_position)
                        break
                if time.time() - start_time > 60:
                    rospy.logwarn("Timeout, skipping waypoint")
                    break
                rate.sleep()
        self.mission_active = False
        self.save_geotags_to_json()

    def save_geotags_to_json(self):
        output_data = {
            'mission_info': {
                'total_geotags': len(self.geotags),
                'geotagging_distance': self.geotagging_distance,
                'timestamp': rospy.Time.now().to_sec()
            },
            'geotags': self.geotags
        }
        filename = f"geotags_mission_{int(rospy.Time.now().to_sec())}.json"
        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=2)
        rospy.loginfo(f"Geotags saved to {filename}")

    def prepare_drone(self):
        for _ in range(5):
            if self.set_mode_client(0, "OFFBOARD").mode_sent:
                rospy.loginfo("OFFBOARD mode set")
                break
            rospy.sleep(1)
        for _ in range(5):
            if self.arming_client(True).success:
                rospy.loginfo("Drone armed")
                break
            rospy.sleep(1)

    def start_mission(self):
        rospy.loginfo("Waiting for drone connection...")
        while not rospy.is_shutdown():
            if self.state.connected:
                break
            rospy.sleep(1)
        self.prepare_drone()
        self.execute_mission()

def main():
    try:
        node = SingleDroneLawnmowerNode()
        mission_thread = threading.Thread(target=node.start_mission)
        mission_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")

if __name__ == '__main__':
    main()
