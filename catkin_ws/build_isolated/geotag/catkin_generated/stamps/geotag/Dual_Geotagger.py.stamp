#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import xml.etree.ElementTree as ET
import re
import matplotlib.path as mplPath
import numpy as np
import math
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic
import threading
import time
import xml.etree.ElementTree as ET  # At top of file if not already

class DualDroneLawnmowerNode:
    def __init__(self):
        rospy.init_node('dual_drone_lawnmower_node', anonymous=True)

        self.drone_states = {
            'drone1': {'state': State(), 'position': NavSatFix(), 'local_pos': PoseStamped()},
            'drone2': {'state': State(), 'position': NavSatFix(), 'local_pos': PoseStamped()}
        }

        self.geotags = {'drone1': [], 'drone2': []}
        self.geotagging_distance = 25.0
        self.grid_size = 25.0
        self.flight_altitude = 10.0

        self.polygon_coords = []
        self.drone1_waypoints = []
        self.drone2_waypoints = []
        self.mission_active = False

        self.setup_ros_interface()
        self.parse_kml_and_generate_waypoints()

        rospy.loginfo("Dual Drone Lawnmower Node Initialized")

    def setup_ros_interface(self):
        rospy.Subscriber('/mavros1/state', State, self.drone1_state_callback)
        rospy.Subscriber('/mavros2/state', State, self.drone2_state_callback)
        rospy.Subscriber('/mavros1/global_position/global', NavSatFix, self.drone1_gps_callback)
        rospy.Subscriber('/mavros2/global_position/global', NavSatFix, self.drone2_gps_callback)
        rospy.Subscriber('/mavros1/local_position/pose', PoseStamped, self.drone1_local_pos_callback)
        rospy.Subscriber('/mavros2/local_position/pose', PoseStamped, self.drone2_local_pos_callback)

        self.drone1_pos_pub = rospy.Publisher('/mavros1/setpoint_position/local', PoseStamped, queue_size=10)
        self.drone2_pos_pub = rospy.Publisher('/mavros2/setpoint_position/local', PoseStamped, queue_size=10)
        self.geotag_pub = rospy.Publisher('/geotag_data', String, queue_size=10)

        self.drone1_arming_client = rospy.ServiceProxy('/mavros1/cmd/arming', CommandBool)
        self.drone2_arming_client = rospy.ServiceProxy('/mavros2/cmd/arming', CommandBool)
        self.drone1_set_mode_client = rospy.ServiceProxy('/mavros1/set_mode', SetMode)
        self.drone2_set_mode_client = rospy.ServiceProxy('/mavros2/set_mode', SetMode)

    def drone1_state_callback(self, msg):
        self.drone_states['drone1']['state'] = msg

    def drone2_state_callback(self, msg):
        self.drone_states['drone2']['state'] = msg

    def drone1_gps_callback(self, msg):
        self.drone_states['drone1']['position'] = msg

    def drone2_gps_callback(self, msg):
        self.drone_states['drone2']['position'] = msg

    def drone1_local_pos_callback(self, msg):
        self.drone_states['drone1']['local_pos'] = msg

    def drone2_local_pos_callback(self, msg):
        self.drone_states['drone2']['local_pos'] = msg

    

def parse_kml_and_generate_waypoints(self):
    try:
        kml_path = "/root/catkin_ws/src/geotag/scripts/farm_iitindore.kml"  # Your actual KML path
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
        mid_point = len(all_waypoints) // 2
        self.drone1_waypoints = all_waypoints[:mid_point]
        self.drone2_waypoints = all_waypoints[mid_point:]

    def calculate_distance(self, pos1, pos2):
        return geodesic((pos1[0], pos1[1]), (pos2[0], pos2[1])).meters

    def should_place_geotag(self, drone_id, current_pos):
        if not self.geotags[drone_id]:
            return True
        last_geotag = self.geotags[drone_id][-1]
        last_pos = (last_geotag['latitude'], last_geotag['longitude'])
        return self.calculate_distance(last_pos, (current_pos.latitude, current_pos.longitude)) >= self.geotagging_distance

    def place_geotag(self, drone_id, position):
        geotag = {
            'drone_id': drone_id,
            'latitude': position.latitude,
            'longitude': position.longitude,
            'altitude': position.altitude,
            'timestamp': rospy.Time.now().to_sec()
        }
        self.geotags[drone_id].append(geotag)
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

    def execute_drone_mission(self, drone_id):
        waypoints = self.drone1_waypoints if drone_id == 'drone1' else self.drone2_waypoints
        publisher = self.drone1_pos_pub if drone_id == 'drone1' else self.drone2_pos_pub
        rate = rospy.Rate(10)
        for i, (lat, lon) in enumerate(waypoints):
            if not self.mission_active:
                break
            target_pose = self.create_pose_stamped(lat, lon)
            rospy.loginfo(f"{drone_id} navigating to waypoint {i+1}/{len(waypoints)}")
            waypoint_reached = False
            start_time = time.time()
            while not waypoint_reached and self.mission_active:
                publisher.publish(target_pose)
                current_pos = self.drone_states[drone_id]['position']
                if current_pos.latitude != 0 and current_pos.longitude != 0:
                    distance = self.calculate_distance((current_pos.latitude, current_pos.longitude), (lat, lon))
                    if distance < 2.0:
                        waypoint_reached = True
                        if self.should_place_geotag(drone_id, current_pos):
                            self.place_geotag(drone_id, current_pos)
                if time.time() - start_time > 60:
                    rospy.logwarn(f"{drone_id} timeout on waypoint {i+1}, skipping.")
                    break
                rate.sleep()
        rospy.loginfo(f"{drone_id} mission completed")

    def save_geotags_to_json(self):
        output_data = {
            'mission_info': {
                'total_geotags': len(self.geotags['drone1']) + len(self.geotags['drone2']),
                'drone1_geotags': len(self.geotags['drone1']),
                'drone2_geotags': len(self.geotags['drone2']),
                'geotagging_distance': self.geotagging_distance,
                'timestamp': rospy.Time.now().to_sec()
            },
            'geotags': self.geotags
        }
        filename = f"geotags_mission_{int(rospy.Time.now().to_sec())}.json"
        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=2)
        rospy.loginfo(f"Geotags saved to {filename}")

    def prepare_drone(self, drone_id):
        set_mode = self.drone1_set_mode_client if drone_id == 'drone1' else self.drone2_set_mode_client
        arm = self.drone1_arming_client if drone_id == 'drone1' else self.drone2_arming_client
        for _ in range(5):
            if set_mode(0, "OFFBOARD").mode_sent:
                rospy.loginfo(f"{drone_id} OFFBOARD mode set")
                break
            rospy.sleep(1)
        for _ in range(5):
            if arm(True).success:
                rospy.loginfo(f"{drone_id} armed")
                break
            rospy.sleep(1)

    def execute_mission(self):
        if not self.drone1_waypoints or not self.drone2_waypoints:
            rospy.logerr("No waypoints generated")
            return
        self.mission_active = True
        rospy.loginfo("Starting lawnmower mission")
        t1 = threading.Thread(target=self.execute_drone_mission, args=('drone1',))
        t2 = threading.Thread(target=self.execute_drone_mission, args=('drone2',))
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        self.mission_active = False
        self.save_geotags_to_json()

    def start_mission(self):
        rospy.loginfo("Waiting for drone connections...")
        while not rospy.is_shutdown():
            if self.drone_states['drone1']['state'].connected and self.drone_states['drone2']['state'].connected:
                break
            rospy.sleep(1)
        self.prepare_drone('drone1')
        self.prepare_drone('drone2')
        self.execute_mission()

def main():
    try:
        node = DualDroneLawnmowerNode()
        mission_thread = threading.Thread(target=node.start_mission)
        mission_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")

if __name__ == '__main__':
    main()
