#!/usr/bin/env python3

import rospy
import json
import numpy as np
from datetime import datetime
from sensor_msgs.msg import NavSatFix

DEG_TO_M = 111139  # Approx conversion factor degrees to meters (at Equator)

class RealTimeGeotagger:
    def __init__(self):
        self.last_position = None
        self.geotags = []
        self.sector_id = 1
        self.drone_id = rospy.get_param("~drone_id", 1)
        self.output_file = rospy.get_param("~output_file", "real_geotags.json")
        self.spacing_m = rospy.get_param("~spacing_m", 25)

        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        rospy.loginfo("✅ Real-time Geotagger initialized.")

    def gps_callback(self, msg):
        if msg.status.status < 0:
            return  # No valid GPS fix yet
        current_position = np.array([msg.latitude, msg.longitude])

        if self.last_position is None:
            self._add_geotag(current_position)
            return

        dist_m = np.linalg.norm((current_position - self.last_position) * DEG_TO_M)
        if dist_m >= self.spacing_m:
            self._add_geotag(current_position)

    def _add_geotag(self, position):
        geotag = {
            "sector_id": self.sector_id,
            "drone_id": self.drone_id,
            "center_latitude": position[0],
            "center_longitude": position[1],
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.geotags.append(geotag)
        self.last_position = position
        self.sector_id += 1

        rospy.loginfo("✅ Geotag {} added at [{}, {}]".format(geotag["sector_id"], position[0], position[1]))
        self._save_json()

    def _save_json(self):
        with open(self.output_file, 'w') as f:
            json.dump(self.geotags, f, indent=2)

if __name__ == "__main__":
    rospy.init_node("real_time_geotagger")
    RealTimeGeotagger()
    rospy.spin()
