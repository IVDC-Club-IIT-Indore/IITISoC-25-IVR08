#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode, CommandBool
from mavros_msgs.msg import Waypoint
import json
import os
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import split
from geographiclib.geodesic import Geodesic
from datetime import datetime
import rospkg

DEG_TO_M = 111139  # Degrees to meters approximation (latitude only)
SPACING = 25  # meters between geotags and lawnmower lines
ALTITUDE = 20  # desired waypoint altitude in meters

class DiagnosticMultiDroneGeotagger:
    def __init__(self):
        rospy.init_node('multi_drone_geotagger', anonymous=True)

        self.drone_namespaces = ['uav1', 'uav2']
        self.positions = {ns: None for ns in self.drone_namespaces}
        self.states = {ns: None for ns in self.drone_namespaces}
        self.last_tags = {ns: None for ns in self.drone_namespaces}
        self.sector_counters = {ns: 1 for ns in self.drone_namespaces}

        self.geotags = []

        # Initialize subscribers with diagnostics
        rospy.loginfo("Setting up subscribers...")
        for ns in self.drone_namespaces:
            rospy.loginfo(f"Creating subscribers for {ns}")
            rospy.Subscriber(f'/{ns}/mavros/global_position/global', NavSatFix, 
                           self.make_gps_callback(ns))
            rospy.Subscriber(f'/{ns}/mavros/state', State, 
                           self.make_state_callback(ns))

        # Check if topics exist
        self.check_topics()

        # Load geofence and create paths
        self.load_geofence()
        self.assign_zones_by_area()
        self.create_lawnmower_paths()

        # Check services availability
        self.check_services()

        # Start with just monitoring - no waypoint pushing yet
        rospy.loginfo("Starting in monitoring mode...")
        rospy.loginfo("Will start geotagging when drones are connected and armed")

        # Start timers
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_geotags)
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self.print_detailed_status)
        
        rospy.loginfo("Diagnostic geotagger initialized - monitoring drone status")

    def check_topics(self):
        """Check if required topics are being published"""
        rospy.loginfo("Checking topic availability...")
        
        all_topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in all_topics]
        
        for ns in self.drone_namespaces:
            gps_topic = f'/{ns}/mavros/global_position/global'
            state_topic = f'/{ns}/mavros/state'
            
            if gps_topic in topic_names:
                rospy.loginfo(f"✓ {gps_topic} topic found")
            else:
                rospy.logwarn(f"✗ {gps_topic} topic NOT found")
                
            if state_topic in topic_names:
                rospy.loginfo(f"✓ {state_topic} topic found")
            else:
                rospy.logwarn(f"✗ {state_topic} topic NOT found")

    def check_services(self):
        """Check if required services are available"""
        rospy.loginfo("Checking service availability...")
        
        for ns in self.drone_namespaces:
            services_to_check = [
                f'/{ns}/mavros/mission/clear',
                f'/{ns}/mavros/mission/push',
                f'/{ns}/mavros/set_mode',
                f'/{ns}/mavros/cmd/arming'
            ]
            
            for service in services_to_check:
                try:
                    rospy.wait_for_service(service, timeout=1.0)
                    rospy.loginfo(f"✓ {service} service available")
                except rospy.ROSException:
                    rospy.logwarn(f"✗ {service} service NOT available")

    def make_gps_callback(self, ns):
        def callback(msg):
            if msg.status.status >= 0:  # Check if GPS fix is valid
                self.positions[ns] = msg
                if self.positions[ns] is None:  # First GPS message
                    rospy.loginfo(f"[{ns}] First GPS fix received")
        return callback

    def make_state_callback(self, ns):
        def callback(msg):
            old_state = self.states[ns]
            self.states[ns] = msg
            
            # Log state changes
            if old_state is None:
                rospy.loginfo(f"[{ns}] First state message received")
            elif (old_state.connected != msg.connected or 
                  old_state.armed != msg.armed or 
                  old_state.mode != msg.mode):
                rospy.loginfo(f"[{ns}] State changed - Connected: {msg.connected}, "
                             f"Armed: {msg.armed}, Mode: '{msg.mode}'")
        return callback

    def load_geofence(self):
        """Load geofence - simplified for testing"""
        rospy.loginfo("Creating simple test geofence...")
        
        # Use your actual GPS coordinates from the log (22.520027, 75.920000)
        center_lat = 22.520027
        center_lon = 75.920000
        
        # Create a small square around the current position (100m x 100m)
        offset = 0.0005  # roughly 50m in degrees
        
        default_coords = [
            (center_lon - offset, center_lat - offset),  # SW
            (center_lon + offset, center_lat - offset),  # SE  
            (center_lon + offset, center_lat + offset),  # NE
            (center_lon - offset, center_lat + offset),  # NW
            (center_lon - offset, center_lat - offset)   # Close
        ]
        
        self.full_poly = Polygon(default_coords)
        rospy.loginfo(f"Created test geofence around ({center_lat}, {center_lon})")

    def assign_zones_by_area(self):
        """Split the polygon into two zones"""
        minx, miny, maxx, maxy = self.full_poly.bounds
        mid_x = (minx + maxx) / 2
        
        zone1_coords = [(minx, miny), (mid_x, miny), (mid_x, maxy), (minx, maxy)]
        zone2_coords = [(mid_x, miny), (maxx, miny), (maxx, maxy), (mid_x, maxy)]
        
        self.drone_zones = {
            'uav1': Polygon(zone1_coords),
            'uav2': Polygon(zone2_coords)
        }
        
        rospy.loginfo(f"Zone areas - UAV1: {self.drone_zones['uav1'].area:.8f}, "
                     f"UAV2: {self.drone_zones['uav2'].area:.8f}")

    def create_lawnmower_paths(self):
        """Create simple test paths"""
        self.paths = {}
        
        for ns in self.drone_namespaces:
            zone = self.drone_zones[ns]
            minx, miny, maxx, maxy = zone.bounds
            
            # Create a simple 4-point path for testing
            path = [
                (minx + 0.0001, miny + 0.0001),
                (maxx - 0.0001, miny + 0.0001),
                (maxx - 0.0001, maxy - 0.0001),
                (minx + 0.0001, maxy - 0.0001)
            ]
            
            self.paths[ns] = path
            rospy.loginfo(f"Created simple test path for {ns} with {len(path)} waypoints")

    def print_detailed_status(self, event):
        """Print detailed status information"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("DETAILED DRONE STATUS")
        rospy.loginfo("=" * 50)
        
        for ns in self.drone_namespaces:
            rospy.loginfo(f"--- {ns.upper()} ---")
            
            state = self.states[ns]
            pos = self.positions[ns]
            
            if state:
                rospy.loginfo(f"Connected: {state.connected}")
                rospy.loginfo(f"Armed: {state.armed}")
                rospy.loginfo(f"Mode: '{state.mode}'")
                rospy.loginfo(f"Guided: {state.guided}")
                rospy.loginfo(f"Manual input: {state.manual_input}")
                rospy.loginfo(f"System status: {state.system_status}")
            else:
                rospy.logwarn("No state data received!")
                
            if pos:
                rospy.loginfo(f"GPS Status: {pos.status.status} (service: {pos.status.service})")
                rospy.loginfo(f"Position: ({pos.latitude:.6f}, {pos.longitude:.6f}, {pos.altitude:.1f}m)")
                rospy.loginfo(f"Covariance type: {pos.position_covariance_type}")
                
                # Check if in zone
                point = Point(pos.longitude, pos.latitude)
                in_zone = self.drone_zones[ns].contains(point)
                rospy.loginfo(f"In assigned zone: {in_zone}")
            else:
                rospy.logwarn("No GPS data received!")
            
            rospy.loginfo("")
                
        rospy.loginfo(f"Total geotags collected: {len(self.geotags)}")
        rospy.loginfo("=" * 50)
        
        # Try to push waypoints if drones are connected
        self.try_waypoint_operations()

    def try_waypoint_operations(self):
        """Try waypoint operations only when drones are connected"""
        for ns in self.drone_namespaces:
            state = self.states[ns]
            if state and state.connected:
                rospy.loginfo(f"[{ns}] Drone is connected - attempting waypoint operations")
                self.try_push_waypoints(ns)
                self.try_set_mode(ns)

    def try_push_waypoints(self, ns):
        """Try to push waypoints for a specific drone"""
        try:
            rospy.wait_for_service(f'/{ns}/mavros/mission/clear', timeout=2.0)
            rospy.wait_for_service(f'/{ns}/mavros/mission/push', timeout=2.0)

            clear_srv = rospy.ServiceProxy(f'/{ns}/mavros/mission/clear', WaypointClear)
            push_srv = rospy.ServiceProxy(f'/{ns}/mavros/mission/push', WaypointPush)

            # Try to clear waypoints
            rospy.loginfo(f"[{ns}] Attempting to clear waypoints...")
            clear_result = clear_srv()
            rospy.loginfo(f"[{ns}] Clear result: {clear_result.success}")

            if clear_result.success:
                # Create simple waypoint
                waypoints = []
                coord = self.paths[ns][0]  # Just one waypoint for testing
                
                wp = Waypoint()
                wp.frame = 3
                wp.command = 16
                wp.is_current = True
                wp.autocontinue = True
                wp.param1 = 1.0
                wp.param2 = 5.0
                wp.param3 = 0.0
                wp.param4 = float('nan')
                wp.x_lat = float(coord[1])
                wp.y_long = float(coord[0])
                wp.z_alt = float(ALTITUDE)
                waypoints.append(wp)

                rospy.loginfo(f"[{ns}] Pushing 1 test waypoint...")
                push_result = push_srv(start_index=0, waypoints=waypoints)
                rospy.loginfo(f"[{ns}] Push result: success={push_result.success}, "
                             f"transferred={push_result.wp_transfered}")

        except Exception as e:
            rospy.loginfo(f"[{ns}] Waypoint operation failed: {e}")

    def try_set_mode(self, ns):
        """Try to set flight mode for a specific drone"""
        try:
            rospy.wait_for_service(f'/{ns}/mavros/set_mode', timeout=2.0)
            mode_srv = rospy.ServiceProxy(f'/{ns}/mavros/set_mode', SetMode)
            
            # Try GUIDED mode only
            rospy.loginfo(f"[{ns}] Attempting to set GUIDED mode...")
            mode_resp = mode_srv(0, 'GUIDED')
            rospy.loginfo(f"[{ns}] Mode set result: {mode_resp.mode_sent}")
            
        except Exception as e:
            rospy.loginfo(f"[{ns}] Mode setting failed: {e}")

    def check_geotags(self, event):
        """Check if drones should place geotags - only when connected and armed"""
        for ns in self.drone_namespaces:
            position = self.positions[ns]
            state = self.states[ns]
            
            # Only geotag if drone is connected, armed, and has GPS
            if (position is None or state is None or 
                not state.connected or not state.armed):
                continue

            try:
                point = Point(position.longitude, position.latitude)
                
                # Check if drone is in its assigned zone
                if not self.drone_zones[ns].contains(point):
                    continue

                # Check spacing from last geotag
                last_tag = self.last_tags[ns]
                if last_tag is not None:
                    dist = Geodesic.WGS84.Inverse(
                        last_tag.y, last_tag.x,
                        position.latitude, position.longitude
                    )['s12']
                    if dist < SPACING:
                        continue

                # Create new geotag
                tag = {
                    "sector_id": self.sector_counters[ns],
                    "drone_id": 1 if ns == 'uav1' else 2,
                    "center_latitude": position.latitude,
                    "center_longitude": position.longitude,
                    "altitude": position.altitude,
                    "timestamp": datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S"),
                    "drone_namespace": ns
                }
                
                self.geotags.append(tag)
                self.sector_counters[ns] += 1
                self.last_tags[ns] = Point(position.longitude, position.latitude)

                rospy.loginfo(f"[{ns}] 🏷️  GEOTAG #{tag['sector_id']} placed at "
                             f"({tag['center_latitude']:.6f}, {tag['center_longitude']:.6f})")

                # Save geotags to file
                self.save_geotags()
                
            except Exception as e:
                rospy.logwarn(f"Error processing geotag for {ns}: {e}")

    def save_geotags(self):
        """Save all geotags to JSON file"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            out_dir = os.path.join(script_dir, '..', 'geotags')
            os.makedirs(out_dir, exist_ok=True)
            
            output_file = os.path.join(out_dir, 'all_geotags.json')
            with open(output_file, 'w') as f:
                json.dump(self.geotags, f, indent=2)
                
        except Exception as e:
            rospy.logwarn(f"Failed to save geotags: {e}")

if __name__ == '__main__':
    try:
        geotagger = DiagnosticMultiDroneGeotagger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Diagnostic geotagger interrupted")
    except Exception as e:
        rospy.logerr(f"Diagnostic geotagger failed: {e}")
        raise
