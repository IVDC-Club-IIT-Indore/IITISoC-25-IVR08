#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

def publish_setpoints():
    pub = rospy.Publisher('/uav1/mavros1/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node('setpoint_publisher_node')

    rate = rospy.Rate(20)  # 20Hz
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2  # Target height (2 meters)

    # Send few setpoints before OFFBOARD
    for i in range(100):
        pose.header.stamp = rospy.Time.now()
        pub.publish(pose)
        rate.sleep()

def arm_and_offboard():
    rospy.wait_for_service('/uav1/mavros1/cmd/arming')
    rospy.wait_for_service('/uav1/mavros1/set_mode')

    arm_srv = rospy.ServiceProxy('/uav1/mavros1/cmd/arming', CommandBool)
    mode_srv = rospy.ServiceProxy('/uav1/mavros1/set_mode', SetMode)

    print("Setting mode to OFFBOARD...")
    mode_resp = mode_srv(0, 'OFFBOARD')
    print(mode_resp)

    print("Arming...")
    arm_resp = arm_srv(True)
    print(arm_resp)

if __name__ == '__main__':
    try:
        publish_setpoints()
        arm_and_offboard()
    except rospy.ROSInterruptException:
        pass
