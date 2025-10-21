#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np


def main():
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=1)
    rospy.init_node('draw_circle', anonymous=True)
    hz = 50
    dt = 1 / hz
    rate = rospy.Rate(hz)

    x0 = 0.4
    y0 = 0.0
    z0 = 0.2
    radius = 0.1
    rps = 0.5

    phi = 0
    rad_per_sec = 2 * np.pi * rps

    while not rospy.is_shutdown():
        x = x0 + radius * np.cos(phi)
        y = y0 + radius * np.sin(phi)
        z = z0

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'panda_link0'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 1
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 0

        pub.publish(msg)

        rate.sleep()
        phi += rad_per_sec * dt
        if phi > np.pi:
            phi -= 2 * np.pi


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
