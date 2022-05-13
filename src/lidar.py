#!/usr/bin/env python3

"""
quick 'n dirty node to open a topic on /scan, useful only really for
debugging/developing your software architecture before dropping the real nodes
in.
"""
import rospy
from math import ceil
from sensor_msgs.msg import LaserScan

class SpoofLidar:
    def __init__(self):
        rospy.init_node("empty_laser_publisher", anonymous=False)
        pub = rospy.Publisher("scan", LaserScan, queue_size=10)
        msg = LaserScan()
        msg.header.frame_id = rospy.get_param('laser_frame_id', 'laser')
        msg.angle_min = 0
        msg.angle_max = 2
        msg.angle_increment = 0.1
        msg.range_min = 0
        msg.range_max = 10
        msg.scan_time = 0

        arr = []
        for _ in range(0, ceil((msg.angle_max - msg.angle_min) / msg.angle_increment)):
            arr.append(10)

        msg.ranges = arr

        rospy.loginfo("Running!")

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time().now()
            pub.publish(msg)
            rate.sleep()

        rospy.loginfo("All done!")

if __name__ == '__main__':
    SpoofLidar()
