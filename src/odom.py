#!/usr/bin/env python

from math import sin, cos

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class SpoofOdom:
    def __init__(self):
        rospy.init_node("odometry_publisher")

        odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

        odom_broadcaster = tf.TransformBroadcaster()

        self.vx = 0
        self.vy = 0
        self.vth = 0

        x = 0.0
        y = 0.0
        th = 0.0

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            delta_x = (self.vx * cos(th) - self.vy * sin(th)) * dt
            delta_y = (self.vx * sin(th) + self.vy * cos(th)) * dt
            delta_th = self.vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(  # type:ignore
                0, 0, th
            )

            # first, we'll publish the transform over tf
            base_link = rospy.get_param("base_link_frame_id", "base_link")
            odom_link = rospy.get_param("odom_link_frame_id", "odom_combined")
            odom_broadcaster.sendTransform(
                (x, y, 0.0), odom_quat, current_time, base_link, odom_link
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(
                Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth)
            )

            # publish the message
            odom_pub.publish(odom)

            last_time = current_time
            r.sleep()

    def cmd_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z


if __name__ == "__main__":
    SpoofOdom()
