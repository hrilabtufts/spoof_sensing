#!/usr/bin/env python

import rospy
from rospy.rostime import Duration
from rosgraph_msgs.msg import Clock

class ClockPublisher:
    def __init__(self):
        rospy.init_node('provide_rostime')
        self.pub = rospy.Publisher('/provided_rostime', Clock, queue_size=1)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.cycle()
            rate.sleep()
        #self.timer = rospy.Timer(Duration(1/100), self.cycle)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.pub.unregister()

    def cycle(self):
        now = rospy.Time().now()
        self.pub.publish(Clock(now))

if __name__ == '__main__':
    ClockPublisher()
    rospy.loginfo("Publishing time")
    rospy.spin()
