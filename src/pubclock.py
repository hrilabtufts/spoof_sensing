#!/usr/bin/env python

import rospy
from rospy.rostime import Duration
from std_msgs.msg import Time

class ClockPublisher:
    def __init__(self):
        rospy.init_node('provide_rostime')
        self.pub = rospy.Publisher('/provided_rostime', Time, queue_size=1)
        self.timer = rospy.Timer(Duration(1/100), self.cycle)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.timer.shutdown()
        self.pub.unregister()

    def cycle(self, event):
        del event
        self.pub.publish(Time(rospy.Time.now()))

if __name__ == '__main__':
    ClockPublisher()
    rospy.loginfo("Publishing time")
    rospy.spin()
