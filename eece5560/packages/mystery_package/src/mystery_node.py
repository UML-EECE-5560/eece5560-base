#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class MysteryNode:
    def __init__(self):
        rospy.Subscriber("input", Float32, self.callback)
        self.pub_raw = rospy.Publisher("output1", Float32, queue_size=10)
        self.pub_units = rospy.Publisher("output2", UnitsLabelled, queue_size=10)
        self.total = 0
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "meters"
        

    def callback(self, msg):
        self.total += msg.data
        self.pub_raw.publish(self.total)
        self.pub_msg.value = self.total
        self.pub_units.publish(self.pub_msg)
        

if __name__ == '__main__':
    rospy.init_node('mystery_node')
    MysteryNode()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

