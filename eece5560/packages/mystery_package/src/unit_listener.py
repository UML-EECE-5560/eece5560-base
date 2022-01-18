#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class UnitListener:
    def __init__(self):
        rospy.Subscriber("output2", UnitsLabelled, self.callback)

    def callback(self, msg):
        rospy.loginfo("Unit Listener heard: " + str(msg.value) + " " + msg.units)
        

if __name__ == '__main__':
    rospy.init_node('unit listener')
    UnitListener()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

