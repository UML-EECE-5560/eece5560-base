#!/usr/bin/env python3
import csv
import rospy
from odometry_hw.msg import DistWheel

def pattern_generator(i):
    if i < 2:
        return (0,0.0785) # left
    elif i >= 2 and i < 22:
        return (.1,.1) # up before U
    elif i >= 22 and i < 24:
        return (0.0785,0) # right
    elif i >= 24 and i < 25:
        return (0.1,0.1) # space before U
    elif i >= 25 and i < 27:
        return (0.0785,0) # right
    elif i >= 27 and i < 47:
        return (.1,.1) # down of U
    elif i >= 47 and i < 49:
        return (0,0.0785) # left
    elif i >= 49 and i < 60:
        return (.1,.1) # bottom of U
    elif i >= 60 and i < 62:
        return (0,0.0785) # left
    elif i >= 62 and i < 82:
        return (.1,.1) # up of U/M
    elif i >= 82 and i < 85:
        return (0.0785,0) # right 135
    elif i >= 85 and i < 95:
        return (.1,.1) # down diagonal
    elif i >= 95 and i < 97:
        return (0,0.0785) # left
    elif i >= 97 and i < 107:
        return (.1,.1) # up diagonal
    elif i >= 107 and i < 110:
        return (0.0785,0) # right 135
    elif i >= 110 and i < 130:
        return (.1,.1) # down for M/L    
    elif i >= 130 and i < 132:
        return (0,0.0785) # left
    elif i >= 132 and i < 145:
        return (.1,.1) # bottom of L
        
    return (0,0)

if __name__ == "__main__":
    rospy.init_node('wheel_tick_pub', anonymous=True)
    pub = rospy.Publisher("dist_wheel", DistWheel, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    for i in range(50):
        pub.publish(DistWheel(0,0))
        if rospy.is_shutdown():
            break
        rate.sleep()

    for i in range(160):
        ticks_left,ticks_right = pattern_generator(i)
        rospy.logwarn("left: %f right: %f" % (ticks_left,ticks_right))
        ticks = DistWheel()
        ticks.dist_wheel_left = ticks_left
        ticks.dist_wheel_right = ticks_right
        pub.publish(ticks)
        if rospy.is_shutdown():
            break
        rate.sleep()
    
