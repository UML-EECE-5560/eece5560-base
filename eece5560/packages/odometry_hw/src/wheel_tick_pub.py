#!/usr/bin/env python3
import csv
import rospy
from odometry_hw.msg import DistWheel


if __name__ == "__main__":
    rospy.init_node('wheel_tick_pub', anonymous=True)
    pub = rospy.Publisher("dist_wheel", DistWheel queue_size=10)
    rospy.sleep(5)
    rate = rospy.Rate(10) # 10hz
    csv_file_name = "wheel_ticks.csv"
    if rospy.has_param("/odom_csv_file_name"):
        csv_file_name = rospy.get_param("/odom_csv_file_name")
    with open(csv_file_name, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        i = 0
        for row in csv_reader:
            ticks_left = float(row[0])
            ticks_right = float(row[1])
            ticks = DistWheel()
            ticks.dist_wheel_left = ticks_left
            ticks.dist_wheel_right = ticks_right
            rospy.loginfo("Wheel Dist: left=%f, right=%f",ticks_left,ticks_right)
            pub.publish(ticks)
            rate.sleep()
    
