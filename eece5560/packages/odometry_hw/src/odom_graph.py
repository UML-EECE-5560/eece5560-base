#!/usr/bin/env python3
import rospy
from odometry_hw.msg import Pose2D
import matplotlib

class OdomGraph:
    def __init__(self):
        self.x_list = list()
        self.y_list = list()
    
    def pose_cb(self,msg):
        self.x_list.append(msg.x)
        self.y_list.append(msg.y)


if __name__ == '__main__':
    try:
        rospy.init_node('odom_graph', anonymous=True)
        output_to_file = False
        if rospy.has_param('output_to_file'):
            if rospy.get_param('output_to_file') == True:
                output_to_file=True
        if rospy.has_param('only_output_to_file'):
            rospy.logwarn("only")
            matplotlib.use("pdf")
        import matplotlib.pyplot as plt
        og = OdomGraph()
        rospy.Subscriber("pose", Pose2D, og.pose_cb)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            plt.plot(og.x_list, og.y_list,'ro-',)
            plt.axis([-0.5,1.5,-1,1])
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('Vehicle Odometry')
            plt.savefig("output_plot.png")
            plt.pause(0.05)        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
