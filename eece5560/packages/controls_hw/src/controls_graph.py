#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import matplotlib

class ControlsGraph:
    def __init__(self):
        self.xp_list = list()
        self.yp_list = list()
        self.xv_list = list()
        self.yv_list = list()
        self.xd_list = list()
        self.yd_list = list()
        self.start_time = 0
        rospy.Subscriber("position", Float32, self.pos_cb)
        rospy.Subscriber("velocity", Float32, self.vel_cb)
        rospy.Subscriber("desired", Float32, self.desired_cb)
    
    def start_clock(self):
        self.start_time = rospy.get_time()
    
    def pos_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xp_list.append(rospy.get_time()-self.start_time)
        self.yp_list.append(msg.data)
        
    def vel_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xv_list.append(rospy.get_time()-self.start_time)
        self.yv_list.append(msg.data)
        
    def desired_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xd_list.append(rospy.get_time()-self.start_time)
        self.yd_list.append(msg.data)




if __name__ == '__main__':
    try:
        rospy.init_node('controls_graph', anonymous=True)
        output_to_file = False
        if rospy.has_param('/output_to_file'):
            rospy.logwarn("Has output to file")
            if rospy.get_param('/output_to_file') == True or rospy.get_param('/output_to_file') == "true":
                output_to_file=True
        if rospy.has_param('/only_output_to_file'):
            rospy.logwarn("Has only output to file")
            if rospy.get_param('/only_output_to_file') == True or rospy.get_param('/only_output_to_file') == "true":
                rospy.logwarn("only outputting to PDF!")
                output_to_file=True
                matplotlib.use("pdf")
        folder = "."
        if rospy.has_param('output_folder'):
            folder = rospy.get_param('output_folder')
                                
        import matplotlib.pyplot as plt
        cg = ControlsGraph()
        rospy.set_param("graph_ready","true")
        rate = rospy.Rate(5) # 5hz
        while not rospy.is_shutdown():     
            plt.plot(cg.xp_list, cg.yp_list, 'b-')
            plt.plot(cg.xd_list, cg.yd_list, 'g-')
            plt.plot(cg.xv_list, cg.yv_list, 'r-')
#            plt.axis([0,30,0,100])
            plt.xlabel('Time (s)')
            plt.ylabel('Measured Value')
            plt.title('Vehicle Motion')
            plt.legend(('$x_d$ (m)', '$x$ (m)', '$\dot{x}$ (m/s)'))
            if output_to_file:
                plt.savefig(folder + "/output_plot.png")
            plt.pause(0.05)        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
