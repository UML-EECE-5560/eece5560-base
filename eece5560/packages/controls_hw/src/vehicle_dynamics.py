#!/usr/bin/env python3
import rospy
from numpy import arange,sign
from random import random
from std_msgs.msg import Float32

class VehicleDynamics:
    def __init__(self, m, mu, g, area, c, rho, v0, p0, noise_mag):
        self.m = m # mass, kg
        self.mu = mu # friction coefficient
        self.g = g # gravity, m/s^2
        self.area = area # cross section, m^2
        self.c = c # drag coefficient
        self.rho = rho # air density, kg/m^3
        self.xd = v0 # initial velocity, m/s
        self.x = p0 # initial position, m
        self.noise_mag = noise_mag # magnitude of noise
        self.control = 0 # control input, m/s^2
        
    def iterate(self, dt):
        # a = engine acceleration - friction - drag
        self.xdd = self.control - \
            sign(self.xd)*(self.mu*self.g + \
                (self.c*self.rho*self.area*self.xd*self.xd)/(2*self.m)) + \
            self.noise_mag*random()
        self.xd += self.xdd*dt
        self.x += self.xd*dt
        return self.x, self.xd, self.xdd
    def update_control(self, control):
        self.control = control.data

if __name__ == '__main__':
    try:
        desired = 30
        rospy.init_node('vehicle_dynamics')
        vd = VehicleDynamics(m=2000, mu=0.3, g=9.8, area=10, c=0.7, rho=1.3, v0=20, p0=0, noise_mag=0)
        rospy.Subscriber("control_input", Float32, vd.update_control)
        pub_xd = rospy.Publisher("velocity", Float32, queue_size=10)
        pub_x = rospy.Publisher("position", Float32, queue_size=10)
        pub_d = rospy.Publisher("desired", Float32, queue_size=10)
        pub_error = rospy.Publisher("error", Float32, queue_size=10)
        
        # wait ~5 seconds to start so rqt_plot has a chance to get going
        # run vehicle dynamics for ~10 seconds
        # exit
        rate = rospy.Rate(1000)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.has_param("controller_ready"):
                if rospy.get_param("controller_ready") != "true":
                    rospy.logwarn("Waiting for controller_ready to be true")
                    rate.sleep()
                    start_time = rospy.get_time()
                    continue
            else:
                rospy.logwarn("Waiting for controller_ready to be true")
                rate.sleep()
                start_time = rospy.get_time()
                continue
            time_elapsed = rospy.get_time() - start_time
            # quit after 30 sec of running
            if time_elapsed > 30:
                rospy.set_param("controller_ready", "false")
                exit()
            vd.iterate(0.001)
            pub_xd.publish(vd.xd)
            pub_x.publish(vd.x)
            pub_d.publish(desired)
            pub_error.publish(desired - vd.x)
            #rospy.logwarn("v=%f, x=%f, e=%f" % (vd.xd, vd.x, desired-vd.x))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
