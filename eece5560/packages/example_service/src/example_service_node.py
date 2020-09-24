#!/usr/bin/env python3

import rospy
from example_service.srv import Fibonacci, FibonacciResponse


class FibonacciService:
    def __init__(self):
        self.calc_fibonacci = rospy.Service('calc_fibonacci', Fibonacci, self.handle_calc_fibonacci)
        
    def handle_calc_fibonacci(self,req):
        r = rospy.Rate(1)
        sequence = [0,1]
        r.sleep()
        if req.order == 1:
            return FibonacciResponse(sequence[0:1])
        r.sleep()
        for i in range(1,req.order):
            sequence.append(sequence[i] + sequence[i-1])
            r.sleep()
        return FibonacciResponse(sequence)

if __name__ == "__main__":
    rospy.init_node('fibonacci_service_node')
    FibonacciService()
    rospy.spin()
    
