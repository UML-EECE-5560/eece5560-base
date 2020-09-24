#! /usr/bin/env python3
# source http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import example_action_server.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    print("starting client")
    client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)

    print("waiting for server")
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    print("sending goal")
    # Creates a goal to send to the action server.
    goal = example_action_server.msg.FibonacciGoal(order=10)

    # Sends the goal to the action server.
    client.send_goal(goal)

    print("waiting for result")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
