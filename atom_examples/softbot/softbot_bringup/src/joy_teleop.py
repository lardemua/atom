#!/usr/bin/env python3

# Imports
from functools import partial

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

#! Max velocitiy for the robot, it only keeps a perfectly straight line up to 0.2, but thats too slow 
MAX_VEL = rospy.get_param("/max_velocity",0.4) # Not a private param

def rangeScaling(value,x1,y1,x2,y2):
    # x1,x2 is the first range [x1,x2], x2,y2 is the range to scale to [x2,y2], value is the number to be scaled
    percentage_of_range = ((value - x1)/(y1-x1))
    scaled_value = x2 + percentage_of_range*(y2-x2)

    return scaled_value 



# Direction Callback Function
def messageReceivedCallbackJoy(message, **kwargs):

    global MAX_VEL
    angular = message.axes[0]
    # The R2 trigger rest's at 1, and goes up to -1 as its pressed
    forward_axe_value = -round(message.axes[4],1)
    reverse_axe_value = -round(message.axes[5],1)

    if(forward_axe_value > 0):# Only triggers if halfway pressed
        linear = rangeScaling(forward_axe_value,0 ,1 ,0 ,MAX_VEL)
        # rospy.loginfo("Going forward")
    elif(reverse_axe_value >0):
        linear = -1*rangeScaling(reverse_axe_value,0 ,1 ,0 ,MAX_VEL)
        # rospy.loginfo("Going reverse")

    else:
        linear = 0

    # TODO implement verbose mode with an argument
    # if verbose:
    #     rospy.loginfo("Linear is " + str(linear)) 
    #     rospy.loginfo("Angular is " + str(angular)) 

    twist_cmd = Twist()  # Message type twist
    twist_cmd.linear.x = linear
    twist_cmd.angular.z = angular
    kwargs["twist_publisher"].publish(twist_cmd)


def main():

    global MAX_VEL
    # Defining variables
    kwargs = dict(twist_publisher=None)

    # Initiating node
    rospy.init_node('controller_joy', anonymous=False)

    rospy.loginfo("Max velocity is "+ str(MAX_VEL))
    # Get parameters
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    joy_topic = rospy.get_param('~joy_topic', '/joy')


    # Define publishers and subscribers
    kwargs["twist_publisher"] = rospy.Publisher(twist_cmd_topic, Twist, queue_size=10)
    
    # Define partials
    messageReceivedCallbackJoy_part = partial(messageReceivedCallbackJoy, **kwargs)
    rospy.Subscriber(joy_topic, Joy, messageReceivedCallbackJoy_part)
    rospy.spin()


if __name__ == '__main__':
    main()
