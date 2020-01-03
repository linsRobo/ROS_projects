#!/usr/bin/env python

import RPi.GPIO as GPIO
import time


from geometry_msgs.msg import Twist

import rospy
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
L1 =21 
L2 =13 
R1 =19
R2 =26
GPIO.setup(L1,GPIO.OUT)
GPIO.setup(L2,GPIO.OUT) 
GPIO.setup(R1,GPIO.OUT) 
GPIO.setup(R2,GPIO.OUT)

def callback(msg):
    velocity_message = Twist()


    rospy.loginfo("Received a /cmd_vel message!")

    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y$

    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angula$
    if(msg.linear.x == 0.5):

        rospy.loginfo("forward")
        forward()
    elif(msg.linear.x == -0.5):
        rospy.loginfo("backward")
        backward()
    elif(msg.angular.z == -1.0):
        rospy.loginfo("right")
    elif(msg.angular.z == 1.0):
        rospy.loginfo("left")
        left()
    else:
        rospy.loginfo("stop")
        stop_now()
def listener():

    rospy.init_node('cmd_vel_listener')

    rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.spin()

def forward():
        GPIO.output(L1,GPIO.HIGH) 
        GPIO.output(L2,GPIO.LOW) 
        GPIO.output(R1,GPIO.HIGH)
        GPIO.output(R2,GPIO.LOW)
def backward(): 
        GPIO.output(L1,GPIO.LOW) 
        GPIO.output(L2,GPIO.HIGH) 
        GPIO.output(R1,GPIO.LOW) 
        GPIO.output(R2,GPIO.HIGH) 
def left(): 
        GPIO.output(L1,GPIO.HIGH) 
        GPIO.output(L2,GPIO.LOW) 
        GPIO.output(R1,GPIO.LOW) 
        GPIO.output(R2,GPIO.HIGH) 
def right(): 
        GPIO.output(L1,GPIO.LOW) 
        GPIO.output(L2,GPIO.HIGH) 
        GPIO.output(R1,GPIO.HIGH) 
        GPIO.output(R2,GPIO.LOW)
def stop_now(): 
        GPIO.output(L1,GPIO.LOW) 
        GPIO.output(L2,GPIO.LOW) 
        GPIO.output(R1,GPIO.LOW) 
        GPIO.output(R2,GPIO.LOW) 
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
        stop_now()
        GPIO.cleanup()

