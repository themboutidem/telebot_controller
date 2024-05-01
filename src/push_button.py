#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from subprocess import Popen, PIPE
from time import sleep

GPIO.setwarnings(False)  # Ignore warning for now
GPIO.setmode(GPIO.BCM)  # Use physical pin numbering
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Button1
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Button2

rospy.init_node('button_publisher')
pub = rospy.Publisher('/subscriber_mode', Float32, queue_size=10)

publisher_process = None
previous = 0

while not rospy.is_shutdown():
    if GPIO.input(23) == GPIO.HIGH:  # Button1
        rospy.loginfo("Button1 was pushed!")
        current = 1
        if current != previous:
            if publisher_process:
                publisher_process.terminate()
                rospy.loginfo("Terminated publisher_process!")
            pub.publish(1.0)
            rospy.loginfo("Button1 published!")
            previous = current
            rospy.loginfo("Previous set to 1")

    if GPIO.input(12) == GPIO.HIGH:  # Button2
        rospy.loginfo("Button2 was pushed!")
        current = 2
        if current != previous:
            if publisher_process:
                publisher_process.terminate()
                rospy.loginfo("Terminated publisher_process!")
            pub.publish(2.0)
            rospy.loginfo("Button2 published!")
            previous = current
            rospy.loginfo("Previous set to 2")

    sleep(0.1)  # Add a small delay to avoid busy-waiting
