#!/usr/bin/env python3
import rospy
from custom_msgs.msg import IMUData
import ctypes
import os
import threading
import time

from ctypes import Structure, c_float

class EulerAngle(Structure):
    _fields_ = [
        ("roll", c_float),
        ("pitch", c_float),
        ("yaw", c_float)
    ]

class Vector3f(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float)
    ]
   

# Load the shared library
lib = ctypes.CDLL(os.path.abspath("/home/ubuntu/catkin_workspace/src/telebot_controller/src/IMU_Code/IMU_Reader.so"))

# Define the return types for the getter functions
lib.get_orientation.restype = EulerAngle
lib.get_displacement.restype = Vector3f
lib.get_status.restype = bool

# Define a flag to signal the thread to stop
stop_thread = False

# Initialize the ROS node
rospy.init_node('right_imu_publisher')

# Create a publisher for the IMUData messages
pub = rospy.Publisher('/right_imu_coordinates', IMUData, queue_size=20)


# Define a function to run in a separate thread
def run_c_program():
    # Call the main function with the device name
    dev_name = "/dev/serial0"  # Example device name, change it to match your setup
    lib.main(2, (ctypes.c_char_p * 2)(b"IMU_Reader.so", dev_name.encode()))

# Create and start the thread
c_program_thread = threading.Thread(target=run_c_program)
c_program_thread.start()

#mp3
rospy.loginfo("Hold Your Hand for 5 seconds in front of you for Calibration")
os.system('cvlc /home/ubuntu/Downloads/publisher1.mp3 vlc://quit')

time.sleep(5)

#mp3
rospy.loginfo("Calibration Complete")
os.system('cvlc /home/ubuntu/Downloads/publisher2.mp3 vlc://quit')

# Function to get the current angle and position values
def get_data():
    return lib.get_orientation(), lib.get_displacement()

time.sleep(2)

#mp3    
rospy.loginfo("In 2 Seconds, Orient your hand to Adjust Position")
os.system('cvlc /home/ubuntu/Downloads/publisher3.mp3 vlc://quit')
time.sleep(2)

# Main loop
while not stop_thread and not rospy.is_shutdown():   
    # Access data
    orientation, displacement = get_data()
    rospy.loginfo(f"Current Angle: ({orientation.roll:.2f}, {orientation.pitch:.2f}, {orientation.yaw:.2f})")

    #Only publish after calibration
    #Create IMU_Publisher
    imu_data = IMUData()
    # Publish the data as an IMUData message
    imu_data.roll = round(orientation.roll, 2)
    imu_data.pitch = round(-(orientation.pitch), 2)
    imu_data.yaw = round(orientation.yaw, 2) 
    imu_data.x = round(0.577, 2)
    imu_data.y = round((0), 2) #X and Y axis are flipped
    imu_data.z = round(0.423, 2)
    imu_data.blending_radius = 0.1
    pub.publish(imu_data)
    rospy.loginfo("I am publishing")

    time.sleep(1)
	
# Wait for the thread to finish
c_program_thread.join()
