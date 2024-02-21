#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import math

def float32_publisher():
	# Initialize the ROS node with a unique name
	rospy.init_node('move_arm', anonymous=True)

	# Create a publisher for the Float32 data on the '/fgripper_signal' topic
	pub = rospy.Publisher('/gripper_position', Float32, queue_size=10)

	# Set the loop rate (in Hz)
	rate = rospy.Rate(1)  # 1 Hz We should change this
    
	# create the spi bus
	spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
	
	# create the cs (chip select)
	cs = digitalio.DigitalInOut(board.D5)

	# create the mcp object
	mcp = MCP.MCP3008(spi, cs)

	# create an analog input channel on pin 0
	chan = AnalogIn(mcp, MCP.P0)

    # Main loop
	while not rospy.is_shutdown():

		# Read input from ADC Channel
		input_volt = chan.voltage
		
		#Floor input voltage to fraction of 3.3V
		input_volt = input_volt/3.3000
		
		#Floor input voltage to 2 decimal places
		input_volt = math.ceil(input_volt*100)/100
		

		try:
			# Create a Float32 message and set its data
			float32_msg = Float32()
			
			if(input_volt>0.5):
				# set value to 1 if voltage is grater than 1.6
				float_value = 1
				float32_msg.data = float_value
				# Publish the message
				pub.publish(float32_msg)
			else:
				float_value = 0
				float32_msg.data = float_value
				# Publish the message
				pub.publish(float32_msg)

		except ValueError:
			print("Invalid input. Please enter a valid float value.")

        # Sleep to maintain the loop rate
		rate.sleep()

if __name__ == '__main__':
	try:
		float32_publisher()
	except rospy.ROSInterruptException:
		pass

