#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import serial
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('/gripper_lidar', Float32, queue_size=1)
    rospy.init_node('gripper_lidar_publisher', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    # Make a connection to the com port.
    serialPortName = '/dev/ttyUSB0'
    serialPortBaudRate = 115200
    port = serial.Serial(serialPortName, serialPortBaudRate, timeout=0.1)
    rospy.loginfo('Initialized gripper LiDAR on port %s', serialPortName)

    # Enable serial mode by sending some characters over the serial port.
    port.write(str.encode('www\r\n'))
    # Read and ignore any unintended responses
    port.readline()

    # Get the product information
    port.write(str.encode('?\r\n'))
    productInfo = port.readline()
    print('Product information: ' + str(productInfo))

    while not rospy.is_shutdown():
        # Get distance reading (First return, default filtering)
    	port.write(str.encode('LD\r\n'))
    	distanceStr = port.readline()
    	# Convert the distance string response into a number
    	distanceCM = float(distanceStr) * 100

    	# Do what you want with the distance information here
    	pub.publish(distanceCM)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
