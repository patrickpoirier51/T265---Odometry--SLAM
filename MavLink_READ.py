#!/usr/bin/env python
### https://gist.github.com/vo/9331349  Simple Mavlink Reader in Python using pymavlink

import math
import time
import sys, os
from optparse import OptionParser

# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil

def handle_heartbeat(msg):
    print "Ping!"

def handle_vision(msg):
    vision_data = (msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)

    x = msg.x
    y = msg.y
    z = msg.z

    roll = msg.roll
    pitch = msg.pitch
    yaw = msg.yaw

    # Convert in Degreee for visualisation    
    #roll *=180/math.pi
    #pitch *=180/math.pi           
    #yaw *=180/math.pi

    #if roll < 0: roll +=360  #ensure it stays between 0 - 360
    #if pitch < 0: pitch +=360  #ensure it stays between 0 - 360      
    #if yaw < 0: yaw +=360  #ensure it stays between 0 - 360

    #print ("Roll", round(roll) , "Pitch", round(pitch) ,"Yaw", round(yaw))
    print ("Roll", ("%.2f" % roll) , "Pitch", ("%.2f" % pitch) ,"Yaw", ("%.2f" % yaw))


def read_loop(m):

	while(True):
		# grab a mavlink message
		msg = m.recv_match(blocking=True)
		#print "grab"
		if not msg:
			return
	    
		# handle the message based on its type
		msg_type = msg.get_type()
		if  msg_type == "VISION_POSITION_ESTIMATE":
			handle_vision(msg)

		#time.sleep(0.05)

		
def main():

	# create a mavlink udp instance
	print "connecting"
	master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

	# enter the data loop
	read_loop(master)


if __name__ == '__main__':
	main()
