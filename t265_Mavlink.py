#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 MavLink             ##
#####################################################

# Import the libraries
from dronekit import connect, VehicleMode
import time
import math

# Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")
import pyrealsense2 as rs


#https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE

def send_vision_position_message(x,y,z,roll,pitch,yaw):
    msg = vehicle.message_factory.vision_position_estimate_encode(
        start_time,	#us	Timestamp (UNIX time or time since system boot)
	x,	        #Global X position
	y,              #Global Y position
        z,	        #Global Z position
        roll,	        #Roll angle
        pitch,	        #Pitch angle
        yaw	        #Yaw angle
        #0              #covariance :upper right triangle (states: x, y, z, roll, pitch, ya
        #0              #reset_counter:Estimate reset counter. 
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    


    
############# Initialize system ###############

#Add extented delay for FC to start
#print("\nDelay for FC to start")
#time.sleep(1)

# Connect to the Vehicle.
print("\nConnecting to vehicle")
#vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True))
vehicle = connect('tcp:192.168.2.239:5763', wait_ready=True)
#vehicle = connect('/dev/ttyUSB0', baud=115200)
print("\nConnected")

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)
print("\nStart streaming")


while True:
    start_time=time.time()
    #microseconds = int(round(time.time())

    # Wait for the next set of frames from the camera
    frames = pipe.wait_for_frames()

    # Fetch pose frame
    pose = frames.get_pose_frame()
    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()
        #print("Frame #{}".format(pose.frame_number))
        #print("T{}".format(data.translation)),
        #print("R{}".format(data.rotation)),
        #print("Velocity: {}".format(data.velocity))
        #print("Acceleration: {}\n".format(data.acceleration))
        #print ("Time", start_time , "FPS", round(.01 / (time.time()- start_time))) #,

        x =  data.translation.x
        y =  data.translation.y
        z =  data.translation.z
        
        Qw = data.rotation.w
        Qx = data.rotation.x
        Qy = data.rotation.z    # permutation of z and y
        Qz = data.rotation.y    # as Chris Anderson proposition in RealSense Github
        

        #LOOKING FORWARD
        #roll = math.asin(2.0*(Qx*Qz - Qw*Qy))
        #pitch = math.atan2(2.0*(Qy*Qz + Qw*Qx), Qw*Qw -Qx*Qx - Qy*Qy + Qz*Qz)              
        #yaw = math.atan2(2.0*(Qx*Qy + Qw*Qz), Qw*Qw + Qx*Qx - Qy*Qy - Qz*Qz)

        
        #LOOKING DOWNWARD
        roll = math.atan2(2.0*(Qx*Qy + Qw*Qz), Qw*Qw + Qx*Qx - Qy*Qy - Qz*Qz) 
        pitch = math.atan2(2.0*(Qy*Qz + Qw*Qx), Qw*Qw -Qx*Qx - Qy*Qy + Qz*Qz)       
        yaw = math.asin(2.0*(Qx*Qz - Qw*Qy))

        
        # Send vectors to Mavlink according to distance and azimuth (above)
        send_vision_position_message(x,y,z,roll,pitch,yaw)
        print "R", ("%0.2f" %roll) , "  P", ("%0.2f" %pitch) ,"  Y", ("%0.2f" %yaw)
               

        #Convert in Degreee for visualisation
        roll *=180/math.pi
        pitch *=180/math.pi           
        yaw *=180/math.pi

        if roll < 0: roll +=360  #ensure it stays between 0 - 360
        if pitch < 0: pitch +=360  #ensure it stays between 0 - 360      
        if yaw < 0: yaw +=360  #ensure it stays between 0 - 360

        #print ("Roll", round(roll) , "Pitch", round(pitch) ,"Yaw", round(yaw))
        #print ("Roll", (roll) , "Pitch", (pitch) ,"Yaw", (yaw))

        time.sleep(.01)



print("\nStop streaming")
pipe.stop()
#Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()
