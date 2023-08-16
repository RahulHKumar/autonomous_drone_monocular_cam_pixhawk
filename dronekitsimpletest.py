from dronekit import connect,LocationGlobalRelative,APIException,VehicleMode
import time
import socket
import math
import cv2, imutils, socket
import numpy as np
import base64
from multiprocessing import Process



def arm():
    while(vehicle.is_armable==False):
        print("waiting for motors")
        time.sleep(1)
    print("vehicle armable")
    print("****")

    vehicle.armed=True
    while(vehicle.armed==False):
        print("waiting for motors")
        time.sleep(1)
    print("vehicle armed. Lookout for motor blades")
    print("****")
    vehicle.simple_takeoff(2)

    return None


# vehicle=connectMyCopter()
connection_string="/dev/ttyAMA0"
baud_rate=57600
print("Connecting")
vehicle=connect(connection_string,baud=baud_rate,wait_ready=True)
print("Connected to drone")


vehicle.mode=VehicleMode("GUIDED")
vehicle.armed=True
print ("Autopilot Firmware version: %s" % vehicle.version)
print ("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print ("Global Location: %s" % vehicle.location.global_frame)
arm()
init = time.time()

while True:
        curr=time.time()
        passed=curr-init
        if(passed>10):
                print("obstacle")
                vehicle.mode=VehicleMode("GUIDED")
        

print("end of script")