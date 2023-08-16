from dronekit import connect,LocationGlobalRelative,APIException,VehicleMode
import time
import socket
import math
import cv2, imutils, socket
import numpy as np
import base64
import cv2, imutils
import numpy as np
import base64
import torch

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


BUFF_SIZE = 65536*4
server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
host_name = socket.gethostname()
host_ip = '192.168.162.12'#  socket.gethostbyname(host_name)
print(host_ip)
port = 9999
socket_address = (host_ip,port)
server_socket.bind(socket_address)
print('Listening at:',socket_address)

vid = cv2.VideoCapture(0) #  replace 'rocket.mp4' with 0 for webcam
fps,st,frames_to_count,cnt = (0,0,20,0)

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

#Arm and take of to altitude of 5 meters
arm_and_takeoff(0.5)

while True:
        msg,client_addr = server_socket.recvfrom(BUFF_SIZE)
        start=time.time()
        diff=0
        print('GOT connection from ',client_addr)
        WIDTH=400
        while(vid.isOpened()):
                _,frame = vid.read()
                curr=time.time()
                diff=curr-start
                if(diff>2):
                    break
                frame = imutils.resize(frame,width=WIDTH)
                encoded,buffer = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
                message = base64.b64encode(buffer)
                server_socket.sendto(message,client_addr)
                frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
                #cv2.imshow('TRANSMITTING VIDEO',frame)
                msg,client_addr = server_socket.recvfrom(BUFF_SIZE)

                if(msg==b"N"):
                    vehicle.mode=VehicleMode("LAND")
                else:
                    send_ned_velocity(0.5, 0, 0, 2)

                # print(msg)
                # if(msg==b"N"):
                #         print("obstacle")
                # arm()
                key = cv2.waitKey(1) & 0xFF
                if(key == ord('q')):
                        server_socket.close()
                        break
                if(cnt == frames_to_count):
                        try:
                                fps = round(frames_to_count/(time.time()-st))
                                st=time.time()
                                cnt=0
                        except:
                                pass
                cnt+=1
        if(diff>2):
            break

print("end of script")

send_ned_velocity(0.5, 0, 0, 5)
vehicle.mode=VehicleMode("LAND")

