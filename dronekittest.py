from dronekit import connect,LocationGlobalRelative,APIException,VehicleMode
import time
import socket
import math
import cv2, imutils, socket
import numpy as np
import base64



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
arm()

while True:
        msg,client_addr = server_socket.recvfrom(BUFF_SIZE)
        print('GOT connection from ',client_addr)
        WIDTH=400
        while(vid.isOpened()):
                _,frame = vid.read()
                frame = imutils.resize(frame,width=WIDTH)
                encoded,buffer = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
                message = base64.b64encode(buffer)
                server_socket.sendto(message,client_addr)
                frame = cv2.putText(frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
                #cv2.imshow('TRANSMITTING VIDEO',frame)
                msg,client_addr = server_socket.recvfrom(BUFF_SIZE)


                # print(msg)
                if(msg==b"N"):
                        print("obstacle")
                        vehicle.mode=VehicleMode("GUIDED")
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

print("end of script")