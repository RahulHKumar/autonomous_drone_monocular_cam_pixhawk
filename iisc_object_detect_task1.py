import numpy as np
import cv2 as cv
import time
from dronekit import connect,LocationGlobalRelative,APIException,VehicleMode

cap = cv.VideoCapture(0)
thresh=1000


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
    vehicle.simple_takeoff(5)
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


start=time.time()
diff=0
while True:
    curr=time.time()
    diff=curr-start
    if(diff>5):
        vehicle.mode=VehicleMode("LAND")
        break
    arm()
    ret,img = cap.read()
    # img=cv.imread("s5.png")
    h,w,channels = img.shape
    b_img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    n_img=cv.cvtColor(img,cv.COLOR_BGR2RGB)

    # Applying the blur function
    blur_img=cv.blur(b_img,(10,10))
    sift = cv.SIFT_create()
    kp = sift.detect(b_img,None)
    k_img=cv.drawKeypoints(b_img,kp,img,flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv.imshow('sift_keypoints',k_img)

    s1=blur_img[0:h , 0:round(w/3)]
    s2=blur_img[0:round(h/3) , round(w/3):round(2*w/3)]
    s3=blur_img[round(h/3):round(2*h/3) , round(w/3):round(2*w/3)]
    s4=blur_img[round(2*h/3):h , round(w/3):round(2*w/3)]
    s5=blur_img[0:h , round(2*w/3):w]

    edges = cv.Canny(k_img,100,200)

    cv.imshow('Original', img)
    cv.imshow('Grayscale', b_img)
    cv.imshow('Blurred', blur_img)
    cv.imshow('Edge', edges)

    e1=cv.Canny(s1,100,200)
    e2=cv.Canny(s2,100,200)
    e3=cv.Canny(s3,100,200)
    e4=cv.Canny(s4,100,200)
    e5=cv.Canny(s5,100,200)

    n=np.sum(blur_img>150)
    print(n)

    if(diff>thresh):
        vehicle.mode=VehicleMode("LAND")
        break

    n_1 = np.sum(e1 > 150)
    n_2 = np.sum(e2 > 150)
    n_3 = np.sum(e3 > 150)
    n_4 = np.sum(e4 > 150)
    n_5 = np.sum(e5 > 150)

    print('Segment 1:', n_1)
    print('Segment 2:', n_2)
    print('Segment 3:', n_3)
    print('Segment 4:', n_4)
    print('Segment 5:', n_5)

    if cv.waitKey(1)==ord('q'):
        break

# Release the VideoCapture object
cap.release()
cv.destroyAllWindows()
