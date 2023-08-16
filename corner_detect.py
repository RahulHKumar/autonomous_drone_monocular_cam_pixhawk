import cv2
import numpy as np
import time


cap = cv2.VideoCapture(0)

def cal_dist(x,y):
    s=(x**2+y**2)**0.5
    return s

while True:
    ret,img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    gray=cv2.blur(gray,(10,10))
    dst = cv2.cornerHarris(gray,5,3,0.04)
    ret, dst = cv2.threshold(dst,0.1*dst.max(),255,0)
    dst = np.uint8(dst)
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)
    for i in range(1, len(corners)):
        print(cal_dist(corners[i][0], corners[i][1]))
    img[dst>0.1*dst.max()]=[0,0,255]
    cv2.imshow('image', img)
    time.sleep(0.5)

    if cv2.waitKey(1)==ord('q'):
        break


# Release the VideoCapture object
cap.release()
cv2.destroyAllWindows()