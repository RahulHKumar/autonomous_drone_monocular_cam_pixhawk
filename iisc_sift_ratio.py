import numpy as np
import cv2 as cv

cap = cv.VideoCapture(0)

while True:
    ret,img = cap.read()
    gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    sift = cv.SIFT_create()
    kp = sift.detect(gray,None)
    k_img=cv.drawKeypoints(gray,kp,img,flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv.imshow('sift_keypoints',k_img)
    edges = cv.Canny(k_img,100,200)
    cv.imshow('edges',k_img)

    if cv.waitKey(1)==ord('q'):
        break

# Release the VideoCapture object
cap.release()
cv.destroyAllWindows()