import cv2
import numpy as np
import time

def PolyArea2D(pts):
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area

cap = cv2.VideoCapture(0)
list=[]
while True:
    ret1,img = cap.read()
    # Convert it to greyscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Applying the blur function
    img=cv2.blur(img,(30,30))
    # Threshold the image
    ret, thresh = cv2.threshold(img,50,255,0)
    # Find the contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # For each contour, find the convex hull and draw it
    # on the original image.
    for i in range(len(contours)):
        hull = cv2.convexHull(contours[i])
        list.append(hull[0,0].tolist())
        print(PolyArea2D(list)/10000)
        cv2.drawContours(img, [hull], -1, (255, 0, 0), 2)
    # Display the final convex hull image
    cv2.imshow('ConvexHull', img)
    time.sleep(0.5)
    if cv2.waitKey(1)==ord('q'):
        break

# Release the VideoCapture object
cap.release()
cv.destroyAllWindows()