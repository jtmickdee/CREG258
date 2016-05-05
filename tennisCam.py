import numpy as np
import cv2
import picamera
from picamera.array import PiRGBArray
import time

trackbarWindowName = "Trackbars"
normPic = "norm"
hsvPic = "hsv"
maskPic ="mask"
dilPic ="dilation"
blurPic = "blurred"

dilSize = 21
winX = 200
imgHeight = 480
imgWidth = 640
imgMiddle = imgWidth/2

camera = picamera.PiCamera()
camera.resolution = (640, 480)
rawCap = PiRGBArray(camera)
time.sleep(0.1)
camera.capture(rawCap, format="bgr")
img = rawCap.array

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv,  np.array([0.11*256, 0.60*256, 0.20*256, 0]),
	      np.array([0.14*256, 1.00*256, 1.00*256, 0]))
#hsv.release()

dilElm = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilSize * 2+1, dilSize * 2+1), (dilSize, dilSize))
dil = cv2.dilate(mask, dilElm, iterations = 1)

blur = cv2.medianBlur(dil, 31)
#dil.release()

#circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 2, blur.shape[0]/4, 200, 100)
circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 2, blur.shape[0]/4, param1=100, param2=100, minRadius=0, maxRadius=0)
#blur.release()

print circles
cv2.imshow('img', img)
cv2.imshow('hsv', hsv)
cv2.imshow('blur', blur)
cv2.waitKey(0)
cv2.destroyAllWindows()
#center = (circles[0, 0], circles[0, 1])
#circles= np.uint16(np.around(circles))
