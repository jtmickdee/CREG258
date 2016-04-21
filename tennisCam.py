import numpy as n
import cv2
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

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  inRange(hsv,  Scalar(0.11*256, 0.60*256, 0.20*256, 0),
	                 Scalar(0.14*256, 1.00*256, 1.00*256, 0), mask)
hsv.release()

dilElm = cv2.getStructuringElement(){cv2.MORPH_ELLIPSE, (dilSize * 2+1, dilSize * 2+1), (dilSize, dilSize))
dil = cv2.dilate(mask, dilElm, iterations = 1)

blur = cv2.medianBlur(dil, 31)
dil.release()

circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 2, blur.rows/4, 200, 100)
blur.release()

center = (circles[0, 0], circles[0, 1])

