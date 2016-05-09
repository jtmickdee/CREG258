#! /usr/bin/env python
import RPi.GPIO as GPIO
import numpy as np
import cv2
import picamera
from picamera.array import PiRGBArray
import time
import serial
import math
from Lidar.lidarlite import Lidar_Lite

GPIO.setmode(GPIO.BCM)
lidarDist = 62
lidarTol = 10

dilSize = 3
winX = 200
imgHeight = 480
imgWidth = 640
imgMiddle = imgWidth/2

#initialzies the lidar lite
lidar = Lidar_Lite()

#initializes the camera
camera = picamera.PiCamera()
camera.resolution = (imgWidth, imgHeight)

#camera servo pins on raspberry pi
camServoPin1 = 22
camServoPin2 = 27

#car servo pins on raspberry pi
carServoPin1 = 24
carServoPin2 = 23

#initializes the grabbing device direction pin on HBridge
grabIn3 = 5
GPIO.setup(grabIn3, GPIO.OUT)
GPIO.output(grabIn3, False)

#initialize the PWM pin on grab HBridge
grabPWM = 6
GPIO.setup(grabPWM, GPIO.OUT)
GPIO.output(grabPWM, True)

#initializes the grabIn4 pin on HBridge
grabIn4 = 13
GPIO.setup(grabIn4, GPIO.OUT)
GPIO.output(grabIn4, False)

grabFreq = 100
grabPWM = GPIO.PWM(grabPWM, grabFreq)

#starts grabing device PWM
grabPWM.start(100)

#setting pinns to output
GPIO.setup(camServoPin1, GPIO.OUT)
GPIO.setup(camServoPin2, GPIO.OUT)

GPIO.setup(carServoPin1, GPIO.OUT)
GPIO.setup(carServoPin2, GPIO.OUT)

#setting pints to low
GPIO.output(camServoPin1, GPIO.LOW)
GPIO.output(camServoPin2, GPIO.LOW)

GPIO.output(carServoPin1, GPIO.LOW)
GPIO.output(carServoPin2, GPIO.LOW)

PWM_MAX = 100
GPIO.setwarnings(False)

#contorls the pulse that is going into the h-bridge in IN2
motorIn1 = 16
GPIO.setup(motorIn1, GPIO.OUT)
GPIO.output(motorIn1, False)

#contorls the pulse that is going into the h-bridge in IN1
motorIn2 = 21
GPIO.setup(motorIn2, GPIO.OUT)
GPIO.output(motorIn2, False)

#PWM signal that controls the cars motor and speed
carMotorPWM = 20
GPIO.setup(carMotorPWM, GPIO.OUT)
GPIO.output(carMotorPWM, True)

carFreq = 100
carPWM = GPIO.PWM(carMotorPWM, carFreq)

carPWM.start(100)
#changes camera servo angle by sending high and low pin signals to arduino
def changeCamAngle(num):
	num = int(num)
#	if num == 0:
#		GPIO.output(camServoPin1, GPIO.LOW)
#		GPIO.output(camServoPin2, GPIO.LOW)
#		print 'Setting both to low'

#turns to 90
	if num == 1:
		GPIO.output(camServoPin1, GPIO.HIGH)
		GPIO.output(camServoPin2, GPIO.LOW)
		print 'Camera setting 1 to high and 2 to low'
#turns to 170
	elif num == 2:
		GPIO.output(camServoPin1, GPIO.LOW)
		GPIO.output(camServoPin2, GPIO.HIGH)
		print 'Camera setting 1 to low and 2 to high'
#turns to 10
	elif num == 3:
		GPIO.output(camServoPin1, GPIO.HIGH)
		GPIO.output(camServoPin2, GPIO.HIGH)
		print 'Camera setting 1 to high and 2 to high'
	time.sleep(3)

def imgProc():
	rawCap = PiRGBArray(camera)
	time.sleep(0.1)
	camera.capture(rawCap, format="bgr")
	img = rawCap.array

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,  np.array([0.11*256, 0.60*256, 0.20*256, 0]),
		      np.array([0.14*256, 1.00*256, 1.00*256, 0]))

	dilElm = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilSize * 2+1, dilSize * 2+1), (dilSize, dilSize))
	dil = cv2.dilate(mask, dilElm, iterations = 1)

	blur = cv2.medianBlur(dil, 31)

	circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 30, param1 = 100, param2 = 17, minRadius = 0 , maxRadius = 1000);
	print circles
	return circles

#looks at around for a tennis ball and returns arduino position and ball coordinates
def scanCourt():
	angleNum = 1
	ball = None
	print ball
	while 1:
		scanAttempts = 0
		while ball is None:
			changeCamAngle(angleNum)
			ball = imgProc()
			if scanAttempts == 3:
				break
			else:
				scanAttempts+=1
		if ball is None:
			angleNum+=1
			if angleNum > 3:
				break
		else:
			break
	if ball is None:
		changeCamAngle(1)
	return angleNum, ball

#adjusts the wheels
def changeWheelsAngle(num):
#turns to 100
	if num == 1:
		GPIO.output(carServoPin1, GPIO.HIGH)
		GPIO.output(carServoPin2, GPIO.LOW)
		print 'Wheels setting 1 to high and 2 to low'
#turns to 110
	elif num == 2:
		GPIO.output(carServoPin1, GPIO.LOW)
		GPIO.output(carServoPin2, GPIO.HIGH)
		print 'Wheels setting 1 to low and 2 to high'
#turns to 90
	elif num == 3:
		GPIO.output(carServoPin1, GPIO.HIGH)
		GPIO.output(carServoPin2, GPIO.HIGH)
		print 'Wheels setting 1 to high and 2 to high'
	time.sleep(3)
	
#turns on grabbing device
def grabOn():
	GPIO.output(grabIn3, True)
	GPIO.output(grabIn4, False)
	print "grabbing device on"

#turns off grabbing device
def grabOff():
	GPIO.output(grabIn3, False)
	GPIO.output(grabIn4, False)

#tells car to go forward or back
def setCarMode(mode):
	if mode == 'f':
		carPWM.ChangeDutyCycle(100)
		GPIO.output(motorIn1, True)
		GPIO.output(motorIn2, False)
		print 'Going forwards'
	elif mode == 'r':
		carPWM.ChangeDutyCycle(100)
		GPIO.output(motorIn1, False)
		GPIO.output(motorIn2, True)
		print 'Going backwards'
	else:
		carPWM.ChangeDutyCycle(0)
		GPIO.output(motorIn1, False)
		GPIO.output(motorIn2, False)
		print 'Stopping'

#controls cars power
def setCarPower(power):
	if power < 0:
		setCarMode('r')
		pwm2 = -int(PWM_MAX * power)
		if pwm2 > PWM_MAX:
			pwm2 = PWM_MAX
	elif power > 0:
		setCarMode('f')
		pwm2 = int(PWM_MAX * power)
		if pwm2 > PWM_MAX:
			pwm2 = PWM_MAX
	else:
		setCarMode('s')
		pwm2 = 0
		print 'Powering down'
	carPWM.changeDutyCycle(pwm2)

#stops cars and powers down
def exitCar():
	GPIO.output(motorIn1, False)
	GPIO.output(motorIn2, False)

#adds location of ball degree change to the initial angle
def angleConvert(angle):
	num = 0
	if angle < 100:
		num = -1
	elif angle > 100:
		num = 1
	return num

def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False

#turns car to angle specified
def turnCar(angle):
	if angle == 1:
		angle = 90
	elif angle == 2:
		angle = 170
	elif angle ==3:
		angle = 10
	print 'turning car'
	ser = serial.Serial('/dev/ttyACM0', 9600)
	ser.readline()
	numCheck = ser.readline()
	initAngle = float(numCheck)
	destAngle = angleConvert(angle)*angle + initAngle 
	if destAngle > 360:
		destAngle = destAngle - 360
	currAngle = initAngle
	diffAngle = math.fabs(destAngle - currAngle)
	setCarMode('r')
	while diffAngle > 10:
		numCheck = ser.readline()
		currAngle = float(numCheck)
		print currAngle
		diffAngle = math.fabs(destAngle - currAngle)
		time.sleep(.5)
	changeWheelsAngle(3)
	
#detects if there is objects in the way while going forward
def waitDetection(seconds):
	start = time.time()
	time.clock()
	elapsed = 0
	connected = -2
	while connected < -1:
		connected = lidar.connect(1)	
	dist = lidar.getDistance()
	while elapsed < seconds:
		if dist < lidarDist -lidarTol:
			setCarMode('s')
			break
		elapsed = time.time()-start
		time.sleep(1)
		dist = lidar.getDistance()

def checkDistance():
	print 'checking distance'
	connected = lidar.connect(1) 
	while connected < -1:
		setCarMode('s')
		connected = lidar.connect(1)
		time.sleep(2)	
	setCarMode('r')	
	while lidar.getDistance() > lidarDist - lidarTol:
		time.sleep(.5)
	#setCarMode('s')
	grabOn()
	time.sleep(10)
	grabOff()
	setCarMode('s')
	time.sleep(1)

time.sleep(.5)	
while 1:
	setCarMode('s')	
	angle, ball = scanCourt()
	#angle = 45
	setCarMode('r')
	time.sleep(.5)
	if not ball is None:
		changeWheelsAngle(angle)
		turnCar(angle)	
		checkDistance()
	else:
		changeWheelsAngle(3)
		waitDetection(5)
