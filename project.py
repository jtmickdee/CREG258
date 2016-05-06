#! /usr/bin/env python
import RPi.GPIO as GPIO
import numpy as np
import cv2
import picamera
from picamera.array import PiRGBArray
import time
import serial
import math

GPIO.setmode(GPIO.BCM)

dilSize = 3
winX = 200
imgHeight = 480
imgWidth = 640
imgMiddle = imgWidth/2

camera = picamera.PiCamera()
camera.resolution = (imgWidth, imgHeight)

#camera servo pins on raspberry pi
camServoPin1 = 22
camServoPin2 = 27

#car servo pins on raspberry pi
carServoPin1 = 24
carServoPin2 = 23

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

carPWM.start(0)
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
	time.sleep(6)

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
			if scanAttempts == 1:
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
	time.sleep(6)
	
	
#creates a Lidar Lite object to get distance and velocity
class Lidar_Lite():
  def __init__(self):
    self.address = 0x62
    self.distWriteReg = 0x00
    self.distWriteVal = 0x04
    self.distReadReg1 = 0x8f
    self.distReadReg2 = 0x10
    self.velWriteReg = 0x04
    self.velWriteVal = 0x08
    self.velReadReg = 0x09

  def connect(self, bus):
    try:
      self.bus = smbus.SMBus(bus)
      time.sleep(0.5)
      return 0
    except:
      return -1

  def writeAndWait(self, register, value):
    self.bus.write_byte_data(self.address, register, value);
    time.sleep(0.02)

  def readAndWait(self, register):
    res = self.bus.read_byte_data(self.address, register)
    time.sleep(0.02)
    return res

  def getDistance(self):
    self.writeAndWait(self.distWriteReg, self.distWriteVal)
    dist1 = self.readAndWait(self.distReadReg1)
    dist2 = self.readAndWait(self.distReadReg2)
    return (dist1 << 8) + dist2

  def getVelocity(self):
    self.writeAndWait(self.distWriteReg, self.distWriteVal)
    self.writeAndWait(self.velWriteReg, self.velWriteVal)
    vel = self.readAndWait(self.velReadReg)
    return self.signedInt(vel)

  def signedInt(self, value):
    if value > 127:
      return (256-value) * (-1)
    else:
      return value

#tells car to go forward or back
def setCarMode(mode):
	if mode == 'f':
		GPIO.output(motorIn1, True)
		GPIO.output(motorIn2, False)
		print 'Going forwards'
	elif mode == 'r':
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

#turns car to angle specified
def turnCar(angle):
	print 'turning car'
	ser = serial.Serial('/dev/ttyACM0', 9600)
	ser.readline()
	print ser.readline()
	print ser.readline()
	initAngle = float(ser.readline())
	destAngle = angleConvert(angle)*angle + initAngle 
	if destAngle > 360:
		destAngle = destAngle - 360
	currAngle = initAngle
	diffAngle = math.fabs(destAngle - currAngle)
	setCarMode('r')
	while diffAngle > 10:
		currAngle = float(ser.readline())
		print currAngle
		diffAngle = math.fabs(destAngle - currAngle)
		time.sleep(.5)
	changeWheelsAngle(3)
	
def checkDistance():
	print 'checking distance'
	lidar = Lidar_Lite()
	connected = lidar.connect(1)
	while connected < -1:
		setCarMode('s')
		connected = lidar.connect(1)
		time.sleep(2)	
	setCarMode('r')	
	while lidar.getDistance() >50:
		time.sleep(.5)
	setCarMode('s')
	
changeWheelsAngle(2)
turnCar(45)	
