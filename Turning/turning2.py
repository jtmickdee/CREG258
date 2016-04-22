#! /usr/bin/env python
import RPi.GPIO as GPIO
import time
import picamera

camera = picamera.PiCamera()

camServoPin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(camServoPin, GPIO.OUT)
pwmCamServo = GPIO.PWM(camServoPin, 100)
pwmCamServo.start(5)

carServoPin = 22
GPIO.setup(carServoPin, GPIO.OUT)
pwmCarServo = GPIO.PWM(carServoPin, 100)
#pwmCarServo.start(5)

#uses center of frame from image processing
centerOfFrame = 320
#allows a default error to have it within range to pass underneath the car
frameError = 10

#updates angle of servo on camera
def updateCamServo(angle):
	if angle > 180:
		angle = 180
	if angle < 0:
		angle = 0
        duty = float(angle) / 10.0 + 2.5
        pwmCamServo.ChangeDutyCycle(duty)
	time.sleep(.05)
	pwmCamServo.stop()

def updateCarServo(angle):
	if angle > 180:
		angle = 180
	if angle < 0:
		angle = 0
        duty = float(angle) / 10.0 + 2.5
        pwmCarServo.ChangeDutyCycle(duty)

#adjusts car position based on angle of servo
#if image is within range
def adjustCarPos(angle):
	print 'adjusting'
	updateCarServo(angle)
	
#uses image processing with angle update
#takes too long to on  raspberrry pi
def checkPhoto(xDim, angle):
	#how much to adjust the angle of the servo
	adjustFactor = angle/2
	if xDim < centerOfFrame + frameError:
		angle -= adjustFactor
	elif xDim > centerOfFrame - frameError:
		angle += adjustFactor
	else:
		#Call driving software
		adjustCarPos(angle)

	updateCamServo(angle)

def adjustCamera(photoCoord, servoAngle):
	#needs to be updated and should be 1 degree of cchange is x pixels
	#x pixels is the xervoAdjustFactor
	servoAdjustFactor = 10
	#xCenterCoord = 425
	midX = centerOfFrame

	diffCoord = midX - photoCoord[0]
	if diffCoord > 0:
		updateCamServo(servoAngle + abs(diffCoord) % servoAdjustFactor)
	elif diffCoord < 0:
		updateCamServo(servoAngle - abs(diffCoord) % servoAdjustFactor)

#while 1:
#	angle = raw_input('Enter angle')
#	update(angle)
updateCamServo(0)
time.sleep(2)
pwmCamServo.start(5)
print('changing servo location')
updateCamServo(45)
print('taking picture')
camera.capture('firstDegreePic.jpg')
time.sleep(5)
pwmCamServo.start(5)
print('changing servo location')
time.sleep(1)
updateCamServo(46)
print('taking picture')
time.sleep(.001)
camera.capture('secondDegreePic.jpg')
time.sleep(2)
GPIO.cleanup()
