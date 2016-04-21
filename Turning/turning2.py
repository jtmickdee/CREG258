#! /usr/bin/env python
import RPi.GPIO as GPIO
import time

pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
pwm = GPIO.PWM(pin, 100)
pwm.start(5)

#uses center of frame from image processing
centerOfFrame = 320
#allows a default error to have it within range to pass underneath the car
frameError = 10

#updates angle of servo on camera
def update(angle):
	if angle > 180:
		angle = 180
	if angle < 0:
		angle = 0
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)

#adjusts car position based on angle of servo 
#if image is within range
def adjustCarPos(angle):
	print 'adjusting'	
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

	update(angle)
	
def adjustCamera(photoCoord, servoAngle):
	#needs to be updated and should be 1 degree of cchange is x pixels
	#x pixels is the xervoAdjustFactor
	servoAdjustFactor = 10
	#xCenterCoord = 425
	midX = centerOfFrame
	
	diffCoord = midX - photoCoord[0]
	if diffCoord > 0:
		update(servoAngle + abs(diffCoord) % servoAdjustFactor)
	elif diffCoord < 0:
		update(servoAngle - abs(diffCoord) % servoAdjustFactor)

#while 1:
#	angle = raw_input('Enter angle')
#	update(angle)
update(45)
time.sleep(2)
update(90)
time.sleep(2)
update(135)
time.sleep(2)
#update(180)
#time.sleep(2)
GPIO.cleanup()
