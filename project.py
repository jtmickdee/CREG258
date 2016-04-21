#! /usr/bin/env python
import RPi.GPIO as GPIO
import time

camServoPin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(camServoPin, GPIO.OUT)
pwmCamServo = GPIO.PWM(camServoPin, 100)
pwmCamServo.start(5)

carServoPin = 22
GPIO.setup(carServoPin, GPIO.OUT)
pwmCarServo = GPIO.PWM(carServoPin, 100)
pwmCarServo.start

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

#max of percentage for the motor max
PWM_MAX = 100

#motor1 In1 on the h-bridge
motorIn1 = 16
gp.setup(motorIn1, gp.OUT)
gp.output(motorIn1, False)

#motor1 in2 on the h-bridge
motorIn2 = 21
gp.setup(motorIn2, gp.OUT)
gp.output(motorIn2, False)

#motor pwm controls percentage of time on
motorPWM = 20
gp.setup(motorPWM, gp.OUT)
gp.output(motorPWM, True)

pwmCarMotor = gp.PWM(motorPWM, 1000)

pwmCarMotor.start(100)

#sets mode of motor so either forward backward Stopping
#all chars forward is f, backword is r, stop is s
def setMode(mode):
	if mode == 'f':
		gp.output(motorIn1, True)
		gp.output(motorIn2, False)
		print 'Going forwards'
	elif mode == 'r':
		gp.output(motorIn1, False)
		gp.output(motorIn2, True)
		print 'Going backwards'
	else:
		pwmCarMotor.ChangeDutyCycle(0)
		gp.output(motorIn1, False)
		gp.output(motorIn2, False)
		print 'Stopping'

#controls power being applied to motor
def setPower(power):
	if power < 0:
		setMode('r')
		pwm2 = -int(PWM_MAX * power)
		if pwm2 > PWM_MAX:
			pwm2 = PWM_MAX
	elif power > 0:
		setMode('f')
		pwm2 = int(PWM_MAX * power)
		if pwm2 > PWM_MAX:
			pwm2 = PWM_MAX
	else:
		setMode('s')
		pwm2 = 0
		print 'Powering down'
	pwmCarMotor.ChangeDutyCycle(pwm2)

#stops motor and cleanups connections
def exit():
	gp.output(motorIn1, False)
	gp.output(motorIn2, False)
	gp.cleanup()
#while 1:
#	angle = raw_input('Enter angle')
#	update(angle)
updateCamServo(45)
time.sleep(2)
updateCamServo(90)
time.sleep(2)
updateCamServo(135)
time.sleep(2)
#update(180)
#time.sleep(2)
GPIO.cleanup()
