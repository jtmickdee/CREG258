#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
PWM_MAX = 100
GPIO.setwarnings(False)

motorIn1 = 16
GPIO.setup(motorIn1, GPIO.OUT)
#GPIO.output(motorIn1, False)


motorIn2 = 21
GPIO.setup(motorIn2, GPIO.OUT)
GPIO.output(motorIn2, False)


motorPWM = 20
GPIO.setup(motorPWM, GPIO.OUT)
GPIO.output(motorPWM, True)

carFreq = 100
carPWM = GPIO.PWM(motorPWM, carFreq)

carPWM.start(0)

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

def exitCar():
	GPIO.output(motorIn1, False)
	GPIO.output(motorIn2, False)
	GPIO.cleanup()
#pow = input('Enter power level')
pow =1
print('Power level is ', pow)
#setCarPower(pow)
time.sleep(2)
#carPWM.ChangeDutyCycle(50)
while 1:
	dir = raw_input('Enter direction f/r/s ')
	setCarMode(dir[0])
	if(dir[0] == 's'):
		break
exitCar()
