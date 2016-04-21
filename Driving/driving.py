#!/usr/bin/env python
import RPi.GPIO as gp
import time

gp.setmode(gp.BCM)
PWM_MAX = 100
gp.setwarnings(False)

motorIn1 = 16
gp.setup(motorIn1, gp.OUT)
#gp.output(motorIn1, False)


motorIn2 = 21
gp.setup(motorIn2, gp.OUT)
gp.output(motorIn2, False)


motorPWM = 20
gp.setup(motorPWM, gp.OUT)
gp.output(motorPWM, True)

frequency = 100
pwm = gp.PWM(motorPWM, frequency)

pwm.start(100)

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
		pwm.ChangeDutyCycle(0)
		gp.output(motorIn1, False)
		gp.output(motorIn2, False)
		print 'Stopping'
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
def exit():
	gp.output(motorIn1, False)
	gp.output(motorIn2, False)
	gp.cleanup()
#pow = input('Enter power level')
pow =1 
print('Power level is ', pow)
#setPower(pow)
setMode('r')
time.sleep(2)
pwm.ChangeDutyCycle(50)
while 1:
	dir = raw_input('Enter direction f/r/s ')
	setMode(dir[0])
	if(dir[0] == 's'):
		break
exit()




