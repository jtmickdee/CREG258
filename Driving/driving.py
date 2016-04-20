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

pwm = gp.PWM(motorPWM, 1000)

pwm.start(100)
#pwm.ChangeDutyCycle(80)

def setMode(mode):
	if mode == 'f':
		gp.output(motorIn1, True)
		gp.output(motorIn2, False)
#		pwm.ChangeDutyCycle(100)
		print 'Going forwards'
		#pwm.ChangeDutyCycle(70)
	elif mode == 'r':
		gp.output(motorIn1, False)
		gp.output(motorIn2, True)
#		pwm.ChangeDutyCycle(100)
		print 'Going backwards'
		#pwm.ChangeDutyCycle(70)
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
setMode('f')
while 1:
	dir = raw_input('Enter direction f/r/s ')
	setMode(dir[0])
	#time.sleep(5)
	if(dir[0] == 's'):
		break
exit()




