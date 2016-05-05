import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

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
		print 'Setting 1 to high and 2 to low'
#turns to 170
	elif num == 2:
		GPIO.output(camServoPin1, GPIO.LOW)
		GPIO.output(camServoPin2, GPIO.HIGH)
		print 'Setting 1 to low and 2 to high'
#turns to 10
	elif num == 3:
		GPIO.output(camServoPin1, GPIO.HIGH)
		GPIO.output(camServoPin2, GPIO.HIGH)
		print 'Setting 1 to high and 2 to high'

#changes car servo angle by sending high and low pin signals to arduino
def changeCarAngle(num):
	num = int(num)
#	if num == 0:
#		GPIO.output(carServoPin1, GPIO.LOW)
#		GPIO.output(carServoPin2, GPIO.LOW)
#		print 'Setting both to low'

#turns to 100
	if num == 1:
		GPIO.output(carServoPin1, GPIO.HIGH)
		GPIO.output(carServoPin2, GPIO.LOW)
		print 'Setting 1 to high and 2 to low'
#turns to 110
	elif num == 2:
		GPIO.output(carServoPin1, GPIO.LOW)
		GPIO.output(carServoPin2, GPIO.HIGH)
		print 'Setting 1 to low and 2 to high'
#turns to 90
	elif num == 3:
		GPIO.output(carServoPin1, GPIO.HIGH)
		GPIO.output(carServoPin2, GPIO.HIGH)
		print 'Setting 1 to high and 2 to high'
num =1
changeCarAngle(num)
time.sleep(5)
num =2
changeCarAngle(num)
time.sleep(5)

num =3
changeCarAngle(num)
time.sleep(5)
GPIO.cleanup()
