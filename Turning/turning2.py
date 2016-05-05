import RPi.GPIO as GPIO
import time
import picamera


#camera = picamera.PiCamera()
#camera.resolution = (640, 480)

camServoPin = 19
GPIO.setmode(GPIO.BCM)
GPIO.setup(camServoPin, GPIO.OUT)
pwmCamServo = GPIO.PWM(camServoPin, 25)
pwmCamServo.start(5)

carServoPin = 12
GPIO.setup(carServoPin, GPIO.OUT)
pwmCarServo = GPIO.PWM(carServoPin, 100)
#pwmCarServo.start(5)

#uses center of frame from image processing
centerOfFrame = 320
#allows a default error to have it within range to pass underneath the car
frameError = 10

#updates angle of servo on camera
def updateCamServo(angle):
	print('Servo turning to ' + str(angle))
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
	time.sleep(5)

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

def takePhoto(filename):
#	camera = picamera.PiCamera()
#	camera.resolution = (640, 480)
#	camera.capture(str(filename)+'.jpg')
	print('taking picture saved at ' + str(filename))
	time.sleep(2)
	#camera.close()
#while 1:
#	angle = raw_input('Enter angle')
#	updateCamServo(angle)
updateCamServo(90)
time.sleep(5)
updateCamServo(45)
time.sleep(5)
updateCamServo(135)
time.sleep(5)
while 1:
	updateCamServo(10)
#camera.close()
#GPIO.cleanup()
