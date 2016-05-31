from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from pisoc import *

class Point(object):
	def __init__(self, x, y):
		self.x = x
		self.y = y
		
def Track(pan, tilt, center, target = Point(160, 120), threshold = Point(16, 24), delta = Point(4, 3)):
	if (center.x > target.x + threshold.x):
		pan.SetAngle(pan.ReadAngle() - delta.x)
	elif (center.x < target.x - threshold.x):
		pan.SetAngle(pan.ReadAngle() + delta.x)
	if (center.y > target.y + threshold.y):
		tilt.SetAngle(tilt.ReadAngle() + delta.y)
	elif (center.y < target.y - threshold.y):
		tilt.SetAngle(tilt.ReadAngle() - delta.y)

if __name__ == "__main__":
	PiSoC(log_level = 'debug')
	pan = Servo(0, max_angle = 320)
	tilt = Servo(1, max_angle = 240)
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size = camera.resolution)

	face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml')

	scale = (camera.resolution[0]/320.0, camera.resolution[1]/240.0)

	time.sleep(0.1)
	
	pan.Start()
	tilt.Start()

	for frame in camera.capture_continuous(rawCapture, format = 'bgr', use_video_port = True):
		image = frame.array
		
		resized = cv2.resize(image, (320, 240))
		gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
		
		faces = face_cascade.detectMultiScale(gray, 1.1, 5)
		if len(faces) > 0:
			for (x, y, w, h) in faces:
				Track(pan, tilt, Point(x + w/2.0, y+ h/2.0))
				break
		faces_resized = [(int(scale[0]*x), int(scale[1]*y), int(scale[0]*w), int(scale[1]*h)) for (x, y, w, h) in faces]
		for (x,y,w,h) in faces_resized:
			cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
		
		cv2.imshow("Result", image)
		key = cv2.waitKey(1) & 0xFF
		
		rawCapture.truncate(0)
		
		if key == ord('q') or key == 27:
			break
	pan.Stop()
	tilt.Stop()

