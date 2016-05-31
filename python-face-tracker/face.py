"""
The MIT License (MIT)

Copyright (c) 2016 Embedit Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

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

