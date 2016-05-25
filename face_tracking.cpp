/*
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
*/

#include "pisoc.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include "raspicam_cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <thread>
#include <queue>
#include <vector>
#include <mutex>
#include <cmath>
#include <condition_variable>

#define RESOLUTION_WIDTH		960
#define RESOLUTION_HEIGHT		720
#define DETECTION_WIDTH			320
#define DETECTION_HEIGHT		240

template <class T>
class Producer
{
	public:
		Producer() : _running(true), _new(true) {}
		void produce(const T &src)
		{
			std::lock_guard<std::mutex> lock(this->_mutex);
			_loc = src;
			_new = true;
			_cv.notify_one();
		}
		void stop()
		{
			std::lock_guard<std::mutex> lock(this->_mutex);
			_running = false;
		}
		void wait()
		{
			std::unique_lock<std::mutex> lock(this->_mutex);
			_cv.wait(lock, std::bind(&Producer<T>::new_data, this));
		}
		bool consume(T &dst)
		{
			std::lock_guard<std::mutex> lock(this->_mutex);
			dst = _loc;
			_new = false;
			return _running;
		}
	private:
		bool new_data()
		{
			return _new;
		}
		T _loc;
		std::mutex _mutex;
		std::condition_variable _cv;
		bool _running;
		bool _new;
};

class Profile
{
	public:
		Profile() : start(0), end(0), smoothing(0.9), frequency(0) {}
		void start_measurement()
		{
			start = cv::getTickCount();
		}
		void stop_measurement()
		{
			end = cv::getTickCount();
			double f = ((float)cv::getTickFrequency())/(end - start);
			frequency = frequency > 0 ? ( frequency * smoothing) + (f * (1.0 - smoothing)) : f; 
		}
		double get_frequency()
		{
			return frequency;
		}
	private:
		double start;
		double end;
		float smoothing;
		double frequency;
		
};


void Detection(Producer<cv::Mat> &stream, Producer<std::vector<cv::Rect>> &solutions, Producer<cv::Point> &target);
void Display(Producer<cv::Mat> &stream, Producer<std::vector<cv::Rect>> &solutions);
void Track(Producer<cv::Point> &target);


void Detection(Producer<cv::Mat> &stream, Producer<std::vector<cv::Rect>> &solutions, Producer<cv::Point> &target)
{
	bool running = true;
	constexpr float scale_x = (float)RESOLUTION_WIDTH/(float)DETECTION_WIDTH;
	constexpr float scale_y = (float)RESOLUTION_HEIGHT/(float)DETECTION_HEIGHT;
	cv::Mat frame, gray;
	cv::CascadeClassifier face_cascade;
	
	std::string fn_lbp = "/usr/local/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml";
	if (!face_cascade.load(fn_lbp))
	{
		throw std::runtime_error("Unable to locate cascade classifier " + fn_lbp);
	}
	
	while (running)
	{
		std::vector<cv::Rect> faces;
		running = stream.consume(frame);
		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		cv::resize(gray, gray, cv::Size(DETECTION_WIDTH, DETECTION_HEIGHT));
		
		face_cascade.detectMultiScale(gray, faces, 1.1, 3, CV_HAAR_SCALE_IMAGE, cv::Size(80, 80));
		
		std::vector<cv::Rect> faces_resized(faces.size());
		
		for (int i = 0; i< faces.size(); i++)
		{
			cv::Rect face = faces[i];
			faces_resized[i] = cv::Rect(face.tl().x * scale_x, face.tl().y * scale_y, face.width * scale_x, face.height * scale_y);
		}
		
		cv::Point center = faces.size() ? cv::Point(cvRound((faces[0].tl().x + faces[0].br().x)/2), cvRound((faces[0].tl().y + faces[0].br().y)/2)) : cv::Point(DETECTION_WIDTH/2, DETECTION_HEIGHT/2);
	
		solutions.produce(faces_resized);
		target.produce(center);
		
	}
	target.stop();
	solutions.stop();
}

void Display(Producer<cv::Mat> &stream, Producer<std::vector<cv::Rect>> &solutions)
{
	raspicam::RaspiCam_Cv Camera;
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, RESOLUTION_WIDTH);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION_HEIGHT);
	
	if (!Camera.open())
	{
		throw std::runtime_error("Unable to open Raspberry Pi camera");
	}
	
	bool running = true;
	cv::Mat frame;
	cv::Mat result;
	std::vector<cv::Rect> faces;
	
	Profile FPS;
	
	while(running)
	{
		FPS.start_measurement();
		
		Camera.grab();
		Camera.retrieve(frame);
		stream.produce(frame);
		
		solutions.consume(faces);
		
		for (int i = 0 ; i< faces.size(); i++)
		{
			cv::rectangle(frame, faces[i], cv::Scalar::all(255), 1);
		}
		
		cv::putText(frame, std::to_string(static_cast<int>(std::round(FPS.get_frequency()))) + " FPS", cv::Point(0, frame.rows), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(255), 1, 8, false);
		cv::imshow("Face Tracking", frame);
		if (cv::waitKey(1) == 27)
		{
			running = false;
		}
		
		FPS.stop_measurement();
	}
	cv::destroyAllWindows();
	stream.stop();
}

void Track(Producer<cv::Point> &target)
{
	constexpr float thresh_x = DETECTION_WIDTH/10.0;
	constexpr float thresh_y = DETECTION_HEIGHT/5.0;
	constexpr int delta_x = DETECTION_WIDTH/80;
	constexpr int delta_y = DETECTION_HEIGHT/80;
	
	bool running = true;
	cv::Point center;
	const cv::Point target_point(DETECTION_WIDTH/2, DETECTION_HEIGHT/2);
	
	Servo pan(0);
	Servo tilt(1);
	
	pan.Start();
	tilt.Start();
	
	pan.changeAngleRange(0, DETECTION_WIDTH);
	tilt.changeAngleRange(0, DETECTION_HEIGHT);
	
	pan.setAngle(DETECTION_WIDTH/2);
	tilt.setAngle(DETECTION_HEIGHT/2);
	
	while (running)
	{
		target.wait();
		running = target.consume(center);
		
		if (center.x > target_point.x + thresh_x)
			pan.Move(-1*delta_x);
		else if (center.x < target_point.x - thresh_x)
			pan.Move(delta_x);
		
		if (center.y > target_point.y + thresh_y)
			tilt.Move(delta_y);
		else if (center.y < target_point.y - thresh_y)
			tilt.Move(-1*delta_y);
		
			
	}
	
	pan.Stop();
	tilt.Stop();
}

int main(int argc, char **argv)
{
	Producer<cv::Mat> stream;
	Producer<std::vector<cv::Rect>> solutions;
	Producer<cv::Point> target;
	
	stream.produce(cv::Mat(RESOLUTION_HEIGHT, RESOLUTION_WIDTH, CV_8UC3, cv::Scalar::all(0)));
	target.produce(cv::Point(DETECTION_WIDTH/2, DETECTION_HEIGHT/2));
	
	std::thread detection(std::bind(Detection, std::ref(stream), std::ref(solutions), std::ref(target)));
	std::thread display(std::bind(Display, std::ref(stream), std::ref(solutions)));
	std::thread track(std::bind(Track, std::ref(target)));
	
	display.join();
	detection.join();
	track.join();
	
	return 0;
}


