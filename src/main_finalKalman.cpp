#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/UltrasonicPosition.hpp"
#include "utils/TimeHelpers.hpp"
#include <queue>
#include <cmath>
#include "ros/ros.h"
#include "ultrasonic_sensor/UltrasonicMsg.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "sensor_msgs/Imu.h"

/************************************************************
Final kalman filter, it takes also into account the input 
of the quadcopter==> the acceleration along z axis

/mavros/imu/data


*************************************************************/


#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.

double z_accel;

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	z_accel = msg->linear_acceleration.z;


}







TimeManager tm;

int main(int argc, char **argv) {

	ros::init(argc, argv, "ultrasonic_sensor_finalKalman");

	UltrasonicSensor Sonar1(trigger, echo);

	ros::NodeHandle n;
	
	//sensor data
	ros::Publisher Sensor_pub = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_finalKalman",50); 
	
	ros::Subscriber Imu_sub = n.subscribe("/mavros/imu/data",50,ImuCallback);


	ultrasonic_sensor::UltrasonicMsg Full_state_filtr;

	ros::Rate loop_rate(40);

	//SETUP KALMAN FILTER
	int stateSize = 2; 
	int measSize = 1;
	int contrSize = 1;
	unsigned int type = CV_32F;	

	//initialize the kalman filter
	cv::KalmanFilter kf(stateSize, measSize, contrSize );

	
	//set dt at each processing step.
	//it initialize MATRIX F
	cv::setIdentity(kf.transitionMatrix); 
	//std::cout<<"matrix F = "<< std::endl << " "<< kf.transitionMatrix<< std::endl << std::endl; 

	//set dt at each processing step.
	//it initialize MATRIX B
	kf.controlMatrix.at<float>(0) = 0;
	kf.controlMatrix.at<float>(1) = 1;
	cv::Mat U(1,1,type);

	//std::cout<<"matrix B = "<< std::endl << " "<< kf.controlMatrix<< std::endl << std::endl; 


	//Measure Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1;
	cv:: Mat meas(1,1,type);	

	//std::cout<<"matrix H = "<< std::endl << " "<< kf.measurementMatrix << std::endl << std::endl;


	//Process Noise Covariance matrix Q
	cv::setIdentity(kf.processNoiseCov);
	kf.processNoiseCov.at<float>(0) = 1e-4;
	kf.processNoiseCov.at<float>(3) = 1e-3;
	//std::cout<<"matrix Q = "<< std::endl << " "<< kf.processNoiseCov << std::endl << std::endl;


	//Measures Noise covariance Matrix R
	kf.measurementNoiseCov = 2.92e-3;
	
	//Initialization covariance error
	//cv::Mat CovInit = 0.003*cv::Mat::eye(stateSize, stateSize, type);	

	cv::setIdentity(kf.errorCovPre);
	kf.errorCovPre.at<float>(0) = 0.003;
	kf.errorCovPre.at<float>(3) = 0.003;

	tm.updateTimer();	

	while(ros::ok()){

		tm.updateTimer();	

		U = z_accel;
		//std::cout<<"input = "<< std::endl << " "<< U << std::endl << std::endl; 


	// FULL KALMAN FILTER
		kf.transitionMatrix.at<float>(1) = tm._dt; //Updating of matrix F

		kf.controlMatrix.at<float>(1) = tm._dt; //updating of matrix B

			std::cout<<"matrix B = "<< std::endl << " "<< kf.controlMatrix<< std::endl << std::endl; 

		cv::Mat Prediction = kf.predict(U);
		//std::cout<<"state = "<< std::endl << " "<< kf.statePre<< std::endl << std::endl; 

		

		meas = Sonar1.distance(timeout); //measurement
		
		kf.correct(meas); //correction

		Full_state_filtr.z_Position = kf.statePost.at<float>(0); //filtered position

		Full_state_filtr.z_Velocity = kf.statePost.at<float>(1);
	
	///////////////////////


		ros::spinOnce;
	
		loop_rate.sleep();

	}





}

