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
#include "nav_msgs/Odometry.h"
#include <fstream>

using namespace std;


/************************************************************
Final kalman filter, it takes also into account the input 
of the quadcopter==> the acceleration along z axis

/mavros/imu/data


*************************************************************/


#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.

//1-state Kalman variables
double Pred_P_k_1 = 0.003;

double Xk;
double Pk;
double Q = 0.5*1e-3;
double R = 2.92e-3;

//for estimate the velocity
bool first = true;
typedef std::queue<double> queue_t; //it allows to implement FIFO access.
queue_t stack_z;
double integral_z = 0;
geometry::UltrasonicPosition temp_plat_filtr;
geometry::UltrasonicPosition temp_plat_prev_filtr;

double z_vel=0;

TimeManager Time;

int i=0;	

void OdomCallback(const nav_msgs::Odometry pose){

	z_vel = pose.twist.twist.linear.z;	

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ultrasonic_sensor_finalKalman");

	UltrasonicSensor Sonar1(trigger, echo);

	ros::NodeHandle n;
	
	//final kalman filter
	ros::Publisher Sensor_pub = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_finalKalman",1); 
	
	//raw data
	ros::Publisher Sensor_pub2 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor",1);

	//one-state kalman filter
	ros::Publisher Sensor_pub3 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_filtr",1);


	ros::Subscriber Imu_sub = n.subscribe("/mavros/local_position/odom",1,OdomCallback);


	ultrasonic_sensor::UltrasonicMsg state;
	ultrasonic_sensor::UltrasonicMsg Full_state_filtr;
	ultrasonic_sensor::UltrasonicMsg state_filtr;


	ros::Rate loop_rate(30);

	//SETUP KALMAN FILTER
	int stateSize = 2; 
	int measSize = 1;
	int contrSize = 1;
	unsigned int type = CV_32F;	

	//initialize the kalman filter
	cv::KalmanFilter kf(stateSize, measSize, contrSize,type );

	
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
	kf.processNoiseCov.at<float>(0) = 0.5*1e-3;
	kf.processNoiseCov.at<float>(1) = 0;	
	kf.processNoiseCov.at<float>(2) = 0;	
	kf.processNoiseCov.at<float>(3) = 0;
	//std::cout<<"matrix Q = "<< std::endl << " "<< kf.processNoiseCov << std::endl << std::endl;


	//Measures Noise covariance Matrix R
	kf.measurementNoiseCov = 2.92e-3;
	
	//Initialization covariance error
	//cv::Mat CovInit = 0.003*cv::Mat::eye(stateSize, stateSize, type);	

	cv::setIdentity(kf.errorCovPre);
	kf.errorCovPre.at<float>(0) = 0.0;
	kf.errorCovPre.at<float>(1) = 0.0;
	kf.errorCovPre.at<float>(2) = 0.0;
	kf.errorCovPre.at<float>(3) = 0.0;

	//simple kalman
	//initialize the kalman filter
	double Pred_Xk_1 = Sonar1.distance(timeout) + 0.1;

	 kf.statePost.at<double>(0) = Pred_Xk_1;
    	 kf.statePost.at<double>(1) = 0;

    	 kf.statePre = kf.statePost;

	Time.updateTimer();	

	while(ros::ok()){

		

		Time.updateTimer();	

		// FULL KALMAN FILTER
		U = 0;
		//std::cout<<"input = "<< std::endl << " "<< U << std::endl << std::endl; 

		kf.transitionMatrix.at<float>(1) = 0;//0.0333; //Updating of matrix F
		kf.transitionMatrix.at<float>(3) = 1; //Updating of matrix F

		std::cout<<"matrix F = "<< std::endl << " "<< kf.transitionMatrix << std::endl; 

		//std::cout<<"matrix B = "<< std::endl << " "<< kf.controlMatrix << std::endl; 

		cv::Mat Prediction = kf.predict(U);
		//std::cout<<"state = "<< std::endl << " "<< kf.statePre<< std::endl << std::endl; 

		
		state.z_Position = Sonar1.distance(timeout) + 0.1; //measurement
		
		meas = state.z_Position;

		kf.correct(meas); //correction

		/////SIMPLE KALMAN
		KF(Pred_Xk_1, Pred_P_k_1, state.z_Position, &Xk, &Pk, Q, R); //kalman filter		
		
		temp_plat_filtr.Z_Position = Xk; //kalman position

		EstimatedVelocity( &temp_plat_filtr, &temp_plat_prev_filtr, &stack_z, &first, Time, &integral_z );


		//////////ROS PUBLISH
		Full_state_filtr.z_Position = kf.statePost.at<float>(0); //filtered position

		Full_state_filtr.z_Velocity = kf.statePost.at<float>(1); //filtered velocity			

		std::cout<<"state: "<<kf.statePre.at<float>(1)<<std::endl;

		state_filtr.z_Position = Xk; 
		state_filtr.z_Velocity = temp_plat_filtr.Z_Velocity;


		Sensor_pub.publish( Full_state_filtr );
		Sensor_pub2.publish( state );
		Sensor_pub3.publish( state_filtr );

		ros::spinOnce();
	
		loop_rate.sleep();
		

	}





}

