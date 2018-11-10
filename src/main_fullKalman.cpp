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


/************************************************************
It compares the basic kalman filter, the full kalman and 
the avarage filter
*************************************************************/


#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.


//For connecting with mocap2mav
geometry::UltrasonicPosition temp_plat;
geometry::UltrasonicPosition temp_plat_prev;
geometry::UltrasonicPosition temp_plat_filtr;
geometry::UltrasonicPosition temp_plat_prev_filtr;


TimeManager tm;

bool first = true;
bool first2 = true;


typedef std::queue<double> queue_t; //it allows to implement FIFO access.
queue_t stack_z;
queue_t stack_z2;
queue_t	dataBuffer;


double integral_z = 0;
double integral_z2 = 0;

double currentValue = 0;

double Pred_P_k_1 = 0.003;

double filtered_distance;

double Xk;

double Pk;


int main(int argc, char **argv) {

	ros::init(argc, argv, "ultrasonic_sensor");

	UltrasonicSensor Sonar1(trigger, echo);

	ros::NodeHandle n;
	
	//sensor data
	ros::Publisher Sensor_pub = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor",50); 
	
	
	//sensor data filtered with one state kalman filter
	ros::Publisher Sensor_pub2 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_Pos_filtr",50);
	
	//sensor data filtered with two states kalman filter	
	ros::Publisher Sensor_pub3 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_Full_filtr",50);
	

	//sensor data filtered with avarage filters
	ros::Publisher Sensor_pub4 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_AvarageFilter",50);

	ros::Rate loop_rate(40);

	//ROS MESSAGES
	ultrasonic_sensor::UltrasonicMsg state;
	ultrasonic_sensor::UltrasonicMsg state_filtr;
	ultrasonic_sensor::UltrasonicMsg Full_state_filtr;
	ultrasonic_sensor::UltrasonicMsg Avaraged_state;
	
	//SETUP FULL KALMAN FILTER
	int stateSize = 2; 
	int measSize = 1;
	int contrSize = 0;
	unsigned int type = CV_32F;	

	//initialize the kalman filter
	cv::KalmanFilter kf(stateSize, measSize, contrSize );

	
	//set dt at each processing step.
	//it initialize MATRIX F
	cv::setIdentity(kf.transitionMatrix); 
	//std::cout<<"matrix F = "<< std::endl << " "<< kf.transitionMatrix<< std::endl << std::endl; 


	//Measure Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1;
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




	//it takes the value of the sensor
	state.z_Position = Sonar1.distance(timeout);

	//it initialize the state value of the kalman
	double Pred_Xk_1 = state.z_Position;

	tm.updateTimer();	
	
	while(ros::ok()){
	
	tm.updateTimer();	



	//BASIC KALMAN FILTER	
    KF(Pred_Xk_1, Pred_P_k_1, state.z_Position, &Xk, &Pk); //kalman filter
	
	state.z_Position = Sonar1.distance(timeout); //measurement
	
	Pred_P_k_1 = Pk; //update of the error
	Pred_Xk_1 = Xk; //update of the state


	//ROS messages for plotting
	state_filtr.z_Position = Xk; 

	///////////////////////////



	// FULL KALMAN FILTER
	kf.transitionMatrix.at<float>(1) = tm._dt; //Updating of matrix F

	cv::Mat Prediction = kf.predict();

	cv:: Mat meas(1,1,type);
	meas = state.z_Position;
	kf.correct(meas); //correction

	Full_state_filtr.z_Position = kf.statePost.at<float>(0); //filtered position

	Full_state_filtr.z_Velocity = kf.statePost.at<float>(1);

	///////////////////////



	// AVARAGE FILTER
	filtered_distance = movingAvarageFilter(state.z_Position, &dataBuffer, &currentValue, 8);

	Avaraged_state.z_Position = filtered_distance;

	////////////////////



	//LCM STUFF TO DO THE VELOCITY

	temp_plat.Z_Position = state.z_Position;
	temp_plat_filtr.Z_Position = state_filtr.z_Position;

	//estimated velocity of non filtered and filtered position
	EstimatedVelocity( &temp_plat, &temp_plat_prev, &stack_z, &first, tm, &integral_z );
	EstimatedVelocity( &temp_plat_filtr, &temp_plat_prev_filtr, &stack_z2, &first2, tm, &integral_z2 );

	//UPDATE VELOCITY

	state.z_Velocity = temp_plat.Z_Velocity;
	state_filtr.z_Velocity = temp_plat_filtr.Z_Velocity;



	Sensor_pub.publish(state);
	Sensor_pub2.publish( state_filtr );	
	Sensor_pub3.publish( Full_state_filtr );
	Sensor_pub4.publish(Avaraged_state);

	ros::spinOnce;
	
	loop_rate.sleep();

	}

}
