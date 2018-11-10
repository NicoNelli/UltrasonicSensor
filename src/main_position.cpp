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

#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.

/************************************************************
It compares two position filtered by two Kalman filters
 with one state variable which is the distance obtained by the sensor.							

*************************************************************/

double Pred_P_k_1 = 0.003;
double Pred2_P_k_1 = 0.003;


double Xk;
double Pk;
double Q = 1e-4;
double R = 2.92e-3;



double Xk2;
double Pk2;
double Q2 = 1e-2;
double R2 = 2.92e-3;


int main(int argc, char **argv) {

	ros::init(argc, argv, "ultrasonic_sensor_filtered_basic_kalman");

	UltrasonicSensor Sonar1(trigger, echo); //it sets the sensor

	ros::NodeHandle n;
	
	//It publishes the distance obtained from the sensor
	ros::Publisher Sensor_pub = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor",50);

	//It publishes the distance obtained from the sensor filtered with Kalman filter
	ros::Publisher Sensor_pub2 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_filtr",50);

	//It publishes the distance obtained from the sensor filtered with Kalman filter
	ros::Publisher Sensor_pub3 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_filtr2",50);

	//rate of filtering and for the sensor
	ros::Rate loop_rate(40);


	//The three ROS messages
	ultrasonic_sensor::UltrasonicMsg state;
	ultrasonic_sensor::UltrasonicMsg state_filtr;
	ultrasonic_sensor::UltrasonicMsg state_filtr2;

	//it takes the distance from the sensor
	state.z_Position = Sonar1.distance(timeout);

	//initialize the kalman filter
	double Pred_Xk_1 = state.z_Position;
	double Pred2_Xk_1 = state.z_Position;
	
	
	while(ros::ok()){
	

/*
	F = [1]
	B = [0]
	H = [1]
	Q = [1e-4]
	R = [2,92e-3]
  */
  
	KF(Pred_Xk_1, Pred_P_k_1, state.z_Position, &Xk, &Pk, Q, R); //kalman filter
	KF(Pred2_Xk_1, Pred2_P_k_1, state.z_Position, &Xk2, &Pk2, Q2, R2); //kalman filter


	//update the covariance of the error and the states of both filters
	Pred2_P_k_1 = Pk2;
	Pred2_Xk_1 = Xk2;

	Pred_P_k_1 = Pk; 
	Pred_Xk_1 = Xk; 


	//measurement output
	state.z_Position = Sonar1.distance(timeout);
	
	//update of the state
	state_filtr.z_Position = Xk; 
	state_filtr2.z_Position = Xk2;


	//PUBLISH
	Sensor_pub.publish( state );
	Sensor_pub2.publish( state_filtr );
	Sensor_pub3.publish( state_filtr2 );

	ros::spinOnce;
	
	loop_rate.sleep();

	}
	
}
