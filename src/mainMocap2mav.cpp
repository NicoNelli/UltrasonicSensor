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
#include "MedianFilter.h"

/************************************************************
FILE TO COMMUNICATE SENSOR DATA WITH MOCAP2MAV

*************************************************************/

#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.


//just to connect the ultrasonic module with the mocap2mav
lcm::LCM handler; 
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



//for computing the velocity
double integral_z = 0;
double integral_z2 = 0;


//Parameters of Kalman filter
double Pred_P_k_1 = 0.003;
double Xk;
double Pk;
double Q = 0.5*1e-3;
double R = 2.92e-3;


int main(int argc, char **argv) {

	ros::init(argc, argv, "ultrasonic_sensor_basic_kalman_velocity");

	UltrasonicSensor Sonar1(trigger, echo); //it sets the sensor

	ros::NodeHandle n;
	
	//ROS topic
	//ros::Publisher Sensor_pub = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor",1);

	ros::Publisher Sensor_pub2 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_filtr",1);

	ros::Rate loop_rate(30);

	//ROS messages for plotting
	ultrasonic_sensor::UltrasonicMsg state;
	ultrasonic_sensor::UltrasonicMsg state_filtr;

	state.z_Position = Sonar1.distance(timeout);

	double Pred_Xk_1 = state.z_Position;

	MedianFilter<double, 5> f;

	tm.updateTimer();	
	
	while(ros::ok()){

	tm.updateTimer();	


	/*
	F = [1]
	B = [0]
	H = [1]
	Q = [0.5*1e-3]
	R = [2,92e-3]
  */
  
    KF(Pred_Xk_1, Pred_P_k_1, state.z_Position, &Xk, &Pk, Q, R); //kalman filter

	//update the covariance of the error and the states of both filters
	Pred_P_k_1 = Pk; 
	Pred_Xk_1 = Xk; 

	//taking the distance from the sensor
	state.z_Position = Sonar1.distance(timeout);
	
	//std::cout<<"" <<Sonar1.GetValidity()<< std::endl;	

	state_filtr.z_Position = Xk; //filtered data
	temp_plat_filtr.isValid = Sonar1.GetValidity();

	//LCM STUFF TO DO THE VELOCITY
	//temp_plat.Z_Position = state.z_Position; //Noisy position

	temp_plat_filtr.Z_Position = state_filtr.z_Position; //kalman position


	//velocity raw data
	//EstimatedVelocity( &temp_plat, &temp_plat_prev, &stack_z, &first, tm, &integral_z );
	
	//velocity after kalman filter
	EstimatedVelocity( &temp_plat_filtr, &temp_plat_prev_filtr, &stack_z2, &first2, tm, &integral_z2 );

	f.addSample(temp_plat_filtr.Z_Velocity);
	

	//FOR PLOTTING THE VELOCITY
	//state.z_Velocity = temp_plat.Z_Velocity;

	if( f.isReady() )
		state_filtr.z_Velocity = f.getMedian();

	else
		state_filtr.z_Velocity = 0;

	//Sensor_pub.publish(state);
	Sensor_pub2.publish( state_filtr );

	handler.publish("UltrasonicSensor/platform",&temp_plat_filtr);

	ros::spinOnce();
	
	loop_rate.sleep();

	}

}
