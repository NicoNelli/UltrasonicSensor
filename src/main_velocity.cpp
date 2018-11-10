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

/************************************************************
It compares a velocity obtained by a filtered position
and a velocity obtained by the noisy position	

*************************************************************/

#define trigger 23	//pin of the raspberry for trigger signal.
#define echo 24 	//pin of the raspberry for echo signal.
#define timeout 150 //timeout in milliseconds.


//just to connect the ultrasonic module with the mocap2mav
geometry::UltrasonicPosition temp_plat;
geometry::UltrasonicPosition temp_plat_prev;
geometry::UltrasonicPosition temp_plat_filtr;
geometry::UltrasonicPosition temp_plat_prev_filtr;


TimeManager tm;
TimeManager tm2;


bool first = true;
bool first2 = true;


typedef std::queue<double> queue_t; //it allows to implement FIFO access.
queue_t stack_z;
queue_t stack_z2;


double integral_z = 0;
double integral_z2 = 0;


//Parameters of Kalman filter
double Pred_P_k_1 = 0.003;
double Xk;
double Pk;
double Q = 1e-4;
double R = 2.92e-3;

int main(int argc, char **argv) {

	ros::init(argc, argv, "ultrasonic_sensor_basic_kalman_velocity");

	UltrasonicSensor Sonar1(trigger, echo); //it sets the sensor

	ros::NodeHandle n;
	
	//ROS topic
	ros::Publisher Sensor_pub = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor",50);
	ros::Publisher Sensor_pub2 = n.advertise<ultrasonic_sensor::UltrasonicMsg>("UltrasonicSensor_filtr",50);

	ros::Rate loop_rate(40);

	//ROS messages for plotting
	ultrasonic_sensor::UltrasonicMsg state;
	ultrasonic_sensor::UltrasonicMsg state_filtr;

	state.z_Position = Sonar1.distance(timeout);

	double Pred_Xk_1 = state.z_Position;


	tm.updateTimer();	
	tm2.updateTimer();	
	
	while(ros::ok()){

	
	tm.updateTimer();	
	tm2.updateTimer();	

	/*
	F = [1]
	B = [0]
	H = [1]
	Q = [1e-4]
	R = [2,92e-3]
  */
  
    KF(Pred_Xk_1, Pred_P_k_1, state.z_Position, &Xk, &Pk, Q, R); //kalman filter

	//update the covariance of the error and the states of both filters
	Pred_P_k_1 = Pk; 
	Pred_Xk_1 = Xk; 

	//taking the distance from the sensor
	state.z_Position = Sonar1.distance(timeout);
	

	state_filtr.z_Position = Xk; //filtered data


	//LCM STUFF TO DO THE VELOCITY
	temp_plat.Z_Position = state.z_Position; //Noisy position
	temp_plat_filtr.Z_Position = state_filtr.z_Position; //Noisy velocity


	EstimatedVelocity( &temp_plat, &temp_plat_prev, &stack_z, &first, tm, &integral_z );
	EstimatedVelocity( &temp_plat_filtr, &temp_plat_prev_filtr, &stack_z2, &first2, tm2, &integral_z2 );


	//FOR PLOTTING THE VELOCITY
	state.z_Velocity = temp_plat.Z_Velocity;
	state_filtr.z_Velocity = temp_plat_filtr.Z_Velocity;

	Sensor_pub.publish(state);
	Sensor_pub2.publish( state_filtr );


	ros::spinOnce;
	
	loop_rate.sleep();

	}

}
