#include <iostream>
#include "ros/ros.h"
#include "ultrasonic_sensor/UltrasonicMsg.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <cmath>

/************************************************************
It compares with statical indexes velocity obtained by a filtered position
and a velocity obtained by the noisy position and the motion capture data
and velocity obtained with an avarage filter

*************************************************************/

std::vector<double> mocapValue;

std::vector<double> measValue;

std::vector<double> filtrValue; //final value of Q

std::vector<double> avarageFiltrValue; 

//the error is defined as the difference between the true value(mocap) and the data acquired from the sensor.
double measError = 0;

double filtrError = 0;

double avarageFiltrError = 0;

double measVariance = 0;

double filtrVariance = 0;

double avarageFiltrVariance = 0;

void odometryCallback(nav_msgs::Odometry pose){

	 mocapValue.push_back(pose.twist.twist.linear.z);

   //pose.twist.twist.linear.z;
}

void sensorCallback(ultrasonic_sensor::UltrasonicMsg data){

     measValue.push_back(data.z_Velocity);

}

void sensorFiltrcallback(ultrasonic_sensor::UltrasonicMsg filtrData){

     filtrValue.push_back(filtrData.z_Velocity);

}

void avarageFiltrcallback(ultrasonic_sensor::UltrasonicMsg avarfiltrData){

     avarageFiltrValue.push_back(avarfiltrData.z_Velocity);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "validation");

	ros::NodeHandle n;

	ros::Subscriber Sensor_sub = n.subscribe("UltrasonicSensor",1,&sensorCallback);

	//It publishes the distance obtained from the sensor filtered with Kalman filter, Q
	ros::Subscriber Sensor_filtr_sub = n.subscribe("UltrasonicSensor_filtr",1,&sensorFiltrcallback);

	//motion capture data	
	ros::Subscriber odometry_sub = n.subscribe("/mavros/local_position/odom",1,&odometryCallback);
	

	ros::Subscriber avarageFiltr_sub = n.subscribe("UltrasonicSensor_AvarageFilter",1,&avarageFiltrcallback);

	ros::Rate loop_rate(30);

	int count = 0;

	while( count < 500 ){


	ros::spinOnce();
	loop_rate.sleep();

	count++;

	}


	/////MEAN ERROR
	for(int i=0; i<mocapValue.size();i++){

		measError += mocapValue[i] - measValue[i];

		filtrError += mocapValue[i] - filtrValue[i];
	
		avarageFiltrError += mocapValue[i] - avarageFiltrValue[i];

	}	
		
		avarageFiltrError = avarageFiltrError/avarageFiltrValue.size();

		measError = measError/mocapValue.size();

		filtrError = filtrError/filtrValue.size();
///////



	/*
	std::cout<<"meas: " << std::endl;
	for(int i=0; i<measValue.size(); i++)
		std::cout<<" " << measValue[i];

	std::cout<<std::endl;
		
	std::cout<<"filtr: " << std::endl;
	for(int i=0; i<filtrValue.size(); i++)
		std::cout<<" " << filtrValue[i];

	std::cout<<std::endl;


	


	//std::cout<<"filtr2: " << std::endl;
	//for(int i=0; i<filtr2Value.size(); i=i+4)
	//	std::cout<<" " << filtr2Value[i];

	//std::cout<<std::endl;
*/

	/////VARIANCE
	for(int i=0; i< measValue.size(); i++ ){

		measVariance += (measValue[i] - measError)*(measValue[i] - measError);

		filtrVariance += (filtrValue[i] - filtrError)*(filtrValue[i] - filtrError);

		avarageFiltrVariance += (avarageFiltrValue[i] - avarageFiltrError)*(avarageFiltrValue[i] - avarageFiltrError);
	}


	avarageFiltrVariance = sqrt( avarageFiltrVariance/avarageFiltrValue.size()*100 );

	measVariance =  sqrt( measVariance/measValue.size()*100 );

	filtrVariance =  sqrt( filtrVariance/measValue.size()*100 );


	std::cout<<"measMeanVel: " << measError*100 <<std::endl;

	std::cout<<"filtrMeanVel: " << filtrError*100 <<std::endl;

	std::cout<<"avarageFilterMeanVel: " << avarageFiltrError*100 <<std::endl;

	std::cout<<"measVarianceVel: " << measVariance <<std::endl;

	std::cout<<"filtrVarianceVel: " << filtrVariance <<std::endl;

	std::cout<<"AvarageFilterVarianceVel: " << avarageFiltrVariance <<std::endl;









}
