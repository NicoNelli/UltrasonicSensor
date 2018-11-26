#include <iostream>
#include "ros/ros.h"
#include "ultrasonic_sensor/UltrasonicMsg.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <cmath>

/************************************************************
It compares with statical indexes position obtained by a filtered position
and a position obtained by the noisy position and the motion capture data
and position obtained by avarage filter

*************************************************************/

std::vector<double> mocapValue;

std::vector<double> measValue;

std::vector<double> filtrValue; //Q

std::vector<double> filtr2Value; //Q2

std::vector<double> avarageFiltrValue; 

//the error is defined as the difference between the true value(mocap) and the data acquired from the sensor.

double measError = 0;

double filtrError = 0;

double filtr2Error = 0;

double avarageFiltrError = 0;

double measVariance = 0;

double filtrVariance = 0;

double filtr2Variance = 0;

double avarageFiltrVariance = 0;

void odometryCallback(nav_msgs::Odometry pose){

     mocapValue.push_back(pose.pose.pose.position.z);

}

void sensorCallback(ultrasonic_sensor::UltrasonicMsg data){

     measValue.push_back(0.1+data.z_Position);

}

void sensorFiltrcallback(ultrasonic_sensor::UltrasonicMsg filtrData){

     filtrValue.push_back(0.1+filtrData.z_Position);

}

void sensorFiltr2callback(ultrasonic_sensor::UltrasonicMsg filtr2Data){

     filtr2Value.push_back(0.1+filtr2Data.z_Position);

}

void avarageFiltrcallback(ultrasonic_sensor::UltrasonicMsg avarfiltrData){

     avarageFiltrValue.push_back(0.1+avarfiltrData.z_Position);

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "validation");

	ros::NodeHandle n;

	ros::Subscriber Sensor_sub = n.subscribe("UltrasonicSensor",1,&sensorCallback);

	//It publishes the distance obtained from the sensor filtered with Kalman filter, Q
	ros::Subscriber Sensor_filtr_sub = n.subscribe("UltrasonicSensor_filtr",1,&sensorFiltrcallback);

	//It publishes the distance obtained from the sensor filtered with Kalman filter, Q2
	ros::Subscriber Sensor_filtr_sub2 = n.subscribe("UltrasonicSensor_filtr2",1,&sensorFiltr2callback);

	ros::Subscriber avarageFiltr_sub = n.subscribe("UltrasonicSensor_AvarageFilter",1,&avarageFiltrcallback);

	//motion capture data	
	ros::Subscriber odometry_sub = n.subscribe("/mavros/local_position/odom",1,&odometryCallback);
	
	ros::Rate loop_rate(30);

	int count = 0;

	while( count < 500 ){


	ros::spinOnce();
	loop_rate.sleep();

	count++;

	}


	/////MEAN ERROR & ERROR MAX
	for(int i=0; i<mocapValue.size();i++){

		measError += mocapValue[i] - measValue[i];

		filtrError += mocapValue[i] - filtrValue[i];

		filtr2Error += mocapValue[i] - filtr2Value[i];

		avarageFiltrError += mocapValue[i] - avarageFiltrValue[i];

	}	

		avarageFiltrError = avarageFiltrError/avarageFiltrValue.size();

		measError = measError/mocapValue.size();

		filtrError = filtrError/filtrValue.size();

		filtr2Error = filtr2Error/filtr2Value.size();
	/////////////////



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

		filtr2Variance += (filtr2Value[i] - filtr2Error)*(filtr2Value[i] - filtr2Error);

		avarageFiltrVariance += (avarageFiltrValue[i] - avarageFiltrError)*(avarageFiltrValue[i] - avarageFiltrError);

	}

	measVariance =  sqrt( measVariance/measValue.size()*100 );

	filtrVariance =  sqrt( filtrVariance/filtrValue.size()*100 );

	filtr2Variance = sqrt( filtr2Variance/filtr2Value.size()*100 );

	avarageFiltrVariance = sqrt( avarageFiltrVariance/avarageFiltrValue.size()*100 );


	std::cout<<"measMeanPos: " << measError*100 <<std::endl;

	std::cout<<"filtrMeanPos: " << filtrError*100 <<std::endl;

	std::cout<<"filtr2MeanPos: " << filtr2Error*100 <<std::endl;

	std::cout<<"avarageFilterMeanPos: " << avarageFiltrError*100 <<std::endl;

	std::cout<<"measVariancePos: " << measVariance <<std::endl;

	std::cout<<"filtrVariancePos: " << filtrVariance <<std::endl;

	std::cout<<"filtr2VariancePos: " << filtr2Variance <<std::endl;

	std::cout<<"AvarageFilterVariancePos: " << avarageFiltrVariance <<std::endl;







}
