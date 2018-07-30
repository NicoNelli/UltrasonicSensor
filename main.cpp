#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"

int trigger = 23;

int echo = 24;

int timeout = 100; //timeout in milliseconds.

int main() {

	UltrasonicSensor Sonar1(trigger, echo);
	
	long now = micros();    
	
	std::cout<<"distance in meters: "<<Sonar1.distance(timeout)<<std::endl;	

	std::cout<<"Time necessary: "<<(micros()-now)/1000000.0<<" s"<<std::endl;	

	std::cout<<"is Valid the measure? "<<Sonar1.GetValidity()<<std::endl;
	
}
