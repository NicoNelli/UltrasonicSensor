#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "lcm_messages/geometry/UltrasonicPosition.hpp"
#include "lcm/lcm-cpp.hpp"
#include <queue>
#include "utils/TimeHelpers.hpp"

#define TRIGGER_PIN 23	//pin of the raspberry for trigger signal.

#define ECHO_PIN 24 	//pin of the raspberry for echo signal.

#define TIMEOUT 150 //timeout in milliseconds.

struct Ultrasonic_Sensor{

int trigger;

int echo;

int timeout;

volatile long startTimeUsec;

volatile long endTimeUsec;

double distanceMeters;

long travelTimeUsec;

bool isValid; //timeout is expired? if yes, the data are not valid.

};


bool Init( Ultrasonic_Sensor * Sonar );	
	
void Init_distance( Ultrasonic_Sensor * Sonar ); //It computes the distance

bool GetValidity( Ultrasonic_Sensor * Sonar ); //It checks if the result is valid(based on the timeout expired )	

void MyInterrupt(void);

bool isExpired( Ultrasonic_Sensor * Sonar, long time ); //it checks if the timeout is expired. 	

void TimeOfTravel(  Ultrasonic_Sensor * Sonar ); 

#endif
 
















