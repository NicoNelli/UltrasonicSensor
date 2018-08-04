#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"

#define SpeedOfSound 343.00


UltrasonicSensor::UltrasonicSensor( int trigger, int echo ) {

	wiringPiSetupGpio();
	//it allows the calling program to use broadcom GPIO pins numbers directly
	//with no re-mapping.

	this->trigger = trigger; //set TRIG pin

	this->echo = echo; // set ECHO pin

	pinMode( trigger, OUTPUT ); //set TRIG pin as output

	pinMode( echo, INPUT ); //set ECHO pin as output

	//ensure that, initially, the TRIG pin is set low, and give some time to
	// settle the sensor.
	digitalWrite( trigger, LOW ); 

	delay(500); //delay in milliseconds.
	
}

void UltrasonicSensor::TimeOfTravel() {

	startTimeUsec = micros();
	
	while ( digitalRead(echo) == HIGH );
	
	endTimeUsec = micros();

}

bool UltrasonicSensor::isExpired( int timeout, long time ) { 
//it checks if the timeout is expired. 	

isValid = ( micros() - time )/1000.0 < timeout;

return isValid;
}

double UltrasonicSensor::distance( int timeout) {

	//firstly, transmit at least 10us high level pulse to the trigger pin.
	//the sensor requires a short 10us pulse to trigger the module.
	//these three functions do this for us.
	delay(10);

	digitalWrite( trigger, HIGH );

	delayMicroseconds(20);

	digitalWrite( trigger, LOW);

	//taking the actual time.
	now = micros();    
	
	//The sensor sets ECHO to high for the amount of time it takes for the pulse to go and 		come back, so it is necessary to measure the amount of time that the ECHO pin stays high.
	while ( digitalRead(echo) == LOW && isExpired( timeout,now ) );	
	TimeOfTravel();


	travelTimeUsec = endTimeUsec - startTimeUsec;
	
	distanceMeters = ( (travelTimeUsec/1000000.0)*SpeedOfSound )/2;

	return distanceMeters;	

}


bool UltrasonicSensor::GetValidity() const {

return isValid;
}



















