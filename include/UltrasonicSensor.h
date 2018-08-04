#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

class UltrasonicSensor {

public:

	UltrasonicSensor( int trigger, int echo );	
	
	double distance( int timeout );

	bool GetValidity() const;	

private:

	bool isExpired( int timeout, long time ); //it checks if the timeout is expired. 	

	void TimeOfTravel(); 
	//It computes the the amount of time it takes for the pulse to go and come back.

	int trigger; //trigger pin

	int echo; //echo pin

	volatile long startTimeUsec;

	volatile long endTimeUsec;

	double distanceMeters;

	long travelTimeUsec;

	long now;

	bool isValid; //timeout is expired? if yes, the data are not valid.
};



#endif
 
