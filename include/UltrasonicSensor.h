#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "lcm_messages/geometry/pose.hpp"
#include "lcm/lcm-cpp.hpp"
#include <queue>
#include "utils/TimeHelpers.hpp"
class UltrasonicSensor {

public:

	UltrasonicSensor( int trigger, int echo );	
	
	double distance( int timeout ); //It computes the distance

	bool GetValidity() const; //It checks if the result is valid(based on the timeout expired )	

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

/*It allows to estimate the velocity based on two measurements.
Parameters:
1) z position of the platform
2) previous z position of the platform
3) FIFO queue.
4) flag to control the first time the function is used.
5) time object to obtain dt for computing the temporal derivative. 
*/
void EstimatedVelocity(geometry::pose *Actual_plat, geometry::pose *Prev_plat, std::queue<double> *queue_t, bool *flag, const TimeManager& time, double *int_z);

//mean value.
void mean(double& sum,double plus,double minus);
#endif
 
















