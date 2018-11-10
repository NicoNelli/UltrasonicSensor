#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#include <cmath>

#define SpeedOfSound 343.00

UltrasonicSensor::UltrasonicSensor( int trigger, int echo ) {

	wiringPiSetupGpio();
	//it allows the calling program to use broadcom GPIO pins numbers directly
	//with no re-mapping.

	this->trigger = trigger; //set TRIG pin

	this->echo = echo; // set ECHO pin

	pinMode( trigger, OUTPUT ); //set TRIG pin as output

	pinMode( echo, INPUT ); //set ECHO pin as input

	//ensure that, initially, the TRIG pin is set low, and give some time to
	// settle the sensor.
	digitalWrite( trigger, LOW ); 

	delay(500); //delay in milliseconds.
	
}

//wait until the echo pin is not high (polling method) 
void UltrasonicSensor::TimeOfTravel() {

	startTimeUsec = micros();
	
	while ( digitalRead(echo) == HIGH );
	
	endTimeUsec = micros();

}

//it checks if the timeout is expired
bool UltrasonicSensor::isExpired( int timeout, long time ) { 

isValid = ( micros() - time )/1000.0 < timeout;

return isValid;
}


double UltrasonicSensor::distance( int timeout) {
/*
firstly, transmit at least 10us high level pulse to the trigger pin.
the sensor requires a short 10us pulse to trigger the module.
these three functions do this for us.
*/

	delay(10);

	digitalWrite( trigger, HIGH );

	delayMicroseconds(20);

	digitalWrite( trigger, LOW);


	//taking the actual time.
	now = micros();    
	
	
	//When there is the rising edge of the echo pin(from low to high, it means that the waves are travelling)
	//the AND relationship with the timeout allows not to make the function blocking.
	while ( digitalRead(echo) == LOW && isExpired( timeout,now ) );	
	TimeOfTravel();

	travelTimeUsec = endTimeUsec - startTimeUsec;
	
	distanceMeters = ( (travelTimeUsec/1000000.0)*SpeedOfSound )/2; //It computes the distance

	return distanceMeters;	

}


bool UltrasonicSensor::GetValidity() const {

return isValid;
}


void mean(double& sum,double plus,double minus){

    sum += plus - minus;

}


void EstimatedVelocity(geometry::UltrasonicPosition *Actual_plat, geometry::UltrasonicPosition *Prev_plat, std::queue<double> *queue_t, bool *flag, TimeManager& time, double *int_z ) {


	int window = 6; //avarage filter with a windows of 6

	if( *flag ) {
		
		Actual_plat->Z_Velocity = 0;
		Actual_plat->Z_Velocity = 0;
		Actual_plat->Z_Velocity = 0;
		*flag = false;
	}
	else {
	

		double dz = ( round( Actual_plat->Z_Position * 1000 ) / 1000 ) - ( round( Prev_plat->Z_Position * 1000 ) / 1000 );
		double vz = dz/( time._dt );
		
		//DEBUG:
		//std::cout<<"dz: "<<dz<<std::endl;
		//std::cout<<"vz: "<<vz<<std::endl;
		//std::cout<<"time: "<<time._dt<<std::endl;


		//mean for smoothing
		if (queue_t->size() == window) {
            
			mean(*int_z , vz, queue_t->front()); //return the oldest element of the FIFO queue
			queue_t->pop(); //it removes the oldest method.
        
		}else
            mean(*int_z,vz,0);		


		queue_t->push(vz);
		Actual_plat->Z_Velocity = *int_z/queue_t->size();
	}

	Prev_plat->Z_Position = Actual_plat->Z_Position;

}



void KF(double Pred_Xk_1, double Pred_P_k_1,double  Zk, double *Xk, double *Pk,double Q, double R){
/**
Pred_Xk = F * Pred_Xk_1

Pred_Pk = F * Pred_P_k_1 * F^t + Q

Kk = Pred_Pk * H^t(H * Pred_Pk * H^t + R)^(-1)

Xk = Pred_Xk + Kk * (Zk - H * Pred_Xk)

Pk = (I - Kk * H) * Pred_Pk 

In this case:

F = [1]

B = [0]

H = [1]

Q = [1e-4]

R = [2,92e-3]

**/

double F = 1;
double H = 1;
//double Q = 1e-4;
//double R = 2.92e-3;


double Pred_Xk = F * Pred_Xk_1;

double Pred_Pk = F * Pred_P_k_1 * F + Q;

double Kk = Pred_Pk * H * ( 1/(H * Pred_Pk * H + R ) );

 *Xk = Pred_Xk + Kk * (Zk - H * Pred_Xk);

 *Pk = (1 - Kk * H) * Pred_Pk;


}


double movingAvarageFilter(double distance, std::queue<double> *queue_t, double *currentValue, int window) {

/*
--distance: value obtained by the sensor

--buffer: set of value of the moving avarage filter

--window: number of value to consider

--currentValue: current avarage value
*/

if (queue_t->size() == window) {
            
	mean( *currentValue , distance, queue_t->front() ); 
	queue_t->pop(); //it removes the oldest method.
        
}else
    mean( *currentValue, distance,0 );		

	
		queue_t->push(distance);
		return *currentValue/queue_t->size();
	}




