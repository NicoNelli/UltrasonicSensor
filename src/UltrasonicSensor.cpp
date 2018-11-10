#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#define SpeedOfSound 343.00

bool Init( Ultrasonic_Sensor * Sonar ) {

	wiringPiSetupGpio();
	//it allows the calling program to use broadcom GPIO pins numbers directly
	//with no re-mapping.

	Sonar->trigger = TRIGGER_PIN; 

	Sonar->echo = ECHO_PIN;
	
	Sonar->timeout = TIMEOUT;

	pinMode( Sonar->trigger, OUTPUT ); //set TRIG pin as output

	pinMode( Sonar->echo, INPUT ); //set ECHO pin as output

	//ensure that, initially, the TRIG pin is set low, and give some time to
	// settle the sensor.
	digitalWrite( Sonar->trigger, LOW ); 

	delay(500); //delay in milliseconds.
	
	return true;
}


void TimeOfTravel( Ultrasonic_Sensor * Sonar ) {

	Sonar->startTimeUsec = micros();
	
	while ( digitalRead(Sonar->echo) == HIGH );
	
	Sonar->endTimeUsec = micros();

}

bool isExpired( Ultrasonic_Sensor * Sonar , long time ) { 
//it checks if the timeout is expired. 	

bool timerExpired = ( micros() - time )/1000.0 > Sonar->timeout;

Sonar->isValid = timerExpired;

return timerExpired;
}

void Init_distance( Ultrasonic_Sensor * Sonar ) {

	//firstly, transmit at least 10us high level pulse to the trigger pin.
	//the sensor requires a short 10us pulse to trigger the module.
	//these three functions do this for us.
	delay(10);

	digitalWrite( Sonar->trigger, HIGH );

	delayMicroseconds(20);

	digitalWrite( Sonar->trigger, LOW);

	//taking the actual time.
	
	//The sensor sets ECHO to high for the amount of time it takes for the pulse to go and 		come back, so it is necessary to measure the amount of time that the ECHO pin stays high.

}


bool GetValidity( Ultrasonic_Sensor * Sonar ) {

return Sonar->isValid;
}


void mean(double& sum,double plus,double minus){

    sum += plus - minus;

}


void EstimatedVelocity(geometry::UltrasonicPosition *Actual_plat, geometry::UltrasonicPosition *Prev_plat, std::queue<double> *queue_t, bool *flag, const TimeManager& time, double *int_z ) {

	int window = 6;

	if( *flag ) {
		
		Actual_plat->Z_Velocity = 0;
		Actual_plat->Z_Velocity = 0;
		Actual_plat->Z_Velocity = 0;
		*flag = false;
	}
	else {
		
		double dz = Actual_plat->Z_Position - Prev_plat->Z_Position;
		double vz = dz/time._dt;
		
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

















