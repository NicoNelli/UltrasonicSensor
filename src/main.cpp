#include <iostream>
#include <wiringPi.h>
#include "UltrasonicSensor.h"
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/UltrasonicPosition.hpp"
#include "utils/TimeHelpers.hpp"
#include <queue>
#include <cmath>
#include <pthread.h>
#include <sys/time.h>

#define SpeedOfSound 343.00

Ultrasonic_Sensor Sonar1;

bool distanceComputed = false; 

bool previousState;

//Initialize
pthread_cond_t distancePerformed = PTHREAD_COND_INITIALIZER;
pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;

long count = 0;

void Interrupt_Handler(void) {
	unsigned int t = micros();
	count++;

	if ( previousState == false){
		Sonar1.startTimeUsec = t;
		previousState = true;		
		
	} else if(  previousState == true){
			Sonar1.endTimeUsec = t;
			previousState = false;
			distanceComputed = true;
	
			pthread_mutex_lock(&m);	
			Sonar1.travelTimeUsec = Sonar1.endTimeUsec - Sonar1.startTimeUsec;
			Sonar1.distanceMeters = ( (Sonar1.travelTimeUsec/1000000.0)*SpeedOfSound )/2;			
			
			pthread_cond_signal(&distancePerformed);		
			pthread_mutex_unlock(&m);

	}


}


TimeManager tm;

bool first = true;

struct timeval tv;
struct timespec ts;

int main() {

	double localDist;
	bool done = Init(&Sonar1);

	if (done) { //if the initialization goes well
		
		wiringPiISR(Sonar1.echo, INT_EDGE_BOTH, Interrupt_Handler); // set interrupt for both raising edge
	


		while(1) {

				previousState = false;
				
				Init_distance(&Sonar1);

				struct timeval tv;
				struct timespec ts;

				gettimeofday(&tv, NULL);
				ts.tv_sec = time(NULL) + 150 / 1000;
				ts.tv_nsec = tv.tv_usec*1000 + 1000*1000 * (150 % 1000);
				ts.tv_sec += ts.tv_nsec / (1000 * 1000 * 1000);
				ts.tv_nsec %= (1000 * 1000 * 1000);
				

				pthread_mutex_lock(&m);	
				pthread_cond_timedwait(&distancePerformed, &m, &ts);
				pthread_mutex_unlock(&m);
	

				std::cout<<"distance: "<<Sonar1.distanceMeters<<std::endl;								

				distanceComputed = false;
			
				usleep(100*1000); //10 Hz
		}
	


	}
	

}
