#include <Arduino.h>
//#include <iostream>
#include "CollisionAvoidanceStrategy.h"
#include "Action.h"

using namespace std;

Action* CollisionAvoidanceStrategy::getHighestYieldingAction(const funcSensor* sensors,BMap* map, int curOr, int curXBel, int curYBel, int tProb){

//    cout << "getting highest yeilding action \n";

    int testReading0 = (*sensors[S0])(0);
    int testReading90 = (*sensors[S90])(0);
    int testReading180 = (*sensors[S180])(0);
    int testReading270 = (*sensors[S270])(0);

//    cout << "test read of sensor array 0 " << testReading0 << "\n";
//    cout << "test read of sensor array 90 " << testReading90 << "\n";
//    cout << "test read of sensor array 180 " << testReading180 << "\n";
//    cout << "test read of sensor array 270 " << testReading270 << "\n";

    int* vds =(int*) malloc(sizeof(int) * 4); 
    int svds = 0;

    int test = 10;

    Action* next = new Action();
    next->c = ActionTypes::NOP;
    next->m = 0;

    curOr = (*sensors[OR0])(0);

    int COLLISION_INTERCEPT = COLLISION_THRESHOLD + STRIDE;

    if(((*sensors[S0])(0)) > COLLISION_INTERCEPT 
		    && ( ((*sensors[S0])(0)) < MAX_RANGE ||  ((*sensors[S180])(0)) < MAX_RANGE )) {
    	*(vds+svds) = curOr;
    	svds++;
    }
   
    if(((*sensors[S90])(0)) > COLLISION_INTERCEPT 
		    && ( ((*sensors[S90])(0)) < MAX_RANGE ||  ((*sensors[S270])(0)) < MAX_RANGE )) {
	    *(vds+svds) = (curOr + 90) % 360;
	    svds++;
    }

    if(((*sensors[S180])(0)) > COLLISION_INTERCEPT
		    && ( ((*sensors[S180])(0)) < MAX_RANGE ||  ((*sensors[S0])(0)) < MAX_RANGE )) {
	    *(vds+svds) = (curOr + 180) % 360;
	    svds++;
    }
    
    if(((*sensors[S270])(0)) > COLLISION_INTERCEPT
		    && ( ((*sensors[S270])(0)) < MAX_RANGE ||  ((*sensors[S90])(0)) < MAX_RANGE )) {
	    *(vds+svds) = (curOr + 270) % 360;
	    svds++;
    }
//printf("svds:%d\n",svds);
#if ARDUINO == 1

  int r = random(1,101);
  
  Serial.print("r: ");
  Serial.print(r);
  Serial.print(", tProb: ");
  Serial.println(tProb);

    if(r/*(random(1,101))*/ < tProb) {
        int index = random(0,svds);
#else
    if((rand() % 100) < tProb) {
    	int index = rand() % svds;
#endif	    
	next->c = ActionTypes::TURN;
   	next->m = *(vds+index);
    } else {

	int i;

        int diff=*(vds+i)-curOr;
	// this is redundant
	for(i=0; i<svds;i++) {
//		if(*(vds+i) == curOr) {
        	
          if(abs(diff)<OREINTATION_THRESHOLD){
            next->c = ActionTypes::MOVE;
            next->m = STRIDE;
            break;
          }
//		}
	}
    }













    free(vds);
    
    return next;
}

