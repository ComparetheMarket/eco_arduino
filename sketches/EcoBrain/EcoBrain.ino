#include <EcoSPI.h>
#include <EcoSensors.h>
#include "thresholds.h"
#include "StrategyFactory.h"
#include "WorkingMemory.h"
#include "sensors.h"
#include "pins_arduino.h"

#define BRAIN_READY              9

// what to do with incoming data
volatile byte command = 0;
volatile int byteCount = 0;
byte* b;

volatile boolean calculate = false;
volatile byte* brainCommand;

byte brainCommandCount = 0;
byte** brainCommands;

boolean firstBelief = true;

BMap* beliefs = new BMap();
funcSensor sensors[NUMSNSRS];                                                                 
StrategyFactory* sf = new StrategyFactory();
IStrategy* s = sf->makeStrategy(StrategyTypes::COLLISION_AVOIDANCE); 
WorkingMemory* wm = new WorkingMemory();
ActionTypes::Enum lastAction = ActionTypes::NOP;

EcoSensors es;

void agent_take_reading(funcSensor* sensors, WorkingMemory* wm, ActionTypes::Enum act) {
    struct LocF* fact = new LocF();

    fact->s0 = ((*sensors[S0])(0));
    fact->s90 = ((*sensors[S90])(0));
    fact->s180 = ((*sensors[S180])(0));
    fact->s270 = ((*sensors[S270])(0));
    fact->ori = ((*sensors[OR0])(0));
    fact->precact = act;

    Serial.println(fact->s0);
    Serial.println(fact->s90);
    Serial.println(fact->s180);
    Serial.println(fact->s270);
    Serial.println(fact->ori);
    Serial.println(fact->precact);

//    printf("sensor reading %d\n", fact->s0);

    // DEBUG - encapsulate
    wm->AssertFact(fact);
}

// start of transaction, no command yet
void ss_falling ()
{
//  digitalWrite(BRAIN_READY, HIGH);
  
  calculate = false;
  
  command = 0;
  byteCount = 0;
//  brainCommandIndex++;
//  brainCommandByteIndex = 0;
  EcoSPI::getBytes(0, b);
}  // end of interrupt service routine (ISR) ss_falling

void setup (void)
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("EcoBrain 10");
  // setup pins
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  pinMode(BRAIN_READY, OUTPUT);
  digitalWrite(BRAIN_READY, HIGH);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

  // interrupt for SS falling edge
  attachInterrupt (0, ss_falling, FALLING);
  
  sensors[S0] = &read_0_sonic_real; //&read_within_range;
  sensors[S90] = &read_90_sonic_real; //&read_within_range;
  sensors[S180] = &read_180_sonic_real; //&read_within_range;                                                           
  sensors[S270] = &read_270_sonic_real; //&read_within_range;                                                           
  sensors[T0] = &read_within_range;
  sensors[OR0] = &read_orientation_real; //&read_orientation_is_0;
}  // end of setup

void loop (void)
{
  if (calculate) {
    digitalWrite(BRAIN_READY, HIGH);
    calculate = false;
    
//    Serial.println("Let's calculate it");
    
    setSensors(es);
    
    if (firstBelief) {
      agent_take_reading(sensors, wm, ActionTypes::NOP);
      firstBelief = false;
    }
    agent_take_reading(sensors, wm, lastAction);
    
    Action* action = s->getHighestYieldingAction(sensors, beliefs, 0, 0, 0, TURN_PROBABILITY);
        
    lastAction = action[0].c;
    
    Serial.println(action[0].c);
        
    byte* cmd;
    byte* data = new byte[4];
    
    switch (action[0].c) {
    case ActionTypes::TURN:
      cmd = new byte[6];
      EcoSPI::getBytes(action[0].m, data);
      cmd[0] = FACE_ROBOT;
      cmd[1] = (byte) 4;
      cmd[2] = data[0];
      cmd[3] = data[1];
      cmd[4] = data[2];
      cmd[5] = data[3];
      break;
    case ActionTypes::MOVE:
      cmd = new byte[6];
      EcoSPI::getBytes((long) action[0].m, data);
      cmd[0] = MOVE_ROBOT;
      cmd[1] = (byte) 4;
      cmd[2] = data[0];
      cmd[3] = data[1];
      cmd[4] = data[2];
      cmd[5] = data[3];
      break;
    case ActionTypes::NOP:
      cmd = new byte[2];
      cmd[0] = END_OF_COMMANDS;  
      cmd[1] = (byte) 0;
      break;
    }
    
    writeCommand(cmd);
        
    delete action;
    
//    print_belief_map(beliefs);
    
    digitalWrite(BRAIN_READY, LOW);
  }
  
//      digitalWrite(BRAIN_READY, LOW);
}  // end of loop

//// draw a i 50*50 grid centred on current location and plot the readings
//// DO NOT WANT THIS IN THE REAL CODE - JUST FOR DEBUGGING
void print_belief_map(BMap* map) {

	int grid[51][51];

	// sprintf
//	Serial.println("Printing belief map\n");

	int x, y;

	for(y=0;y<51;y++) {
		for(x=0;x<51;x++) {
			grid[x][y]=0;	
		}
	}
	
	int i;

	for(i=0;i<map->_size;i++) {
		Hn* point = map->_points[i];

		int rebasedPointX = 25 + point->x;
		int rebasedPointY = 25 + point->y;
		int isObstacle = point->is_obstacle;

		if (rebasedPointX > 0 && rebasedPointX < 51 && rebasedPointY > 0 && rebasedPointY < 51) {
			grid[rebasedPointX] [rebasedPointY] = isObstacle;
		}

//		printf("%d,%d,%d\n",point->x, point->y, point->is_obstacle);
//		printf("%d,%d,%d\n",rebasedPointX, rebasedPointY, isObstacle);
	}

	for(y=51;y>=0;y--) {
		printf("\n");
		for(x=0;x<51;x++) {
			if(x == (map->_curx +25) && y == (map->_cury+25)) {
				printf(" ^");
			}
			else if(x == 25 && y == 25) {
				printf(" 0");
			} else if(grid[x][y]==0) {
				printf(" -");
			} else {
				printf(" %d", grid[x][y]);	
			}
		}
	}

	printf("\nFinished printing beleif map\n");
}

void writeCommand(byte* &cmd) {
//  Serial.println("Write command");

  int messageSize = ((uint8_t)cmd[1]) + 2;

//  Serial.print("messageSize: ");  
//  Serial.println(messageSize);

  brainCommand = new byte[messageSize];

  for(int x=0;x<messageSize;x++) {
    brainCommand[x] = cmd[x];
  }
}

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
 
  switch (command)
  {
  // no command? then this is the command
  case 0:
    command = c;
    SPDR = 0;
    break;

//  case RESET_SENSORS:
//    // new ecosensors object;
//    break;
  case BRAIN_COMMAND:
    SPDR = brainCommand[byteCount++];  
    break;
    
  case BRAIN_CALCULATE:
    calculate = true;
    SPDR = 0;
    break;
    
  case BRAIN_MAP:
    SPDR = 0;
    break;
  
  case ULTRASONIC_0:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic0 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_45:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic45 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_90:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic90 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_135:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic135 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_180:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic180 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_225:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic225 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_270:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic270 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
  case ULTRASONIC_315:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.ultrasonic315 = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
    
  case COMPASS_BEARING:
    b[byteCount++] = c;
    if (byteCount == 4) {
      es.compassBearing = EcoSPI::bytesToLong(b);
    }
    SPDR = 0;
    break;
    
  case SPI_HANDSHAKE:
    SPDR = DEVICE_BRAIN;
    break;
    
  } // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect
