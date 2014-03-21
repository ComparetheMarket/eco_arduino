#include <ArduinoRobot.h>
#include <EcoSPI.h>
#include "myCompass.h"
#include "pins_arduino.h"

#define ROBOT_READY_PIN          TKD0

#define MOUSE_DATA                TK5
#define MOUSE_CLOCK               TK6

//PS2Mouse mouse(MOUSE_DATA, MOUSE_CLOCK, STREAM);
long deltaX = 0;
long deltaY = 0;

// what to do with incoming data
volatile byte command = 0;
volatile int byteCount = 0;
byte* b;

byte other;

volatile unsigned long bearing = 0;
volatile long move_distance = 0;
volatile long face_bearing = 0;

volatile boolean faceRobot = false;
volatile boolean moveRobot = false;
volatile boolean stopRobot = false;

boolean process_it = false;
boolean new_command = false;

myCompass comp;

void setup (void)
{
  pinMode(ROBOT_READY_PIN, OUTPUT);
  digitalWrite(ROBOT_READY_PIN, HIGH);
  
  // have to send on master in, *slave out*
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  
  Serial.begin(115200);
  
  delay(500);
  
  Serial.println();
  Serial.println("EcoRobot");
  
  Serial.println("Initialise Robot");
  // initialize the robot
  Robot.begin();
  
  Serial.println("Initialise Compass");
  comp.begin();
  
  delay(1500);
  
  Serial.println("Calibrate Compass");
  comp.startCalibration();
  
  Robot.motorsWrite(100,-100);
  
  delay(6000);

  Robot.motorsStop(); //Fast stop the robot
  delay(1000); // wait for a moment
  
  Serial.println("End Compass Calibration");
  comp.endCalibration();

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

  digitalWrite(ROBOT_READY_PIN, LOW);
}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
  switch (command)
  {
  // no command? then this is the command
  case 0:
    command = c;
    new_command = true;
    SPDR = 0;
    break;

  case COMPASS_BEARING:
    SPDR = EcoSPI::getByte(byteCount++, bearing);
    break;
    
  case FACE_ROBOT:
    if (!faceRobot) {
      faceRobot = true;
    } else {
      b[byteCount++] = c;
      if (byteCount == 4) {
        face_bearing = EcoSPI::bytesToLong(b);
        process_it = true;
      }
    }
    SPDR = 0;
    break;
    
  case MOVE_ROBOT:
    if (!moveRobot) {
      moveRobot = true; 
    } else {
      b[byteCount++] = c;
      if (byteCount == 4) {
        move_distance = EcoSPI::bytesToLong(b);
        process_it = true;
      }
    }
    SPDR = 0;
    break;
    
  case SPI_HANDSHAKE:
    SPDR = DEVICE_ROBOT;
    break;

  default:
    other = c;
    SPDR = 0;

  } // end of switch

}  // end of interrupt service routine (ISR) SPI_STC_vect

void loop (void)
{ 
  bearing = comp.getReading();
  
  if (process_it && faceRobot) {
    faceRobot = false;
    digitalWrite(ROBOT_READY_PIN, HIGH);
    Robot.motorsStop(); //Fast stop the robot
    delay(1000); // wait for a moment
    pointTo(face_bearing);
    digitalWrite(ROBOT_READY_PIN, LOW);
  }
  
  if (process_it && moveRobot) {
    moveRobot = false;
    digitalWrite(ROBOT_READY_PIN, HIGH);
    Robot.motorsWrite(120,120); // this is speed, not distance
    delay(200); // move a specific distance
    Robot.motorsStop(); //Fast stop the robot
    digitalWrite(ROBOT_READY_PIN, LOW);
  }
  
  if (new_command && digitalRead (SS) == HIGH) { 
    b = new byte[4];
    command = 0;
    byteCount = 0;
    faceRobot = false;
    moveRobot = false;
    stopRobot = false;
    move_distance = 0;
    face_bearing = 0;
    process_it = false;
    new_command = false;
    other = 0x79;
  }
}  // end of loop

// Copy of pointsTo function from Motors.cpp as need to change speed
void pointTo(int angle){
  
  Serial.print("pointTo: ");
  Serial.print(angle);
  Serial.println(" degrees");
  
  int target=angle;
  uint8_t speed=120;
  target=target%360;
  if(target<0){
    target+=360;
  }
  int direction=angle;
  while(1){
    if(direction>0){
      Robot.motorsWrite(speed,-speed);//right
      delay(10);
    }else{
      Robot.motorsWrite(-speed,speed);//left
      delay(10);
    }
    
    int currentAngle=comp.getReading();
    int diff=target-currentAngle;
    
    Serial.print("target: ");
    Serial.print(target);
    Serial.print(", currentAngle: ");
    Serial.print(currentAngle);
    Serial.print(", diff: ");
    Serial.println(diff);
    
    if(diff<-180) 
      diff += 360;
    else if(diff> 180) 
      diff -= 360;
    direction=-diff;
		
    if(abs(diff)<5){
      Robot.motorsWrite(0,0);
        return;
      }
    }
}

