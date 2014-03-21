#include <EcoSPI.h>
#include <NewPing.h>
#include "pins_arduino.h"

#define SAMPLE_SIZE          3    // Number of samples for average
#define SONAR_NUM            8    // Number or sensors.
#define MAX_DISTANCE         500  // Maximum distance (in cm) to ping.
#define PING_INTERVAL        33   // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define ULTRASONIC_PIN_0     3
#define ULTRASONIC_PIN_45    4
#define ULTRASONIC_PIN_90    5
#define ULTRASONIC_PIN_135   6
#define ULTRASONIC_PIN_180   7
#define ULTRASONIC_PIN_225   8
#define ULTRASONIC_PIN_270   9
#define ULTRASONIC_PIN_315   A0

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned long cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {        // Sensor object array.
  NewPing(ULTRASONIC_PIN_0,   ULTRASONIC_PIN_0,   MAX_DISTANCE),    // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(ULTRASONIC_PIN_45,  ULTRASONIC_PIN_45,  MAX_DISTANCE),
  NewPing(ULTRASONIC_PIN_90,  ULTRASONIC_PIN_90,  MAX_DISTANCE),
  NewPing(ULTRASONIC_PIN_135, ULTRASONIC_PIN_135, MAX_DISTANCE),
  NewPing(ULTRASONIC_PIN_180, ULTRASONIC_PIN_180, MAX_DISTANCE),
  NewPing(ULTRASONIC_PIN_225, ULTRASONIC_PIN_225, MAX_DISTANCE),
  NewPing(ULTRASONIC_PIN_270, ULTRASONIC_PIN_270, MAX_DISTANCE),
  NewPing(ULTRASONIC_PIN_315, ULTRASONIC_PIN_315, MAX_DISTANCE)
};

// what to do with incoming data
volatile byte command = 0;
int byteCount = 0;

// start of transaction, no command yet
void ss_falling ()
{
  command = 0;
  byteCount = 0;
}  // end of interrupt service routine (ISR) ss_falling

void setup (void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Ultrasonics");
  
  // setup pins
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

  // interrupt for SS falling edge
  attachInterrupt (0, ss_falling, FALLING);
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}  // end of setup

void loop (void)
{
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = sonar[currentSensor].ping_median(SAMPLE_SIZE)  / US_ROUNDTRIP_CM;
    }
  }
}  // end of loop

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
    
  case ULTRASONIC_0:
    SPDR = EcoSPI::getByte(byteCount++, cm[0]);
    break;
  case ULTRASONIC_45:
    SPDR = EcoSPI::getByte(byteCount++, cm[1]);
    break;
  case ULTRASONIC_90:
    SPDR = EcoSPI::getByte(byteCount++, cm[2]);
    break;
  case ULTRASONIC_135:
    SPDR = EcoSPI::getByte(byteCount++, cm[3]);
    break;
  case ULTRASONIC_180:
    SPDR = EcoSPI::getByte(byteCount++, cm[4]);
    break;
  case ULTRASONIC_225:
    SPDR = EcoSPI::getByte(byteCount++, cm[5]);
    break;
  case ULTRASONIC_270:
    SPDR = EcoSPI::getByte(byteCount++, cm[6]);
    break;
  case ULTRASONIC_315:
    SPDR = EcoSPI::getByte(byteCount++, cm[7]);
    break;
    
  case SPI_HANDSHAKE:
    SPDR = DEVICE_ULTRASONIC;
    break;
  } // end of switch
}  // end of interrupt service routine (ISR) SPI_STC_vect

