// https://www.sparkfun.com/datasheets/Components/HMC6352.pdf
// http://forum.arduino.cc/index.php?topic=129161.0

#include "myCompass.h"
#include <Wire.h>

void myCompass::begin(){
	Wire.begin();
}
float myCompass::getReading(){
	_beginTransmission();
	_endTransmission();
	
  //time delays required by HMC6352 upon receipt of the command
  //Get Data. Compensate and Calculate New Heading : 6ms
  delay(6);

  Wire.requestFrom(HMC6352SlaveAddress, 2); //get the two data bytes, MSB and LSB

  //"The heading output data will be the value in tenths of degrees
  //from zero to 3599 and provided in binary format over the two bytes."
  byte MSB = Wire.read();
  byte LSB = Wire.read();

  float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
  float headingInt = headingSum / 10; 
  
  return headingInt;
}

void myCompass::_beginTransmission(){
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(HMC6352ReadAddress);
}
void myCompass::_endTransmission(){
  Wire.endTransmission();
}

void myCompass::startCalibration() {
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(0x43);
  Wire.endTransmission();
  delay(10);
}

void myCompass::endCalibration() {
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(0x45);
  Wire.endTransmission();
  delay(15);
}
