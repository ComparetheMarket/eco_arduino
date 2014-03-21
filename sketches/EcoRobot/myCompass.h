#ifndef myCompass_h
#define myCompass_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//0x21==0x42>>1, from bildr's code
#define HMC6352SlaveAddress 0x21
#define HMC6352ReadAddress 0x41

class myCompass{
	public:
		void begin();
		float getReading();
		void startCalibration();
		void endCalibration();
	private:
		void _beginTransmission();
		void _endTransmission();

};

#endif
