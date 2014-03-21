#ifndef EcoSPI_h
#define EcoSPI_h

#include "Arduino.h"
#include "EcoSensors.h"
#include <SPI.h>

#define START_BYTE         0x00
#define NEXT_BYTE          0xFF

// Sensors
#define RTC                0x80   // Maybe make these commands, i.e. CMD_RTC, CMD_READ_ULTRASONIC_0
#define ULTRASONIC_0       0x81
#define ULTRASONIC_45      0x82
#define ULTRASONIC_90      0x83
#define ULTRASONIC_135     0x84
#define ULTRASONIC_180     0x85
#define ULTRASONIC_225     0x86
#define ULTRASONIC_270     0x87
#define ULTRASONIC_315     0x88
#define TEMPERATURE_LEFT   0x89
#define TEMPERTURE_RIGHT   0x8A
#define LIGHT_LEFT         0x8B
#define LIGHT_RIGHT        0x8C
#define COMPASS_BEARING    0x8D
#define CO_PPM             0x8E

// Commands
#define BRAIN_COMMAND      0xA0
#define FACE_ROBOT         0xA1
#define MOVE_ROBOT         0XA2
#define BRAIN_MAP          0XAA
//#define
#define PANIC_ROBOT        0xAD
#define BRAIN_CALCULATE    0xAE
#define END_OF_COMMANDS    0xAF
//#define RESET_SENSORS      0xC0

// Devices
#define SPI_HANDSHAKE      0xF0 // Maybe make this command, i.e. CMD_HANDSHAKE
#define DEVICE_ULTRASONIC  0xF1
#define DEVICE_ENVIRONMENT 0xF2
#define DEVICE_BRAIN       0xF3
#define DEVICE_ROBOT       0xF4

struct SPI_BRAIN_COMMAND {
    byte command;
    uint8_t size;
    byte* data;
};

class EcoSPI
{
  public:
    EcoSPI(int interruptPin, byte device);
    boolean isConnected();
    boolean getReadings(EcoSensors &es);
    boolean sendReadings(EcoSensors es);
    boolean getCommands(SPI_BRAIN_COMMAND* &cmds);
    boolean getCommand(SPI_BRAIN_COMMAND &cmd);
    void sendCommand(SPI_BRAIN_COMMAND cmd);
    void sendCommand(byte* &cmd, int length);
    static byte getByte(int byteIndex, long val);
    static void getBytes(long val, byte* &buf);
    static long bytesToLong(byte b[4]);
  private:
    EcoSensors _es;
    void _startInterrupt();
    void _stopInterrupt();
    boolean _handshake();
    void _getReading(byte sensor, long &a);
    void _sendReading(byte sensor, long a);
    SPI_BRAIN_COMMAND _getCommand();
    int _interruptPin;
    byte _device;
    byte _transferAndWait(volatile byte data);
};

#endif
