#include "Arduino.h"
#include <SPI.h>
#include "EcoSPI.h"

#define DELAY_100NS do { asm volatile("nop"); }while(0);
#define DELAY_SPI(X) { int ii=0; do {  asm volatile("nop"); }while(++ii<X);}
#define DELAY_TRANSFER() DELAY_SPI(20)

EcoSPI::EcoSPI(int interruptPin, byte device)
{
  _interruptPin = interruptPin;
  _device = device;

  pinMode(_interruptPin, OUTPUT);
  digitalWrite(_interruptPin, HIGH);
}

void EcoSPI::_startInterrupt() {
  // enable Slave Select
  digitalWrite(_interruptPin, LOW);
  delay(20); // Let the imterrupt routine take effect
}

void EcoSPI::_stopInterrupt() {
  // disable Slave Select
  digitalWrite(_interruptPin, HIGH);
  delay(10);
}

boolean EcoSPI::_handshake() {
  byte a;

  _startInterrupt();

  _transferAndWait (SPI_HANDSHAKE);  // handshake command
  _transferAndWait (0);
  a = _transferAndWait (0);

  _stopInterrupt();

  Serial.print("Handshake ");

  switch (_device) {
    case DEVICE_ULTRASONIC:
      Serial.print("DEVICE_ULTRASONIC");
      break;
    case DEVICE_ENVIRONMENT:
      Serial.print("DEVICE_ENVIRONMENT");
      break;
    case DEVICE_BRAIN:
      Serial.print("DEVICE_BRAIN");
      break;
    case DEVICE_ROBOT:
      Serial.print("DEVICE_ROBOT");
      break;
  }

  if (a == _device) {
    Serial.println(" Succeeded.");
  } else {
    Serial.println(" Failed.");
  }

  return (a == _device);
}

boolean EcoSPI::isConnected() {
  return _handshake();
}

boolean EcoSPI::getReadings(EcoSensors &es)
{
  if (!_handshake()) {
    return false;
  }

  switch (_device) {
    case DEVICE_ULTRASONIC:
      _getReading(ULTRASONIC_0,     es.ultrasonic0);
      _getReading(ULTRASONIC_45,    es.ultrasonic45);
      _getReading(ULTRASONIC_90,    es.ultrasonic90);
      _getReading(ULTRASONIC_135,   es.ultrasonic135);
      _getReading(ULTRASONIC_180,   es.ultrasonic180);
      _getReading(ULTRASONIC_225,   es.ultrasonic225);
      _getReading(ULTRASONIC_270,   es.ultrasonic270);
      _getReading(ULTRASONIC_315,   es.ultrasonic315);
      break;
    case DEVICE_ROBOT:
      _getReading(COMPASS_BEARING,  es.compassBearing);
      break;
  }
  return true;
}

boolean EcoSPI::sendReadings(EcoSensors es)
{
  if (!_handshake()) {
    return false;
  }

  if (_device == DEVICE_BRAIN) {
    _sendReading(ULTRASONIC_0,     es.ultrasonic0);
    _sendReading(ULTRASONIC_45,    es.ultrasonic45);
    _sendReading(ULTRASONIC_90,    es.ultrasonic90);
    _sendReading(ULTRASONIC_135,   es.ultrasonic135);
    _sendReading(ULTRASONIC_180,   es.ultrasonic180);
    _sendReading(ULTRASONIC_225,   es.ultrasonic225);
    _sendReading(ULTRASONIC_270,   es.ultrasonic270);
    _sendReading(ULTRASONIC_315,   es.ultrasonic315);

    _sendReading(COMPASS_BEARING,  es.compassBearing);
  }

  return true;
}

boolean EcoSPI::getCommands(SPI_BRAIN_COMMAND* &cmds)
{
  if (!_handshake()) {
    return false;
  }

  if (_device == DEVICE_BRAIN) {

    SPI_BRAIN_COMMAND cmd = _getCommand();

    cmds[0] = cmd;
  }

  return true;
}

void EcoSPI::_getReading(byte sensor, long &a) {
  byte* b = new byte[4];

  _startInterrupt();

  _transferAndWait (sensor);  // handshake command
  _transferAndWait (10);
  b[0] = _transferAndWait (20);
  b[1] = _transferAndWait (30);
  b[2] = _transferAndWait (40);
  b[3] = _transferAndWait (50);

  _stopInterrupt();

  a = bytesToLong(b);
}

void EcoSPI::_sendReading(byte sensor, long a) {
  _startInterrupt();

  _transferAndWait (sensor);  // handshake command
  _transferAndWait (getByte(0,a));
  _transferAndWait (getByte(1,a));
  _transferAndWait (getByte(2,a));
  _transferAndWait (getByte(3,a));

  _stopInterrupt();
}

boolean EcoSPI::getCommand(SPI_BRAIN_COMMAND &cmd) {
  _startInterrupt();

  _transferAndWait (BRAIN_COMMAND);  // handshake command        -- returns last value, i.e. handshake
  _transferAndWait (0);              // returns 0 as we set the command

  cmd.command = _transferAndWait (0);

  if (cmd.command != FACE_ROBOT && cmd.command != MOVE_ROBOT) {
    _stopInterrupt();
    return false;
  }

  cmd.size = (uint8_t) _transferAndWait (0);

  for(int i=0; i<cmd.size; i++) {
    cmd.data[i] = _transferAndWait (0);
  }

  _stopInterrupt();

  return true;
}

SPI_BRAIN_COMMAND EcoSPI::_getCommand() {
  SPI_BRAIN_COMMAND cmd;

  _startInterrupt();

  _transferAndWait (BRAIN_COMMAND);  // handshake command        -- returns last value, i.e. handshake
  _transferAndWait (0);              // returns 0 as we set the command

  cmd.command = _transferAndWait (0);
  /*cmd.size =*/ _transferAndWait (0);
  cmd.size = 4;

  for(int i=0; i<cmd.size; i++) {
    cmd.data[i] = _transferAndWait (0);
  }

  _stopInterrupt();

  return cmd;
}

void EcoSPI::sendCommand(SPI_BRAIN_COMMAND cmd)
{
  if (!_handshake()) {
    return;// false;
  }

  _startInterrupt();

  _transferAndWait (cmd.command);
  _transferAndWait (cmd.size);

  for (int i=0; i<cmd.size; i++)
  {
    _transferAndWait (cmd.data[i]);
  }

  _stopInterrupt();
}


void EcoSPI::sendCommand(byte* &cmd, int length)
{
  if (!_handshake()) {
    return;// false;
  }

  _startInterrupt();

  for (int i=0; i<length; i++)
  {
    _transferAndWait (cmd[i]);
  }

  _stopInterrupt();
}

byte EcoSPI::_transferAndWait (volatile byte data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  byte result = SPDR;
  DELAY_TRANSFER();

  return result;
} // end of transferAndWait

long EcoSPI::bytesToLong(byte b[4]) {
  long val = 0;
  val = b[0] << 24;
  val |= b[1] << 16;
  val |= b[2] << 8;
  val |= b[3];
  return val;
}

byte EcoSPI::getByte(int byteIndex, long val) {
  byte output;

  switch(byteIndex)
  {
  case 0:
    output = (byte)((val >> 24) & 0xff);
    break;
  case 1:
    output = (byte)((val >> 16) & 0xff);
    break;
  case 2:
    output = (byte)((val >> 8) & 0xff);
    break;
  case 3:
    output = (byte)(val & 0xff);
    break;
  default:
    output = 0x0;
  }

  return output;
}

void EcoSPI::getBytes(long val, byte* &buf) {
  buf[0] = (byte)((val >> 24) & 0xff);
  buf[1] = (byte)((val >> 16) & 0xff);
  buf[2] = (byte)((val >> 8) & 0xff);
  buf[3] = (byte)(val & 0xff);
}
