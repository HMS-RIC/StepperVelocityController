#include <SPI.h>
#include "XNucleoDualStepperDriver.h"

int XNucleoStepper::_motorsPerBoard = 2;

// Constructors
XNucleoStepper::XNucleoStepper(int position, int CSPin, int resetPin, int busyPin)
{
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = busyPin;
  _SPI = &SPI;
}

XNucleoStepper::XNucleoStepper(int position, int CSPin, int resetPin)
{
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = -1;
  _SPI = &SPI;
}

void XNucleoStepper::SPIPortConnect(SPIClass *SPIPort)
{
  _SPI = SPIPort;
}

int XNucleoStepper::busyCheck(void)
{
  if (_busyPin == -1)
  {
    if (getParam(STATUS) & 0x0002) return 0;
    else                           return 1;
  }
  else 
  {
    if (digitalRead(_busyPin) == HIGH) return 0;
    else                               return 1;
  }
}
