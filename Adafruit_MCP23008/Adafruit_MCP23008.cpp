/*************************************************** 
  This is a library for the MCP23008 i2c port expander

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

//#if defined(__arm__) && defined(CORE_TEENSY)
//    #include <i2c_t3.h> // TLB: 4/26/17
//#elif __AVR_ATtiny85__
//    #include <TinyWireM.h>
//    #define Wire TinyWireM
//#else
//    #include <Wire.h>
//#endif
#include <i2c_t3.h> // TLB: 4/26/17

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include "Adafruit_MCP23008.h"
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

void Adafruit_MCP23008::begin(uint8_t addr) {
  if (addr > 7) {
    addr = 7;
  }
  i2caddr = addr;

  Wire.begin();
  //Wire.setClock(400000); // TLB: 4/26/17

  // set defaults!
  Wire.beginTransmission(MCP23008_ADDRESS | i2caddr);
#if ARDUINO >= 100
  Wire.write((byte)MCP23008_IODIR);
  Wire.write((byte)0xFF);  // all inputs
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);	
#else
  Wire.send(MCP23008_IODIR);
  Wire.send(0xFF);  // all inputs
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);
  Wire.send(0x00);	
#endif
  Wire.endTransmission();

}

void Adafruit_MCP23008::begin(void) {
  begin(0);
}

void Adafruit_MCP23008::pinMode(uint8_t p, uint8_t d) {
  uint8_t iodir;
  

  // only 8 bits!
  if (p > 7)
    return;
  
  iodir = read8(MCP23008_IODIR);

  // set the pin and direction
  if (d == INPUT) {
    iodir |= 1 << p; 
  } else {
    iodir &= ~(1 << p);
  }

  // write the new IODIR
  write8(MCP23008_IODIR, iodir);
}

uint8_t Adafruit_MCP23008::readGPIO(void) {
  // read the current GPIO input 
  return read8(MCP23008_GPIO);
}

void Adafruit_MCP23008::writeGPIO(uint8_t gpio) {
  write8(MCP23008_GPIO, gpio);
}


void Adafruit_MCP23008::digitalWrite(uint8_t p, uint8_t d) {
  uint8_t gpio;
  
  // only 8 bits!
  if (p > 7)
    return;

  // read the current GPIO output latches
  gpio = readGPIO();

  // set the pin and direction
  if (d == HIGH) {
    gpio |= 1 << p; 
  } else {
    gpio &= ~(1 << p);
  }

  // write the new GPIO
  writeGPIO(gpio);
}

void Adafruit_MCP23008::pullUp(uint8_t p, uint8_t d) {
  uint8_t gppu;
  
  // only 8 bits!
  if (p > 7)
    return;

  gppu = read8(MCP23008_GPPU);
  // set the pin and direction
  if (d == HIGH) {
    gppu |= 1 << p; 
  } else {
    gppu &= ~(1 << p);
  }
  // write the new GPIO
  write8(MCP23008_GPPU, gppu);
}

bool Adafruit_MCP23008::setInterruptMode(InterruptMode mode)
{
    uint8_t iocon = read8(MCP23008_IOCON);

    switch (mode) {
    case ActiveHigh:
        iocon &= ~(1 << 2); // active driver output (ODR=0)
        iocon |= (1 << 1);  // active-high (INTPOL=1)
        break;

    case ActiveLow:
        iocon &= ~(1 << 2); // active driver output (ODR=0)
        iocon &= ~(1 << 1); // active-low (INTPOL=0)
        break;

    case OpenDrain:
        iocon |= (1 << 2); // open-drain output (ODR=1)
        // INTPOL is ignored when ODR=1
        break;

    default:
        return false;
    }

    write8(MCP23008_IOCON, iocon);

    return true;
}

int8_t Adafruit_MCP23008::attachInterrupt(uint8_t p, int mode) {
	if (p > 7){
		return -1;
	}
	
	// ensure pin p is configured as input
	pinMode(p, INPUT);
	
	// read current settings so we only affect the pin we want
	uint8_t intcon  = read8(MCP23008_INTCON);
	uint8_t defval  = read8(MCP23008_DEFVAL);
	uint8_t gpinten = read8(MCP23008_GPINTEN);
	
	// enable pin p for interrupt-on-change
	gpinten |= (1 << p);
		
	// set triggering mode
	switch (mode) {
	case RISING:
		// set intcon to 1 (pin p is compared against DEFVAL)
		intcon |= (1 << p);
		
		// set DEFVAl to 0 (interrupt trigger when p goes HIGH)
		defval &= ~(1 << p);
		break;
		
	case FALLING:
		// set intcon to 1 (pin p is compared against DEFVAL)
		intcon |= (1 << p);
		
		// set DEFVAl to 1 (interrupt trigger when p goes LOW)
		defval |= (1 << p);
		break;
		
	case CHANGE:
		// set intcon to 0 (pin p is compared against previous value)
		intcon &= ~(1 << p);
		break;
		
	default:
		return -1; // incorrect mode
	}
	
	// write settings
	write8(MCP23008_INTCON, intcon);
	write8(MCP23008_DEFVAL, defval);
	write8(MCP23008_GPINTEN, gpinten);
	
	return 1;
}

uint8_t Adafruit_MCP23008::readInterrupt(void) {
	// read interrupt flag register
	return read8(MCP23008_INTF);
}

bool Adafruit_MCP23008::readInterrupt(uint8_t p) {
	if (p > 7){
		return 0;
	}
	
	// read interrupt flag register
	uint8_t intf = read8(MCP23008_INTF);
	
	// check pin p has triggered an interrupt
    intf &= (1 << p); // isolate bit p
    intf = (intf >> p); // move bit p to bit 0
    return (bool)intf;
}

int8_t Adafruit_MCP23008::whichInterrupt(void)
{
    // read INTF register 
    uint8_t intf = readInterrupt();

    // check that only one interrupt was triggered (must be 0 or a power of 2)
    if (intf & (intf - 1)) {
        return -1;
    }
	else if (intf == 0) {
		return -2; // no pins triggered
	}

    int8_t intPin = 0;
    uint8_t ii = 1;

    // iterate through intf to find which bit is set
    while (!(ii & intf)) {
        ii = ii << 1;
        intPin++;
    }

    return intPin;
}

bool Adafruit_MCP23008::resetInterrupts(void) {
    // Clearing interrupt depends on mode:
    // CHANGE - interrupt pin is active until readGPIO() is called
    // RISING/FALLING - readGPIO() must also be called, but additionally, the pin is active as long as the interrupt condition exists
    readGPIO();

    return true;
}

uint8_t Adafruit_MCP23008::digitalRead(uint8_t p) {
  // only 8 bits!
  if (p > 7)
    return 0;

  // read the current GPIO
  return (readGPIO() >> p) & 0x1;
}

uint8_t Adafruit_MCP23008::read8(uint8_t addr) {
  Wire.beginTransmission(MCP23008_ADDRESS | i2caddr);
#if ARDUINO >= 100
  Wire.write((byte)addr);	
#else
  Wire.send(addr);	
#endif
  Wire.endTransmission();
  Wire.requestFrom(MCP23008_ADDRESS | i2caddr, 1);

#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}


void Adafruit_MCP23008::write8(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(MCP23008_ADDRESS | i2caddr);
#if ARDUINO >= 100
  Wire.write((byte)addr);
  Wire.write((byte)data);
#else
  Wire.send(addr);	
  Wire.send(data);
#endif
  Wire.endTransmission();
}
