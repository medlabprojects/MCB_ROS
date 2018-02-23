//=========================================================================//
/*
MCBpins Class

Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2)
This class initializes and allows easy referencing to all pins


Trevor Bruns

Changelog-
2/20/2016: Initial Creation
3/13/2017: Moved from MCB.h to its own header file
		   Changed all uses of <vector> to standard arrays
		   Moved pin initializations from constructor to init()
3/21/2017: Modified init() to properly reset WIZ820io

*/
//=========================================================================//

#ifndef MCBpins_h
#define MCBpins_h

#include <stdint.h>
#include <Adafruit_MCP23008.h>

class MCBpins
{
public:
    MCBpins(void) {};

	void init(void) // initializes all SPI chip-select pins
	{
		// begin reset for WIZ820io
		pinMode(resetWiz, OUTPUT);
		digitalWriteFast(resetWiz, LOW);

		// de-select WIZ820io
		pinMode(csWiz, OUTPUT);
		digitalWriteFast(csWiz, HIGH);

		// de-select SD card
		pinMode(csSdCard, OUTPUT);
		digitalWriteFast(csSdCard, HIGH);

		// de-select global pin for DACs (SYNC pin)
		pinMode(csDac, OUTPUT);
		digitalWriteFast(csDac, HIGH);

		// CTRL switch input
		pinMode(modeSelect, INPUT);

        // setup MCP23008
        i2cPins.begin();
        i2cPins.setInterruptMode(Adafruit_MCP23008::ActiveLow);
        i2cPins.digitalWrite(i2cEnableGlobal, HIGH); // HIGH = INHIBIT
        i2cPins.pinMode(i2cEnableGlobal, OUTPUT);
        i2cPins.pinMode(i2cBrakeHw, INPUT);
        i2cPins.attachInterrupt(i2cBrakeHw, CHANGE);
        i2cPins.pinMode(i2cEnableM0, INPUT);
        i2cPins.attachInterrupt(i2cEnableM0, CHANGE);
        i2cPins.pinMode(i2cEnableM1, INPUT);
        i2cPins.attachInterrupt(i2cEnableM1, CHANGE);
        i2cPins.pinMode(i2cEnableM2, INPUT);
        i2cPins.attachInterrupt(i2cEnableM2, CHANGE);
        i2cPins.pinMode(i2cEnableM3, INPUT);
        i2cPins.attachInterrupt(i2cEnableM3, CHANGE);
        i2cPins.pinMode(i2cEnableM4, INPUT);
        i2cPins.attachInterrupt(i2cEnableM4, CHANGE);
        i2cPins.pinMode(i2cEnableM5, INPUT);
        i2cPins.attachInterrupt(i2cEnableM5, CHANGE);

        // interrupt pin for MCP23008
        pinMode(i2cInt, INPUT);

		// pins for daughterboard modules
		for (uint8_t aa = 0; aa < maxNumBoards; aa++)
		{
			// de-select quadrature decoders
			pinMode(csEnc[aa], OUTPUT);
			digitalWriteFast(csEnc[aa], HIGH);
			// status LEDs (green)
			pinMode(led[aa], OUTPUT);
			digitalWriteFast(led[aa], LOW);
			// software brakes (HIGH = amps disabled)
			pinMode(ampCtrl[aa], OUTPUT);
			digitalWriteFast(ampCtrl[aa], HIGH); // should match MCB::ampCtrlState_[aa]
		}

		// end reset pulse for WIZ820io
		digitalWriteFast(resetWiz, HIGH);
	}

	// These pins are current as of Rev 1.4
    Adafruit_MCP23008 i2cPins; // extra GPIO available over I2C via MCP23008
    const uint8_t i2cBrakeHw = 0;  // i2cPin: connected to hardware brake switch
    const uint8_t i2cEnableM5 = 2; // i2cPin: state of enable pin for motor 5 amp
    const uint8_t i2cEnableM4 = 3; // i2cPin: state of enable pin for motor 4 amp
    const uint8_t i2cEnableM3 = 4; // i2cPin: state of enable pin for motor 3 amp
    const uint8_t i2cEnableM2 = 5; // i2cPin: state of enable pin for motor 2 amp
    const uint8_t i2cEnableM1 = 6; // i2cPin: state of enable pin for motor 1 amp
    const uint8_t i2cEnableM0 = 7; // i2cPin: state of enable pin for motor 0 amp
    //const uint8_t i2cEnableGlobal = 7; // i2cPin: global enable control for all amps
    const uint8_t i2cInt = 22; // interrupt pin of MCP23008; signals when the enable pin of an amp changes

    const uint8_t EnableGlobal = 33; // global enable control for all amps
    const uint8_t maxNumBoards = 6; // number of daughterboard sockets
	const uint8_t csSdCard = 4;     // SD card chip-select
	const uint8_t resetWiz = 9; // WIZ820io reset pin
	const uint8_t csWiz = 10;	// WIZ820io chip-select pin
	const uint8_t csDac = 27;   // global chip-select pin (SYNC) for DACs (AD5761R)
    const uint8_t modeSelect = 28; // connected to manual/ROS mode select switch
	const uint8_t csEnc[6] = { 20, 17, 15, 29, 32, 30 }; // chip-select pins for quadrature encoder IC (LS7366R)
    const uint8_t ampCtrl[6] = { 21, 16, 14, 25, 26, 31 }; // motor amp control pins (HIGH = POWER ON)
    const uint8_t led[6] = { 24, 7, 6, 5, 3, 2 };   // control the green LEDs
	const uint8_t buttonDown = 0;
	const uint8_t buttonUp = 1;
	const uint8_t buttonMenu = 23;
	const uint8_t buttons[3] = { buttonDown, buttonUp, buttonMenu };
	const float   buttonThresh[3] = { 1500, 1500, 1500 }; // thresholds that constitute a key press (pF)
	volatile bool buttonStates[3] = { 0, 0, 0 }; // volatile in case used within interrupt
};

#endif