/*=========================================================================//

	MCB Class
	
	This class handles the modules plugged into the motor controller board
	
	Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2)
	
	
	Trevor Bruns
	
	Changelog-
		2/20/2016: Initial Creation
		3/13/2017: Moved MCBpin struct into its own class -> MCBpins.h
				   Fixed all issues due to switching from vectors to arrays in MCBpins.h
				   Compiles and appears to run successfully
		3/31/2017: Added waitForButtonHold()
        4/18/2017: Autodetection of modules

//=========================================================================*/

#ifndef MCB_h
#define MCB_h

#include "MCBmodule.h"
#include <ArduinoSTL.h> // this contains a <vector> class
#include "AD5761R.h"
#include "si5351.h"
#include "core_pins.h"
#include "MCBpins.h"

typedef std::vector<uint8_t>  Uint8Vec;
typedef std::vector<int16_t>  Int16Vec;
typedef std::vector<int32_t>  Int32Vec;
typedef std::vector<uint32_t> Uint32Vec;
typedef std::vector<float>    FloatVec;
typedef std::vector<double>	  DoubleVec;
typedef std::vector<bool>	  BoolVec;

class MCB
{	
public:
	MCBpins pins; // Teensy pin definitions for MCB
    MCB(void);
	~MCB(void);

    uint8_t numModules(void); // returns number of motor modules detected
    BoolVec isModuleConfigured(void); // returns moduleConfigured_
    bool isModuleConfigured(uint8_t positition); // returns moduleConfigured_[position]

	int  init(void); // initializes modules; returns number of detected modules
    void waitForButtonHold(void); // pauses program until Menu/Up/Down are all held for 2 seconds
    
	void enableAmp(uint8_t position);  // sets inhibit pin for motor amp
	void disableAmp(uint8_t position); // sets inhibit pin for motor amp
	void disableAllAmps(void); // disables all amps, regardles of numModules_
	void enableAllAmps(void);  // NOTE: only enables n = numModules_

	void setGains(uint8_t position, float kp, float ki, float kd); // sets PID gains for module located at [position] 
    FloatVec getGains(uint8_t position); // returns [kp, ki, kd] as float vector
    float getEffort(uint8_t position);   // returns computed effort of PID controller (prior to maxAmps saturation check)
	void setMaxAmps(uint8_t position, float maxAmps); // [Amps] set based on motor current corresponding to max DAC output
	float getMaxAmps(uint8_t position);               // DOES NOT directly limit current!! This must be done via ESCON Studio software
	
    void setCountDesired(uint8_t position, int32_t countDesired); // set desired count of motor located at [position]
    int32_t  getCountDesired(uint8_t position); // returns the current target of motor located at [position]
    Int32Vec getCountsDesired(void); // returns vector of last commanded count targets
    Int32Vec getCountsLast(void); // returns most recent encoder counts
    int32_t getCountLast(uint8_t moduleNum); // returns most recent motor position

    void stepPID(void); // PID controller performs one step (reads encoders, computes effort, updates DACs)

    void setLEDG(uint8_t position, bool state); // sets green LED
	void setLEDG(bool state); // sets all green LEDs
	void toggleLEDG(uint8_t position); // toggle state of green LED
	
    void setDACs(Int16Vec const &val); // manually set DAC outputs
	
	Uint32Vec readButtons(void);	 // check capacitive sensor buttons for key presses
	bool isDownPressed(void); // returns current state of down button
	bool isUpPressed(void);	 // returns current state of up button
	bool isMenuPressed(void); // returns current state of menu button
	bool isEverythingPressed(void); // true if down/up/menu are all pressed

private:
    std::vector<MCBmodule> modules_; // each module controls a single motor
	AD5761R DAC_; // provides access to all DACs
	Int16Vec DACval_; // stores the current DAC output commands
	Si5351 si5351_; // clock generator for encoder ICs (LS7366R)
    void addModule(uint8_t position);  // creates and adds module to <vector>modules
    uint8_t numModules_;
    BoolVec moduleConfigured_; // false if no module or module not configured successfully
	BoolVec LEDG_ = { LOW, LOW, LOW, LOW, LOW, LOW }; // Green LED status (true = on)
	bool isPinsInit = false; // true after pins.init() has been called
};

#endif