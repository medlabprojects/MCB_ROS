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
typedef std::vector<int8_t>   Int8Vec;
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
    bool isModuleConfigured(uint8_t position); // returns moduleConfigured_[position]

	int  init(void); // initializes modules; returns number of detected modules
    void waitForButtonHold(void); // pauses program until Menu/Up/Down are all held for 2 seconds
    
    void setPolarity(uint8_t position, bool polarity); // used to make sure positive current -> positive encoder counts
    void setPolarity(bool polarity); // sets all modules

    bool isAmpEnabled(uint8_t position); // true if amp output is enabled
    bool enableAmp(uint8_t position);  // sets inhibit pin for motor amp
	bool disableAmp(uint8_t position); // sets inhibit pin for motor amp
	bool disableAllAmps(void); // disables all amps, regardles of numModules_
	bool enableAllAmps(void);  // NOTE: only enables n = numModules_

    uint8_t updateAmpStates(void); // call this after ampEnableFlag_ has been set true; returns device # that caused trigger
    bool limitSwitchState(uint8_t position); // returns limitSwitchState_[position]
    bool eStopState(void); // returns eStopState_
    bool limitSwitchTriggeredFlag(void); // returns limitSwitchTriggeredFlag_
    void resetLimitSwitchTriggered(void); // resets limitSwitchTriggeredFlag_ and limitSwitchTriggered to false
    bool ampEnableFlag(void);
    void setAmpEnableFlag(void); // ONLY to be used by ampEnableISR() to set ampEnableFlag_ true; only disabled by calling updateAmpStates()

	void setGains(uint8_t position, float kp, float ki, float kd); // sets PID gains for module located at [position] 
    FloatVec getGains(uint8_t position); // returns [kp, ki, kd] as float vector
    float getEffort(uint8_t position);   // returns computed effort of PID controller (prior to maxAmps saturation check)
	
    void setCountDesired(uint8_t position, int32_t countDesired); // set desired count of motor located at [position]
    int32_t  getCountDesired(uint8_t position); // returns the current target of motor located at [position]
    Int32Vec getCountsDesired(void); // returns vector of last commanded count targets
    Int32Vec getCountsLast(void); // returns most recent encoder counts
    int32_t  getCountLast(uint8_t moduleNum); // returns most recent motor position
    bool     resetCount(uint8_t moduleNum);   // resets the encoder count to zero
    bool     resetCounts(void); // resets all encoders to zero

    void stepPid(void); // PID controller performs one step (reads encoders, computes effort, updates DACs)
    void restartPid(uint8_t position); // resets PID controller, but keeps gains

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
    bool isPinsInit = false; // true after pins.init() has been called
    std::vector<MCBmodule> modules_; // each module controls a single motor
    void addModule(uint8_t position);  // creates and adds module to <vector>modules
    BoolVec moduleConfigured_; // false if no module or module not configured successfully
    uint8_t numModules_; // number of modules detected; equal to modules_.size() 

	AD5761R DAC_; // provides access to all DACs
	Int16Vec DACval_; // stores the current DAC output commands
	
    Si5351 si5351_; // clock generator for encoder ICs (LS7366R)

    volatile bool ampEnableFlag_ = true; // this flag is set by ampEnabledISR whenever ampEnabled_ state changes
    bool limitSwitchTriggeredFlag_ = false; // gets set true whenever a limitSwitchState changes
    int8_t  whichDevice(void);  // returns the index (0-5) of the motor that caused the interrupt; E-stop/hardware brake = 6
    Int8Vec whichDevices(void); // use if there are multiple pins interrupted; i.e. when whichLimitSwitch() = -1
    BoolVec limitSwitchTriggered_ = { false, false, false, false, false, false }; // true when limit switch has been triggered
    BoolVec limitSwitchState_ = { false, false, false, false, false, false }; // current state of each limit switch
    BoolVec ampCtrlState_ = { false, false, false, false, false, false }; // current state of ampCtrl pins (aka brake_sw)
    BoolVec ampEnabled_ = { false, false, false, false, false, false }; // current state of each amp's enable pin
    bool    eStopState_ = false; // current state of E-stop/hardware brake

    BoolVec LEDG_ = { LOW, LOW, LOW, LOW, LOW, LOW }; // Green LED status (true = on)
};

#endif