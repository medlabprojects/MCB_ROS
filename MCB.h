/*=========================================================================//

	MCB Class
	
	This class handles the modules plugged into the motor controller board
	
	Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2+)
	
	
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

    int init(bool ingoreErrors = false); // initializes modules; returns number of detected modules or -1 if errors (unless ignoreErrors = true)

    enum class ErrorCode { NO_ERROR, WRONG_MODULE_ORDER, ESTOP_TRIGGERED, LIMIT_SWITCH_TRIGGERED_ON_STARTUP };
    ErrorCode getErrorCode() { return errorCode_; } // returns any current error code

    uint8_t numModules(void) { return numModules_; } // returns number of motor modules detected
    BoolVec isModuleConfigured(void) { return moduleConfigured_; } // returns moduleConfigured_
    bool isModuleConfigured(uint8_t position) { return moduleConfigured_[position]; }

    void waitForButtonHold(void); // pauses program until Menu/Up/Down are all held for 2 seconds
    
    void setPolarity(uint8_t position, bool polarity); // used to make sure positive current -> positive encoder counts
    void setPolarity(bool polarity); // sets all modules
    
    bool setGlobalInhibit(bool inhibit, bool eStopCheck = true); // global inhibit control for all amps; TRUE = INHIBIT (i.e. no power to motors)
    bool isAmpEnabled(uint8_t position); // true if amp output is enabled
    bool enableAmp(uint8_t position);  // sets inhibit pin for motor amp
	bool disableAmp(uint8_t position); // sets inhibit pin for motor amp
	bool disableAllAmps(void); // disables all amps, regardles of numModules_
	bool enableAllAmps(void);  // NOTE: only enables n = numModules_

    enum class LimitSwitch { ESTOP, LIMIT_M0, LIMIT_M1, LIMIT_M2, LIMIT_M3, LIMIT_M4, LIMIT_M5, ERROR };
    LimitSwitch positionToLimitSwitch(uint8_t position); // convert position index to MCB::LimitSwitch type
    uint8_t limitSwitchToPosition(LimitSwitch limitSwitch); // convert MCB::LimitSwitch type to position index
    MCB::ErrorCode initLimitSwitchStates(void); // determines initial states of limit switches and e-stop by briefly toggling enableGlobal and ampCtrl pins
    bool updateAmpStates(void); // call this after ampEnableFlag_ has been set true. Returns true if a limit switch was triggered
    std::vector<LimitSwitch> triggeredLimitSwitches(void) { return triggeredLimitSwitches_; }
    bool limitSwitchState(uint8_t position) { return limitSwitchState_[position]; }
    bool eStopState(void) { return eStopState_; }
    //bool triggeredLimitSwitchFlag(void) { return triggeredLimitSwitchFlag_; }
    void resetTriggeredLimitSwitches(void) { triggeredLimitSwitches_.clear(); }

    void setAmpEnableFlag(void) { ampEnableFlag_ = true; } // ONLY to be used by ampEnableISR() to set ampEnableFlag_ true; flag is only disabled by calling updateAmpStates()
    bool ampEnableFlag(void) { return ampEnableFlag_; }

    void setGains(uint8_t position, float kp, float ki, float kd) { modules_.at(position).setGains(kp, ki, kd); } // sets PID gains for module located at [position] 
    FloatVec getGains(uint8_t position); // returns [kp, ki, kd] as float vector
    float getEffort(uint8_t position) { return modules_.at(position).getEffort(); } // returns computed effort of PID controller (prior to maxAmps saturation check)
	
    void setCountDesired(uint8_t position, int32_t countDesired) { modules_.at(position).setCountDesired(countDesired); } // set desired count of motor located at [position]
    int32_t  getCountDesired(uint8_t position) { return modules_.at(position).getCountDesired(); } // returns the current target of motor located at [position]
    Int32Vec getCountsDesired(void); // returns vector of last commanded count targets
    Int32Vec getCountsLast(void); // returns most recent encoder counts
    int32_t  getCountLast(uint8_t moduleNum) { return modules_.at(moduleNum).getCountLast(); } // returns most recent motor position
    int32_t  readCountCurrent(uint8_t moduleNum){ return modules_.at(moduleNum).readCount(); } // reads (via SPI) the current encoder count
    bool     resetCount(uint8_t moduleNum);   // resets the encoder count to zero
    bool     resetCounts(void); // resets all encoders to zero

    void stepPid(void); // PID controller performs one step (reads encoders, computes effort, updates DACs)
    void restartPid(uint8_t position) { modules_.at(position).restartPid(); } // resets PID controller, but keeps gains

    void setLEDG(uint8_t position, bool state); // sets green LED
	void setLEDG(bool state); // sets all green LEDs
	void toggleLEDG(uint8_t position); // toggle state of green LED
	
    void setDACs(Int16Vec const &val); // manually set DAC outputs
	
	Uint32Vec readButtons(void);	 // check capacitive sensor buttons for key presses
	bool isDownPressed(void) { return pins.buttonStates[0]; };
	bool isUpPressed(void) { return pins.buttonStates[1]; }
    bool isMenuPressed(void) { return pins.buttonStates[2]; }
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

    ErrorCode errorCode_ = ErrorCode::NO_ERROR;
    volatile bool ampEnableFlag_ = true; // this flag is set by ampEnabledISR whenever ampEnabled_ state changes
    std::vector<LimitSwitch> triggeredLimitSwitches_; // vector of any currently triggered limit switches/e-stop; reset with resetTriggeredLimitSwitches()
    BoolVec limitSwitchState_ = { LOW, LOW, LOW, LOW, LOW, LOW }; // current state of each limit switch
    BoolVec ampEnabled_ = { false, false, false, false, false, false }; // current state of each amp's enable pin
    bool    brakeHwState_ = HIGH; // HIGH = all amps DISABLED
    bool    eStopState_ = false; // current state of E-stop/hardware brake (true = triggered/activated)

    BoolVec LEDG_ = { LOW, LOW, LOW, LOW, LOW, LOW }; // Green LED status (true = on)

    //int8_t  whichDevice(void);  // returns the index (0-5) of the motor that caused the interrupt; E-stop/hardware brake = 6
    //Int8Vec whichDevices(void); // use if there are multiple pins interrupted; i.e. when whichLimitSwitch() = -1
};

#endif