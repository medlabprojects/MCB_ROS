#include "MCB.h"
#include "MCBmodule.h"
#include "MCBpins.h"
#include <ArduinoSTL.h> 
#include "si5351.h"
#include <SPI.h>

MCB::MCB(void)
	: DAC_(pins.csDac)	
{	
	// reserve memory for modules
	modules_.reserve(pins.maxNumBoards);
	DACval_.reserve(pins.maxNumBoards);
}

int MCB::init(bool ignoreErrors /* = false */)
// detects and initializes daughterboard motor modules
// returns number of modules detected
// returns -1 to indicate error. Read error code with getErrorCode()
{
    numModules_ = 0;

	// initialize MCB pins (if not already)
	if (!isPinsInit) {
		pins.init();
		isPinsInit = true;
	}
	
    // Determine initial states of limit switches and e-stop
    initLimitSwitchStates(); // sets errorCode_

    if (errorCode_ != MCB::ErrorCode::NO_ERROR && !ignoreErrors) {
        return -1;
    }


	// initialize encoder clock used by all LS7366R 
	si5351_.init(SI5351_CRYSTAL_LOAD_8PF, 0);
	si5351_.set_freq(50000000ULL, 0ULL, SI5351_CLK0); // [hundreths of Hz] Set CLK0 to output 500 kHz
	si5351_.output_enable(SI5351_CLK1, 0); // Disable other clocks
	si5351_.output_enable(SI5351_CLK2, 0);
	
	// initialize SPI
	SPI.begin();
	
	// create and attempt to initialize each module (along with each encoder IC)
	for (uint8_t ii = 0; ii < pins.maxNumBoards; ii++)
	{
		addModule(ii); // creates a module and adds to modules vector

        if (isModuleConfigured(ii)) {
            numModules_++;
        }
	}

    // check that all detected boards are in a row starting from the first socket (i.e. no gaps) 
    for (uint8_t ii = 0; ii < numModules_; ii++)
    {
        if (!isModuleConfigured(ii))
        {
            errorCode_ = MCB::ErrorCode::WRONG_MODULE_ORDER;
            return -1; // error: incorrect module order
        }
    }
	
	// initialize 
    initDACs();
	//DAC_.beginTransfer();
	//for (uint8_t bb = 0; bb < numModules_; bb++)
	//{
	//	DAC_.reset(); // software reset
	//}
	//DAC_.endTransfer();
	//
	//DAC_.beginTransfer();
	//for (uint8_t bb = 0; bb < numModules_; bb++)
	//{
	//	DAC_.init(); // setup ctrl register
	//}
	//DAC_.endTransfer();

	//DAC_.beginTransfer();
	//for (uint8_t bb = 0; bb < numModules_; bb++)
	//{
	//	DAC_.set(0); // Set output to 0 volts
	//}
	//DAC_.endTransfer();

    return numModules_;
}

void MCB::initDACs(void)
{
    // initialize DACs
    DAC_.beginTransfer();
    for (uint8_t bb = 0; bb < numModules_; bb++)
    {
        DAC_.reset(); // software reset
    }
    DAC_.endTransfer();

    DAC_.beginTransfer();
    for (uint8_t bb = 0; bb < numModules_; bb++)
    {
        DAC_.init(); // setup ctrl register
    }
    DAC_.endTransfer();

    DAC_.beginTransfer();
    for (uint8_t bb = 0; bb < numModules_; bb++)
    {
        DAC_.set(0); // Set output to 0 volts
    }
    DAC_.endTransfer();
}

void MCB::waitForButtonHold(void)
{
    // initialize MCB pins (if not already)
    if (!isPinsInit) {
        pins.init();
        isPinsInit = true;
    }

    uint32_t holdTime = 2000; // [ms] how long buttons must be held before function returns
    uint32_t timeButtonsPressed = 0; // [ms] how long buttons have been held
    uint32_t timeStart = 0;

    setLEDG(false);

    // keep checking MCB buttons until Menu/Up/Down are all held for 2 seconds
    while (timeButtonsPressed < holdTime)
    {
        // check buttons
        readButtons();
        if (isDownPressed() && isUpPressed() && isMenuPressed()) {
            // if just pressed
            if (timeButtonsPressed == 0)
            {
                timeStart = millis();
                delay(1); // ensure next millis() call will be different
            }

            // light LEDs in sequence for user feedback
            if (timeButtonsPressed < (holdTime / 7)) {
                if (!LEDG_[0]) { setLEDG(0, HIGH); }
            }
            else if (timeButtonsPressed < (2 * holdTime / 7)) {
                if (!LEDG_[1]) { setLEDG(1, HIGH); }
            }
            else if (timeButtonsPressed < (3 * holdTime / 7)) {
                if (!LEDG_[2]) { setLEDG(2, HIGH); }
            }
            else if (timeButtonsPressed < (4 * holdTime / 7)) {
                if (!LEDG_[3]) { setLEDG(3, HIGH); }
            }
            else if (timeButtonsPressed < (5 * holdTime / 7)) {
                if (!LEDG_[4]) { setLEDG(4, HIGH); }
            }
            else if (timeButtonsPressed < (6 * holdTime / 7)) {
                if (!LEDG_[5]) { setLEDG(5, HIGH); }
            }
            else {
                setLEDG(LOW);
                delayMicroseconds(50000);
                setLEDG(HIGH);
                delayMicroseconds(50000);
            }

            timeButtonsPressed = millis() - timeStart;
        }
        else {
            timeButtonsPressed = 0;
            setLEDG(false);
        }
    }
    setLEDG(false);
    delay(500); // for visual confirmation
}

void MCB::addModule(uint8_t position)
{
    DACval_.push_back(0); // initialize DAC output values to 0
	modules_.push_back(MCBmodule(pins.csEnc[position])); // create new MCBmodule and add to storage vector
    moduleConfigured_.push_back(false);
	moduleConfigured_.at(position) = modules_.at(position).init(); // initialize modules
}

void MCB::setPolarity(uint8_t position, bool polarity)
{
    modules_.at(position).setMotorPolarity(polarity);
}

void MCB::setPolarity(bool polarity)
{
    for (uint8_t aa = 0; aa < pins.maxNumBoards; aa++)
    {
        setPolarity(aa, polarity);
    }
}

bool MCB::isAmpEnabled(uint8_t position)
{
    // first check if update is needed
    if (ampEnableFlag_) {
        updateAmpStates();
    }

    if (position >= numModules_) {
        return 0;
    }
    return ampEnabled_.at(position);
}

bool MCB::enableAmp(uint8_t position)
{
    bool success = false;

    // verify valid input 
    if (position >= numModules_) {
        return success;
    }

    // check if update is needed first
    if (ampEnableFlag_) {
        updateAmpStates();
    }

    // ensure module has been configured
    if (isModuleConfigured(position)) 
    {
        // check that amp is not already enabled and that e-stop is disabled
        if (!isAmpEnabled(position) && !eStopState_)
        {
            // prevent sudden movement once powered
            restartPid(position); // restart the PID controller
            readCountCurrent(position); // step PID to update encoder position
            setCountDesired(position, getCountLast(position)); // set desired count to current

            // amp is enabled when ampCtrl == limitSwitchState
            digitalWriteFast(pins.ampCtrl[position], limitSwitchState_.at(position));
            ampEnabled_[position] = true; // update amp state
           
            // the changing ampEnabled pin triggers the interrupt again
            // since we are aware of this (we caused it), it is safe to reset
            pins.i2cPins.resetInterrupts();
            //ampEnableFlag_ = false;
        }

        success = true;
    }
    else {
        success = false;
    }

    return success;
}

bool MCB::disableAmp(uint8_t position)
{
    bool success = false;

    // verify valid input 
    if (position >= numModules_) {
        return success;
    }

    // check if update is needed first
    if (ampEnableFlag_) {
        updateAmpStates();
    }

    // ensure module has been configured AND e-stop is not currently triggered
    if (isModuleConfigured(position) && !eStopState())
    {
        // check that amp is not already disabled
        if (isAmpEnabled(position))
        {
            // prevent sudden movement once powered
            setCountDesired(position, readCountCurrent(position)); // sync current/desired position
            restartPid(position); // restart the PID controller
            DACval_.at(position) = modules_.at(position).effortToDacCommand(0.0); // set DAC to command 0 amps

            // amp is disabled when ampCtrl != limitSwitchState
            digitalWriteFast(pins.ampCtrl[position], !limitSwitchState_.at(position));
            ampEnabled_[position] = false; // update amp state

            // the changing ampEnabled pin triggers the interrupt again
            // since we are aware of this (we caused it), it is safe to reset
            pins.i2cPins.resetInterrupts();
            //ampEnableFlag_ = false;
        }

        success = true;
    }
    else {
        success = false;
    }

    return success;
}

bool MCB::disableAllAmps(void)
{
    bool success = true;

    // disable all motor amp outputs
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
        if (!disableAmp(aa)) {
            success = false;
        }
	}

    return success; // false if any were unsuccessful
}

bool MCB::enableAllAmps(void)
{
    bool success = true;

    // set globalInhibit false
    setGlobalInhibit(false);

	// enable all motor amp outputs
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
        if (!enableAmp(aa)) {
            success = false;
        }
	}

    return success; // false if any were unsuccessful
}

bool MCB::setGlobalInhibit(bool inhibit, bool eStopCheck /* = true */)
{
    // setting FALSE  => each amp will be enabled depending on ampCtrlState_ and limitSwitchState_
    // setting TRUE   => all amps DISABLED (regardless of ampCtrlState_ and limitSwitchState_)

    bool success = false;

    if (inhibit) {
        // if we want global inhibit TRUE, simply set the output 
        digitalWriteFast(pins.globalInhibit, inhibit);
        success = true;
    }
    else if (eStopCheck) {
        // if we want global inhibit FALSE, we must first ensure the e-stop is not triggered
        if (!eStopState_) {
            digitalWriteFast(pins.globalInhibit, inhibit);

            // verify that hardware brake line has been set correctly (i.e. FALSE/LOW)
            delayMicroseconds(1); // short delay to allow for signal propogation
            brakeHwState_ = bitRead(pins.i2cPins.readGPIO(), pins.i2cBrakeHw);
            if (inhibit == brakeHwState_) {
                success = true;
            }
        }
    }
    else {
        // forcefully override e-stop check and set FALSE
        digitalWriteFast(pins.globalInhibit, inhibit);
        success = true;
    }
    
    return success;
}

bool MCB::updateAmpStates(void)
{
    /*
    Returns true if the e-stop or a limit switch was triggered
    Returns false if ampEnabled has not changed OR was changed by the user (via enable/disable functions) 

                AMP ENABLE TRUTH TABLE
    eStopState | limitSwitchState | ampCtrlState |=| ampEnabled
    -----------------------------------------------------------
        0      |        0         |       0      |=|     1
        0      |        0         |       1      |=|     0
        0      |        1         |       0      |=|     0
        0      |        1         |       1      |=|     1
        1      |        0         |       0      |=|     0
        1      |        0         |       1      |=|     0
        1      |        1         |       0      |=|     0
        1      |        1         |       1      |=|     0

    E-Stop triggered => amps always off
    limitSwitchState /= ampCtrlState => amps disabled
    limitSwitchState == ampCtrlState => amps enabled
    */

    bool triggered = false;
    
    triggeredLimitSwitches_.clear(); // reset
    
    bool eStopStatePrevious = eStopState_;
    bool globalInhibitState = digitalReadFast(pins.globalInhibit);

    // read current states
    uint8_t i2cStates = pins.i2cPins.readGPIO();
    ampEnabled_[0] = bitRead(i2cStates, pins.i2cEnableM0);
    ampEnabled_[1] = bitRead(i2cStates, pins.i2cEnableM1);
    ampEnabled_[2] = bitRead(i2cStates, pins.i2cEnableM2);
    ampEnabled_[3] = bitRead(i2cStates, pins.i2cEnableM3);
    ampEnabled_[4] = bitRead(i2cStates, pins.i2cEnableM4);
    ampEnabled_[5] = bitRead(i2cStates, pins.i2cEnableM5);
    brakeHwState_  = bitRead(i2cStates, pins.i2cBrakeHw); // HIGH = all amps DISABLED

    // update eStopState_
    if (brakeHwState_ && !globalInhibitState) {
        // if brakeHwState_ is true, but globalInhibitState is false, then the e-stop must have been triggered
        eStopState_ = true;
    }
    else {
        eStopState_ = false;
    }
    

    if (eStopState_ != eStopStatePrevious) // e-stop triggered
    {   
        triggeredLimitSwitches_.push_back(MCB::LimitSwitch::ESTOP);

        // to be safe, re-initialize all limit switch states when first switching off e-stop and ensure all amps are disabled
        if (!eStopState_) {
            initLimitSwitchStates(); // disables all motors and sets globalInhibit true
        }

        triggered = true;
    }
    else if (!eStopState_) // e-stop was not triggered and is currently disabled (NOTE: limit switch states can only be inferred when e-stop is disabled)
    {
        bool limitSwitchStateTemp;

        // update limitSwitchState_
        for (uint8_t aa = 0; aa < ampEnabled_.size(); aa++) {
            if (ampEnabled_.at(aa)) {
                // amp is only enabled when limitSwitchState = ampCtrl
                limitSwitchStateTemp = digitalReadFast(pins.ampCtrl[aa]);
            }
            else {
                // if amp is disabled then limitSwitchState = !ampCtrl
                limitSwitchStateTemp = !digitalReadFast(pins.ampCtrl[aa]);
            }

            // compare against previous limitSwitchState
            if (limitSwitchState_[aa] != limitSwitchStateTemp) {
                triggered = true;
                triggeredLimitSwitches_.push_back(positionToLimitSwitch(aa)); // store which was triggered
                limitSwitchState_[aa] = limitSwitchStateTemp; // update
            }
            else {
                // ampEnabled must have changed due to ampCtrl or globalInhibit (via the user), not limitSwitch
            }
        }

        if (triggeredLimitSwitches_.size() == 0) {
            // ampEnabled must have changed due to ampCtrl (via the user), not limitSwitch
            triggeredLimitSwitches_.clear(); // ensure vector is empty
        }
    }

    // update green LEDs to indicate limit switch state (on = switch closed)
    for (uint8_t aa = 0; aa < pins.maxNumBoards; aa++) {
        setLEDG(aa, limitSwitchState_.at(aa));
    }

    // ensure interrupt pin has been reset (active low so should be high)
    while (!digitalReadFast(pins.i2cInt)) {
        pins.i2cPins.resetInterrupts();
    }

    // reset flag
    ampEnableFlag_ = false;

    return triggered;
}

MCB::ErrorCode MCB::initLimitSwitchStates(void)
{   
    // This function determines the state of all limit switches and sets ampCtrl pins to ensure amps remain disabled when global enable is set true
    // returns errorCode_

     triggeredLimitSwitches_.clear(); // reset
     errorCode_ = MCB::ErrorCode::NO_ERROR; // reset

    // ensure global inhibit is true
    setGlobalInhibit(true);

    // ensure ampCtrl pins are in default state (HIGH = amps disabled, since limitSwitchState_ should be LOW if not triggered)
    for (uint8_t ii = 0; ii < pins.maxNumBoards; ii++)
    {
        digitalWriteFast(pins.ampCtrl[ii], HIGH); 
    }

    // briefly set globalInhibit false and read enable lines
    setGlobalInhibit(false, false); // override e-stop check
    delayMicroseconds(1);
    uint8_t i2cStates = pins.i2cPins.readGPIO();
    setGlobalInhibit(true);

    BoolVec ampEnabled(6, false);
    bool brakeHwState;
    ampEnabled[0] = bitRead(i2cStates, pins.i2cEnableM0);
    ampEnabled[1] = bitRead(i2cStates, pins.i2cEnableM1);
    ampEnabled[2] = bitRead(i2cStates, pins.i2cEnableM2);
    ampEnabled[3] = bitRead(i2cStates, pins.i2cEnableM3);
    ampEnabled[4] = bitRead(i2cStates, pins.i2cEnableM4);
    ampEnabled[5] = bitRead(i2cStates, pins.i2cEnableM5);
    brakeHwState  = bitRead(i2cStates, pins.i2cBrakeHw);

    // if e-stop is not currently triggered, then brakeHwState should have gone LOW when we set globalInhibit LOW/FALSE
    eStopState_ = brakeHwState; 

    // check if any amps were enabled, and update ampCtrl to ensure it won't next time
    if (eStopState_) { // e-stop is triggered
        triggeredLimitSwitches_.push_back(MCB::LimitSwitch::ESTOP);
        errorCode_ = MCB::ErrorCode::ESTOP_TRIGGERED;
    }
    else { // can only determine limit switch states if e-stop is not triggered
        for (uint8_t ii = 0; ii < pins.maxNumBoards; ii++) {
            if (ampEnabled.at(ii)) {
                // since amp was enabled, limitSwitchState must be HIGH, and we must set ampCtrl LOW to ensure amp will be disabled
                limitSwitchState_[ii] = HIGH;
                digitalWriteFast(pins.ampCtrl[ii], LOW);
                triggeredLimitSwitches_.push_back(positionToLimitSwitch(ii));
                errorCode_ = MCB::ErrorCode::LIMIT_SWITCH_TRIGGERED_ON_STARTUP;
            }

            // all amps should now be in disabled state
            ampEnabled_[ii] = false;
        }
    }

    // update green LEDs to indicate limit switch state (on = switch closed)
    for (uint8_t aa = 0; aa < pins.maxNumBoards; aa++) {
        setLEDG(aa, limitSwitchState_.at(aa));
    }

    // ensure interrupt pin has been reset (active low so should be high)
    while (!digitalReadFast(pins.i2cInt)) {
        pins.i2cPins.resetInterrupts();
    }

    return errorCode_;
}

MCB::LimitSwitch MCB::positionToLimitSwitch(uint8_t position)
{
    MCB::LimitSwitch tmp;

    if (position > 5) {
        tmp = MCB::LimitSwitch::ERROR;
    }
    else {
        switch (position)
        {
        case 0:
            tmp = MCB::LimitSwitch::LIMIT_M0;
            break;
        case 1:
            tmp = MCB::LimitSwitch::LIMIT_M1;
            break;
        case 2:
            tmp = MCB::LimitSwitch::LIMIT_M2;
            break;
        case 3:
            tmp = MCB::LimitSwitch::LIMIT_M3;
            break;
        case 4:
            tmp = MCB::LimitSwitch::LIMIT_M4;
            break;
        case 5:
            tmp = MCB::LimitSwitch::LIMIT_M5;
            break;
        default:
            tmp = MCB::LimitSwitch::ERROR;
            break;
        }
    }
    
    return tmp;
}

uint8_t MCB::limitSwitchToPosition(LimitSwitch limitSwitch)
{
    uint8_t position;

    switch (limitSwitch)
    {
    case MCB::LimitSwitch::LIMIT_M0:
        position = 0;
        break;
    case MCB::LimitSwitch::LIMIT_M1:
        position = 1;
        break;
    case MCB::LimitSwitch::LIMIT_M2:
        position = 2;
        break;
    case MCB::LimitSwitch::LIMIT_M3:
        position = 3;
        break;
    case MCB::LimitSwitch::LIMIT_M4:
        position = 4;
        break;
    case MCB::LimitSwitch::LIMIT_M5:
        position = 5;
        break;
    case MCB::LimitSwitch::ESTOP:
        position = 7;
        break;
    case MCB::LimitSwitch::ERROR:
        position = 0xFF;
        break;
    default:
        position = 0xFF;
        break;
    }

    return position;
}

FloatVec MCB::getGains(uint8_t position)
{
    FloatVec gains;
    gains.push_back(modules_.at(position).getKp());
    gains.push_back(modules_.at(position).getKi());
    gains.push_back(modules_.at(position).getKd());

    return gains;
}

void MCB::stepPid(void)
{
	// step PID controllers
	for (uint8_t aa = 0; aa < modules_.size(); aa++)
	{
        // only step controller if amp is enabled to prevent integral windup
        if (isAmpEnabled(aa)) {
            DACval_.at(aa) = modules_.at(aa).step();
        }
	}
	
	// update DACs
	setDACs(DACval_);
}

void MCB::setDACs(Int16Vec const &val)
{

	DAC_.beginTransfer();

	// send out in reverse order (data is pushed through the daisy chain)
	for (uint8_t bb = val.size(); bb > 0; bb--)
	{
		DAC_.set(val.at(bb - 1));
	}

	DAC_.endTransfer();
}

void MCB::setLEDG(uint8_t position, bool state)
{
	LEDG_.at(position) = state;
	digitalWriteFast(pins.led[position], LEDG_.at(position));
}

void MCB::setLEDG(bool state)
{
	for (int ii = 0; ii < pins.maxNumBoards; ii++)
	{
		LEDG_.at(ii) = state;
		digitalWriteFast(pins.led[ii], LEDG_.at(ii));
	}
}

//void MCB::toggleLEDG(uint8_t position)
//{
//	// if on -> set off, else turn on
//    if (LEDG_.at(position)) {
//		setLEDG(position, LOW);
//	}
//	else {
//		setLEDG(position, HIGH);
//	}
//}

//void MCB::toggleLEDG(void)
//{
//    // toggle all together, synced to LEDG_[0]
//    setLEDG(!LEDG_.at(0));
//}

Int32Vec MCB::getCountsDesired(void)
{
    Int32Vec countsDesired(pins.maxNumBoards);
    for (int ii = 0; ii < numModules_; ii++) {
        countsDesired.at(ii) = modules_.at(ii).getCountDesired();
    }
    return countsDesired;
}

Int32Vec MCB::getCountsLast(void)
{
	Int32Vec countsLast(pins.maxNumBoards);
	for (uint8_t aa = 0; aa < modules_.size(); aa++)
	{
        countsLast.at(aa) = modules_.at(aa).getCountLast();
	}
	
	return countsLast;
}

bool MCB::resetCount(uint8_t moduleNum)
{
    bool success = false;

    // verify module number exists
    if (moduleNum < numModules_) {
        // ensure module is disabled before zeroing encoder count
        disableAmp(moduleNum);
        success = modules_[moduleNum].resetCount();
    }
   
    return success;
}

bool MCB::resetCounts(void)
{
    bool success = true;

    for (int ii = 0; ii < numModules_; ii++) {
        // unsuccessful if any fail
        if (!resetCount(ii)) {
            success = false; 
        }
    }

    return success;
}

Uint32Vec MCB::readButtons(void)
{	
	uint8_t numButtons = sizeof(pins.buttons);
	Uint32Vec buttonValues;
	buttonValues.resize(numButtons);
	
	for (uint8_t aa = 0; aa < numButtons; aa++)
	{
		buttonValues.at(aa) = touchRead(pins.buttons[aa]);
		pins.buttonStates[aa] = (buttonValues[aa] > pins.buttonThresh[aa]);
	}
	
	return buttonValues;
}

bool MCB::isEverythingPressed(void)
{
	if (isMenuPressed() && isUpPressed() && isDownPressed()) {
		return true;
	}
	else {
		return false;
	}
}

MCB::~MCB(void)
{
	// ensure all amps are disabled
	disableAllAmps();
    setGlobalInhibit(true);
}
