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

int MCB::init(void)
// detects and initializes modules
// returns number of modules detected
// returns -1 to indicate incorrect module configuration (i.e. not in order starting from first socket)
{
    numModules_ = 0;

	// initialize MCB pins (if not already)
	if (!isPinsInit) {
		pins.init();
		isPinsInit = true;
	}
	
	// initialize encoder clock used by all LS7366R 
	si5351_.init(SI5351_CRYSTAL_LOAD_8PF, 0);
	si5351_.set_freq(2000000000ULL, 0ULL, SI5351_CLK0); // Set CLK1 to output 20 MHz (max frequency when LS7366R Vdd = 3.3V)
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
            return -1; // error: incorrect module configuration
        }
    }
	
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

    return numModules_;
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
    ampEnabled_.push_back(false);
}

uint8_t MCB::numModules(void)
{
    return numModules_;
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
    if (position >= numModules_) {
        return 0;
    }
    return ampEnabled_.at(position);
}

bool MCB::enableAmp(uint8_t position)
{
    bool success = false;

    // ensure module has been configured
    if (isModuleConfigured(position)) 
    {
        // check that amp is not already enabled
        if (!isAmpEnabled(position)) 
        {
            // amp is enabled when ampCtrlState == limitSwitchState
            ampCtrlState_[position] = limitSwitchState_.at(position);
            digitalWriteFast(pins.ampCtrl[position], ampCtrlState_.at(position));

            // update amp state
            ampEnabled_[position] = true;
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

    // ensure module has been configured
    if (isModuleConfigured(position))
    {
        // check that amp is not already disabled
        if (isAmpEnabled(position))
        {
            // amp is disabled when ampCtrlState != limitSwitchState
            ampCtrlState_[position] = !limitSwitchState_.at(position);
            digitalWriteFast(pins.ampCtrl[position], ampCtrlState_.at(position));

            // update amp state
            ampEnabled_[position] = false;
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

	// enable all motor amp outputs
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
        if (!enableAmp(aa)) {
            success = false;
        }
	}

    return success; // false if any were unsuccessful
}

void MCB::updateLimitSwitchStates(void)
{
    /*
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

    */


    // first check if e-stop is enabled
    uint8_t i2cStates = pins.i2cPins.readGPIO();
    eStopState_ = bitRead(i2cStates, pins.i2cBrakeHw);

    if (!eStopState_)
    {
        // ampEnabled_ states can only be inferred if e-stop is not enabled
        ampEnabled_[0] = bitRead(i2cStates, pins.i2cEnableM0);
        ampEnabled_[1] = bitRead(i2cStates, pins.i2cEnableM1);
        ampEnabled_[2] = bitRead(i2cStates, pins.i2cEnableM2);
        ampEnabled_[3] = bitRead(i2cStates, pins.i2cEnableM3);
        ampEnabled_[4] = bitRead(i2cStates, pins.i2cEnableM4);
        ampEnabled_[5] = bitRead(i2cStates, pins.i2cEnableM5);

        // infer states of limit switches
        for (uint8_t aa = 0; aa < ampEnabled_.size(); aa++) {
            if (ampEnabled_.at(aa) == ampCtrlState_.at(aa)) {
                // amp is only enabled when ampCtrlState = limitSwitchState
                limitSwitchState_[aa] = ampCtrlState_.at(aa);
            }
            else {
                limitSwitchState_[aa] = !ampCtrlState_.at(aa);
            }
        }
    }
    else // e-stop triggered
    {
        // for safety, ensure amps will be disabled after e-stop is disabled
        disableAllAmps();
    }

    // update green LEDs to indicate limit switch state
    for (uint8_t aa = 0; aa < limitSwitchState_.size(); aa++) {
        setLEDG(aa, limitSwitchState_.at(aa));
    }

    // reset interrupt
    pins.i2cPins.resetInterrupts(); // NOTE: shouldn't be needed since readGPIO() above should reset it 
}

//void MCB::processLimitSwitch(void)
//{
//    // determine which device triggered the interrupt
//    int8_t device = whichLimitSwitch();
//
//    if (device == -2) 
//    {
//        // none detected
//        return; 
//    }
//
//    setLEDG(false);
//
//    // check if more than one device was triggered
//    // E-stop/hardware brake will cause all enable pins to trigger
//    if (device == -1) {
//
//        Int8Vec devices = whichLimitSwitches();
//
//        for (int ii = 0; ii < devices.size(); ii++) {
//            limitSwitchTriggered_[devices.at(ii)] = true;
//
//            // indicate with green LEDs
//            setLEDG(devices.at(ii), true);
//
//            // sync desired with current count to prevent movement when power is restored
//            setCountDesired(devices.at(ii), getCountLast(devices.at(ii)));
//
//            // restart PID controller to prevent windup
//            restartPid(devices.at(ii));
//        }
//    }
//
//
//    // check if triggering device is a limit switch
//    if ((device >= 0) && (device <= 5)) {
//        limitSwitchTriggered_[device] = true;
//
//        // indicate with green LED
//        setLEDG(device, true);
//
//        // zero out encoder to prevent movement when power is restored
//        setCountDesired(device, getCountLast(device));
//
//        // reset PID controller to prevent windup
//        restartPid(device);
//    }
//    // check if more than one device was triggered
//    else if (device == -1) {
//        
//        Int8Vec devices = whichLimitSwitches();
//
//        for (int ii = 0; ii < devices.size(); ii++) {
//            limitSwitchTriggered_[devices.at(ii)] = true;
//
//            // indicate with green LEDs
//            setLEDG(devices.at(ii), true);
//
//            // sync desired with current count to prevent movement when power is restored
//            setCountDesired(devices.at(ii), getCountLast(devices.at(ii)));
//
//            // restart PID controller to prevent windup
//            restartPid(devices.at(ii));
//        }
//    }
//    // check if it is the E-stop
//    else if (device == 6) {
//        // indicate with green LEDs
//        setLEDG(true);
//
//        for (int ii = 0; ii < numModules_; ii++) {
//            // zero out encoders to prevent movement when power is restored
//            setCountDesired(ii, getCountLast(ii));
//
//            // restart PID controller to prevent windup
//            restartPid(ii);
//        }
//    }
//}

int8_t MCB::whichLimitSwitch(void)
{
    /*
    multiple pins with interrupt conditions = -1
    no pins with interrupt conditions = -2
    0-5 -> index of motor whose limit switch was triggered
    6 -> E-stop or hardware brake was triggered

    In reality, if the E-stop is triggered it will also cause all 
    other enable pins to trigger as well. So it should always appear
    as -1 and not 6.    
    */

    int8_t device;

    int8_t triggeringPin = pins.i2cPins.whichInterrupt();

    switch (triggeringPin) {
    case -1:
        device = -1; // multiple pins triggered
        break;

    case 0:
        device = 6; // BrakeHW
        break;

    case 1:
        device = 7; // i2cGpio
        break;

    case 2:
        device = 5; // i2cEnableM5
        break;

    case 3:
        device = 4; // i2cEnableM4
        break;

    case 4:
        device = 3; // i2cEnableM3
        break;

    case 5:
        device = 2; // i2cEnableM2
        break;

    case 6:
        device = 1; // i2cEnableM1
        break;

    case 7:
        device = 0; // i2cEnableM0
        break;

    default:
        device = -2; // no pins triggered
    }

    return device;
}

Int8Vec MCB::whichLimitSwitches(void)
{
    Int8Vec devices;

    // read INTF register of MCP23008
    uint8_t triggeringPins = pins.i2cPins.readInterrupt();
    
    if (!triggeringPins) {
        devices.push_back(-1);
        return devices; // no interrupted pins detected
    }

    // check each bit to determine if it was triggered
    for (int ii = 0; ii < 8; ii++) {
        uint8_t tmp = triggeringPins;
        tmp &= (1 << ii); // isolate bit ii

        // if non-zero, determine the device corresponding to pin ii
        if (tmp) {
            switch (ii) {
            case 0:
                devices.push_back(6); // BrakeHW
                break;
            case 1:
                devices.push_back(7); // i2cGpio
                break;

            case 2:
                devices.push_back(5); // i2cEnableM5
                break;

            case 3:
                devices.push_back(4); // i2cEnableM4
                break;

            case 4:
                devices.push_back(3); // i2cEnableM3
                break;

            case 5:
                devices.push_back(2); // i2cEnableM2
                break;

            case 6:
                devices.push_back(1); // i2cEnableM1
                break;

            case 7:
                devices.push_back(0); // i2cEnableM0
                break;

            default:
                break; // no pins triggered
            }
        }
    }
    
    return devices;
}

void MCB::setGains(uint8_t position, float kp, float ki, float kd)
{
	modules_.at(position).setGains(kp, ki, kd);
}

FloatVec MCB::getGains(uint8_t position)
{
    FloatVec gains;
    gains.push_back(modules_.at(position).getKp());
    gains.push_back(modules_.at(position).getKi());
    gains.push_back(modules_.at(position).getKd());

    return gains;
}

float MCB::getEffort(uint8_t position)
{
    return modules_.at(position).getEffort();
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

void MCB::restartPid(uint8_t position)
{
    modules_.at(position).restartPid();
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

void MCB::toggleLEDG(uint8_t position)
{
	// if on -> set off, else turn on
	if (LEDG_.at(position)) {
		LEDG_.at(position) = LOW;
		setLEDG(position, LOW);
	}
	else {
		LEDG_.at(position) = HIGH;
		setLEDG(position, HIGH);
	}
}

void MCB::setCountDesired(uint8_t position, int32_t countDesired)
{
	modules_.at(position).setCountDesired(countDesired);
}

int32_t MCB::getCountDesired(uint8_t position)
{
    return modules_.at(position).getCountDesired();
}

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

int32_t MCB::getCountLast(uint8_t moduleNum)
{	
	return modules_.at(moduleNum).getCountLast();
}

bool MCB::resetCount(uint8_t moduleNum)
{
    return modules_.at(moduleNum).resetCount();
}

bool MCB::resetCounts(void)
{
    bool success = true;

    for (int ii = 0; ii < numModules_; ii++) {
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

bool MCB::isDownPressed(void)
{
	return pins.buttonStates[0];
}

bool MCB::isUpPressed(void)
{
	return pins.buttonStates[1];
}

bool MCB::isMenuPressed(void)
{
	return pins.buttonStates[2];
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

BoolVec MCB::isModuleConfigured(void)
{
    return moduleConfigured_;
}

bool MCB::isModuleConfigured(uint8_t position)
{
    if (position >= moduleConfigured_.size()) {
        return 0;
    }

    return moduleConfigured_.at(position);
}

MCB::~MCB(void)
{
	// ensure all amps are disabled
	disableAllAmps();
}
