#include "MCB.h"
#include "MCBmodule.h"
#include "MCBpins.h"
#include <ArduinoSTL.h> 
#include "si5351.h"
#include <SPI.h>

MCB::MCB(void)
	: DAC_(pins.csDAC)	
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
	si5351_.set_freq(3000000000ULL, 0ULL, SI5351_CLK0); // Set CLK1 to output 30 MHz
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
	modules_.push_back(MCBmodule(position)); // create new MCBmodule and add to storage vector
    moduleConfigured_.push_back(false);
	moduleConfigured_.at(position) = modules_.at(position).init(); // initialize modules
}

uint8_t MCB::numModules(void)
{
    return numModules_;
}

void MCB::enableAmp(uint8_t position)
{
	// software brakes (HIGH = amp enabled)
	digitalWriteFast(pins.ampEnable[position], HIGH);
}

void MCB::disableAmp(uint8_t position)
{
	// software brakes (LOW = amp disabled)
	digitalWriteFast(pins.ampEnable[position], LOW);
}

void MCB::disableAllAmps(void)
{
	for (uint8_t aa = 0; aa < pins.maxNumBoards; aa++)
	{
		// software brakes (LOW = amps disabled)
		digitalWriteFast(pins.ampEnable[aa], LOW);
	}
}

void MCB::enableAllAmps(void)
{
	// enable motor amp outputs and turn on green LEDs
	for (uint8_t aa = 0; aa < numModules_; aa++)
	{
		// software brakes (HIGH = amps enabled)
		digitalWriteFast(pins.ampEnable[aa], HIGH);
		setLEDG(aa, HIGH);
	}
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

void MCB::setMaxAmps(uint8_t position, float maxAmps)
{
	modules_.at(position).setMaxAmps(maxAmps);
}

float MCB::getMaxAmps(uint8_t position)
{
	return modules_.at(position).getMaxAmps();
}

void MCB::stepPID(void)
{
	
	// step PID controllers
	for (uint8_t aa = 0; aa < modules_.size(); aa++)
	{
		DACval_.at(aa) = modules_.at(aa).step();
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
	digitalWriteFast(pins.LEDG[position], LEDG_.at(position));
}

void MCB::setLEDG(bool state)
{
	for (int ii = 0; ii < pins.maxNumBoards; ii++)
	{
		LEDG_.at(ii) = state;
		digitalWriteFast(pins.LEDG[ii], LEDG_.at(ii));
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

Int32Vec MCB::getCountsLast(void)
{
	Int32Vec temp;
	for (uint8_t aa = 0; aa < modules_.size(); aa++)
	{
		temp.at(aa) = modules_.at(aa).getCountLast();
	}
	
	return temp;
}

int32_t MCB::getCountLast(uint8_t moduleNum)
{	
	return modules_.at(moduleNum).getCountLast();
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
    return moduleConfigured_.at(position);
}

MCB::~MCB(void)
{
	// ensure all amps are disabled
	disableAllAmps();
}
