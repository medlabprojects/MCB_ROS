/*=========================================================================//

	Motor Class
	
	This class handles the control of an individual motor
	
	Designed for use with Teensy 3.1 and Motor Control Board (Rev 1.1+)
	
	
	Trevor Bruns		
		
//=========================================================================*/

#include "MCBmodule.h"
#include <stdint.h>
#include <SPI.h>
#include "LS7366R.h"
#include "PID_f32.h"
#include <math.h>

MCBmodule::MCBmodule(uint8_t csEnc)
	: enc_(csEnc) // create encoder interface
{
}

bool MCBmodule::init(float kp, float ki, float kd)
{
    // set up PID controller
    pid_.init(kp, ki, kd); 

    // set up encoder IC (LS7366R)
    int maxAttempts = 5;
    int attempts = 0;
    while (attempts < maxAttempts) {
        attempts++;
        if (enc_.init()) {
            configured_ = true;
        }
    }
    return configured_;
}

bool MCBmodule::init(void)
{
    // set up PID controller
    pid_.init();

    // set up encoder IC (LS7366R)
    int maxAttempts = 5;
    int attempts = 0;
    while (attempts < maxAttempts) {
        attempts++;
        if (enc_.init()) {
            configured_ = true;
            break;
        }
    }
    return configured_;
}

uint16_t MCBmodule::step(void)
{
    uint16_t dacCmd; // DAC command

    int8_t polarity = 1;
    if (!motorPolarity_) {
        polarity = -1;
    }

	if (isConfigured()){
		// read current motor position and compute error
		countError_ = countDesired_ - readCount();

		// step PID controller to compute effort in Amps
		effort_ = polarity * pid_.step(float(countError_));

		// enforce voltage output bounds (typically -10V to +10V) and convert to int16 for DAC
		dacCmd = effortToDacCommand(effort_);

	}
    else {
        dacCmd = effortToDacCommand(0.0); // shouldn't be called, but here for safety
    }

	return dacCmd;
}

uint16_t MCBmodule::effortToDacCommand(float effort)
{
	float effortTemp = effort;
    
	// check for saturation
    float eps = 1e-6; // machine epsilon 
	if (effort > (dacRange_[1] + eps)) { 
        effortTemp = dacRange_[1]; 
    }
	else if (effort < (dacRange_[0] - eps)) { 
        effortTemp = dacRange_[0]; 
    }

    // encode effort to 16-bit DAC code
    // DAC code = (2^16)*(effort - Vmin)/(Vmax - Vmin)
    return static_cast<uint16_t>( 65535.0f * (effortTemp - dacRange_[0]) / (dacRange_[1] - dacRange_[0]) );
}

int32_t MCBmodule::readCount(void)
{
	// read LS7366R
	countLast_ = enc_.getCount();

	return countLast_;
}

bool MCBmodule::resetCount(void)
{
    bool success = false;
    uint8_t maxAttempts = 5; // number of reset attempts before giving up

    for (uint8_t attempt = 0; attempt < maxAttempts; attempt++) {
        // reset count register to zero
        enc_.resetCount();

        // call readCount() to make sure we update countLast_
        if (!readCount()) { // should be zero
            success = true;
            
            // prevent suddent movement upon re-enabling motor
            restartPid();
            setCountDesired(0);

            break;
        }
    }
    
    return success;
}

MCBmodule::~MCBmodule(void)
{
}
