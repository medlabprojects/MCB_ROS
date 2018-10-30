/*=========================================================================//
	
	MCBmodule Class
	
	This class handles the control of an individual daughterboard module
	
	Designed for use with Teensy 3.1 and Motor Control Board (Rev 1.2)
	
	
	Trevor Bruns
	
	Changelog-
		2/10/2016: Initial Creation
		3/13/2017: Compiles and appears to run successfully
		
//=========================================================================*/

#ifndef MCBmodule_h
#define MCBmodule_h

#include <stdint.h>
#include "LS7366R.h"
#include "PID_f32.h"

class MCBmodule
{
public:
	MCBmodule(uint8_t csEnc); // needs to know SPI chip-select pin for it's encoder
	~MCBmodule(void);
	
	bool init(float kp, float ki, float kd); // initializes encoder/PID controller and enables module
	bool init(void); // ^^ except initializes with PID gains all set to 0.0
    bool isConfigured(void) { return configured_; } // returns true after proper initializationa

    // Motor
    void setMotorPolarity(bool polarity) { motorPolarity_ = polarity; } // for brushed motors if +/- phases are opposite of what encoder expects

	// Encoder
    void setCountDesired(int32_t countDesired) { countDesired_ = countDesired; }; // sets target position for motor in encoder counts
    int32_t getCountDesired(void) { return countDesired_; }
	int32_t readCount(void);    // reads current position from encoder
    int32_t getCountLast(void) { return countLast_; } // returns result of most recent read_count(), does NOT query encoder
    bool    resetCount(void);   // resets the encoder count to zero

	// PID Controller
	uint16_t step(void);	// steps the PID controller, returns next DAC command
    void restartPid(void) { pid_.reset(); } // call this after changing gains, resets state buffer to zeros
    int32_t getError(void) { return countError_; } // [counts] returns last computed error
    float getEffort(void) { return effort_; } // [volts] returns last computed effort
    void setGains(float kp, float ki, float kd) { pid_.setGains(kp, ki, kd); }
    void setKp(float kp) { pid_.setKp(kp); }
    float getKp(void) { return pid_.getKp(); }
    void setKi(float ki) { pid_.setKi(ki); };
    float getKi(void) { return pid_.getKi(); }
    void setKd(float kd) { pid_.setKd(kd); }
    float getKd(void) { return pid_.getKd(); }
	uint16_t effortToDacCommand(float effort); // converts a motor effort [volts] to a DAC command [0,2^16]                    

private:
    bool configured_ = false;

    // Motor
    bool motorPolarity_ = 1; // used to make sure positive current -> positive encoder counts

	// DAC
    float dacRange_[2] = { -10.0, 10.0 }; // [volts] DAC output range; PID effort will be scaled/saturated based on these values

	// Encoder
	LS7366R enc_; // Quadrature encoder interface
	
	// PID
	PID_f32 pid_; // PID controller, 32-bit float version
	int32_t countLast_ = 0;
	volatile int32_t countDesired_ = 0;
	int32_t	countError_ = 0;
	float effort_ = 0; // unsaturated, computed effort from controller in Amps
};

#endif
