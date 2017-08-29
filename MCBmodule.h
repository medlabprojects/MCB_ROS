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

#define MODULE_ENABLE  true
#define MODULE_DISABLE false

class MCBmodule
{
public:
	MCBmodule(uint8_t csEnc); // needs to know SPI chip-select pin for it's encoder
	~MCBmodule(void);
	
	bool init(float kp, float ki, float kd); // initializes encoder/PID controller and enables module
	bool init(void); // ^^ except initializes with PID gains all set to 0.0
	void setStatus(bool status); // start or stop module (and thus, the associated motor)
	bool getStatus(void);

    // Motor
    void setMotorPolarity(bool polarity);

	// Encoder
	void setCountDesired(int32_t countDesired); // sets target position for motor in encoder counts
	int32_t getCountDesired(void);
	int32_t readCount(void); // reads current position from encoder
	int32_t getCountLast(void); // returns result of most recent read_count(), does NOT query encoder

	// PID Controller
	uint16_t step(void);	// steps the PID controller, returns next DAC command
	void restartPid(void); // call this after changing gains, resets state buffer to zeros
	int32_t getError(void); // [counts] returns last computed error
	float getEffort(void); // [volts] returns last computed effort
	void setGains(float kp, float ki, float kd);
	void setKp(float kp);
	float getKp(void);
	void setKi(float ki);
	float getKi(void);
	void setKd(float kd);
	float getKd(void);
	uint16_t effortToDacCommand(float effort); // converts a motor effort [volts] to a DAC command [0,2^16]                    

private:
	bool status_ = false; // module starts disabled

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
