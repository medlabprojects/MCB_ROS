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
	MCBmodule(uint8_t position);
	~MCBmodule(void);
	
	void init(float kp, float ki, float kd); // initializes encoder/PID controller and enables module
	void init(void); // ^^ except initializes with PID gains all set to 0.0
	void setStatus(bool status); // start or stop module (and thus, the associated motor)
	bool getStatus(void); 

	// DAC
	void setMaxAmps(float maxAmps); // [Amps] set based on motor current corresponding to max DAC output
	float getMaxAmps(void);         // DOES NOT directly limit current!! This must be done via ESCON Studio software

	// Encoder
	uint8_t getENCpin(void);	 // returns LS7366R chip select pin
	void setCountDesired(int32_t countDesired); // sets target position for motor in encoder counts
	int32_t getCountDesired(void);
	int32_t readCount(void); // reads current position from encoder
	int32_t getCountLast(void); // returns result of most recent read_count(), does NOT query encoder

	// PID Controller
	int16_t step(void);	// steps the PID controller, returns next DAC value
	void restartPID(void); // call this after changing gains, resets state buffer to zeros
	int32_t getError(void); // [counts] returns last computed error
	float getEffort(void); // [Amps] returns last computed effort
	void setGains(float kp, float ki, float kd);
	void setKp(float kp);
	float getKp(void);
	void setKi(float ki);
	float getKi(void);
	void setKd(float kd);
	float getKd(void);
	int16_t convertEffortToDAC(float effort); // converts a motor effort [Amps] to a DAC command
                                  

private:
	uint8_t position_; // module position on the motor board (i.e. physical slot)
	bool status_ = false; // module starts disabled
	
	// DAC
	float maxAmps_ = 1.0; // [Amps] maximum motor current, PID will assumes this corresponds to max DAC value

	// Encoder
	uint8_t pinsENC_[6] = { 20, 17, 15, 29, 32, 30 }; // LS7366R chip select pins (MCB Rev 1.2)
	LS7366R ENC_; // Quadrature encoder interface
	
	// PID
	PID_f32 PID_; // PID controller, 32-bit float version
	int32_t countLast_ = 0;
	volatile int32_t countDesired_ = 0;
	int32_t	countError_ = 0;
	float effort_ = 0; // unsaturated, computed effort from controller in Amps
	//float32_t	kp_ = 0.0;	// proportional gain
	//float32_t	ki_ = 0.0;	// integral gain
	//float32_t	kd_ = 0.0;	// derivative gain
};

#endif
