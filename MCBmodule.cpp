/*=========================================================================//

	Motor Class
	
	This class handles the control of an individual motor
	
	Designed for use with Teensy 3.1 and Motor Control Board (Rev 1.1)
	
	
	Trevor Bruns
	
	Changelog-
		2/10/2016: Initial Creation
		3/13/2017: Compiles and appears to run successfully
		
//=========================================================================*/

#include "MCBmodule.h"
#include <stdint.h>
#include <SPI.h>
#include "LS7366R.h"
#include "PID_f32.h"
#include <math.h>

MCBmodule::MCBmodule(uint8_t position)
	: position_(position)
	//, ENC_(getENCpin())	// setup LS7366R
	, ENC_(pinsENC_[position])
{
}

void MCBmodule::init(float kp, float ki, float kd)
{
	ENC_.init(); // set up encoder IC (LS7366R)
	PID_.init(kp, ki, kd); // set up PID controller
	setStatus(MODULE_ENABLE);
}

void MCBmodule::init(void)
{

	ENC_.init(); // set up encoder IC (LS7366R)

	PID_.init(); // set up PID controller; gains all set to 0.0

	setStatus(MODULE_ENABLE);

}

int16_t MCBmodule::step(void)
{
	int16_t tmp = 0;

	if (getStatus()){
		// read current motor position and compute error
		countError_ = countDesired_ - readCount();

		// step PID controller to compute effort in Amps
		effort_ = PID_.step(float(countError_));

		// enforce max current bounds and convert to int16 for DAC
		tmp = convertEffortToDAC(effort_);

	}

	return tmp;
}

float MCBmodule::getEffort(void)
{
	return effort_;
}

void MCBmodule::restartPID(void)
{
	PID_.reset();
}

int32_t MCBmodule::getError(void)
{
	return countError_;
}

int16_t MCBmodule::convertEffortToDAC(float effort)
{
	float effortTemp = effort;

	// check for saturation
	if (effort > maxAmps_) { effortTemp = maxAmps_; }
	else if (effort < -maxAmps_) { effortTemp = -maxAmps_; }

	// compute relative effort and scale to int16 range
	return (int16_t)(32767 * (effortTemp / maxAmps_));
}

void MCBmodule::setStatus(bool status)
{
	status_ = status;
}

bool MCBmodule::getStatus(void)
{
	return status_;
}

void MCBmodule::setMaxAmps(float maxAmps)
{
	maxAmps_ = fabsf(maxAmps); // just want the magnitude
}

float MCBmodule::getMaxAmps(void)
{
	return maxAmps_;
}

void MCBmodule::setCountDesired(int32_t countDesired)
{
	
	countDesired_ = countDesired;
}

int32_t MCBmodule::getCountDesired(void)
{
	return countDesired_;
}

int32_t MCBmodule::readCount(void)
{
	// read LS7366R
	countLast_ = ENC_.count();

	return countLast_;
}

int32_t MCBmodule::getCountLast(void)
{
	return countLast_;
}

void MCBmodule::setGains(float kp, float ki, float kd)
{
	PID_.setGains(kp, ki, kd);
}

void MCBmodule::setKp(float kp)
{
	PID_.setKp(kp);
}

void MCBmodule::setKi(float ki)
{
	PID_.setKi(ki);
}

void MCBmodule::setKd(float kd)
{
	PID_.setKd(kd);
}

float MCBmodule::getKp(void)
{
	return PID_.getKd();
}

float MCBmodule::getKi(void)
{
	return PID_.getKi();
}

float MCBmodule::getKd(void)
{
	return PID_.getKd();
}

uint8_t MCBmodule::getENCpin(void)
{	
	return pinsENC_[position_];
}

MCBmodule::~MCBmodule(void)
{
}
