/*=========================================================================//

PID_f32 Class

This class implements the PID functions of the CMSIS DSP library
for ARM microcontrollers.

All data types are 32-bit floats.

Trevor Bruns

Changelog-
3/14/2017: Initial Creation

//=========================================================================*/

#include "PID_f32.h"
#include <arm_math.h>

PID_f32::PID_f32()
{
}

PID_f32::~PID_f32(){}

void PID_f32::init(float32_t kp, float32_t ki, float32_t kd)
{
	PID_.Kp = kp;
	PID_.Ki = ki;
	PID_.Kd = kd;
	arm_pid_init_f32(&PID_, 1);
}

void PID_f32::init()
{
	PID_.Kp = 0.0;
	PID_.Ki = 0.0;
	PID_.Kd = 0.0; 
	arm_pid_init_f32(&PID_, 1);
}

float32_t PID_f32::step(float32_t error)
{
	float32_t out = arm_pid_f32(&PID_, error);
	return out;
}

void PID_f32::reset()
{
	arm_pid_reset_f32(&PID_); 
}

void PID_f32::setGains(float32_t kp, float32_t ki, float32_t kd)
{
	PID_.Kp = kp;
	PID_.Ki = ki;
	PID_.Kd = kd;
	arm_pid_init_f32(&PID_, 1); // recomputes A0,A1,A2 using new gains and resets state buffer
}

void PID_f32::getGains(float32_t& kp, float32_t& ki, float32_t& kd)
{
	kp = getKp();
	ki = getKi();
	kd = getKd();
}

void PID_f32::setKp(float32_t kp)
{
	PID_.Kp = kp;
}

float32_t PID_f32::getKp(void)
{
	return PID_.Kp;
}

void PID_f32::setKi(float32_t ki)
{
	PID_.Ki = ki;
}

float32_t PID_f32::getKi(void)
{
	return PID_.Ki;
}

void PID_f32::setKd(float32_t kd)
{
	PID_.Kd = kd;
}

float32_t PID_f32::getKd(void)
{
	return PID_.Kd;
}