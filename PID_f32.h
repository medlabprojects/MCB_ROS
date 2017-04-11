/*=========================================================================//

PID_f32 Class

This class implements the PID functions of the CMSIS DSP library 
for ARM microcontrollers.

All data types are 32-bit floats.

Trevor Bruns

Changelog-
3/14/2017: Initial Creation

//=========================================================================*/

#ifndef PID_f32_h
#define PID_f32_h

#include <arm_math.h>

class PID_f32
{
public:
	PID_f32();
	~PID_f32();

	void init(float32_t kp, float32_t ki, float32_t kd); // must be called before step()
	void init(); // defaults to kp = kd = ki = 0.0
	float32_t step(float32_t error); // steps PID controller and computes effort
	void reset(); // resets state buffer to zeros

	void setGains(float32_t  kp, float32_t  ki, float32_t  kd);
	void getGains(float32_t& kp, float32_t& ki, float32_t& kd);
	void setKp(float32_t kp);
	float32_t getKp(void);
	void setKi(float32_t ki);
	float32_t getKi(void);
	void setKd(float32_t kd);
	float32_t getKd(void);
	
private:
	arm_pid_instance_f32 PID_;
};

#endif