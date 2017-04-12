#include "MCB_FSM.h"
#include "MCB.h"
#include "MCBmodule.h"
#include "MCBpins.h"
#include <i2c_t3.h>
#include <SPI.h>
#include <ArduinoSTL.h>
#include "LS7366R.h"
#include "AD5761R.h"
#include "si5351.h"
#include "PID_f32.h"
#include <arm_math.h>
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h" 
#include <beginner_tutorials/EncoderMessage.h>
#include <std_srvs\SetBool.h>


MCBstate nextState;

void setup()
{
	// initialize MCB finite state machine
	nextState = stepStateMachine(statePowerUp);
}

//----------------------------------------------------------------//

void loop()
{
	nextState = stepStateMachine(nextState);
}