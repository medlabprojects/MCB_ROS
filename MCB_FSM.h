/*=========================================================================//

MCB_FSM.h

Implements a finite state machine for the Motor Control Board

Designed for use with Teensy 3.1/3.2 and Motor Control Board (Rev 1.2)


Trevor Bruns

Changelog-
04/10/2017: Initial Creation

//=========================================================================*/

#ifndef MCB_FSM_H
#define MCB_FSM_H

#include "MCB.h"
#include "MCBmodule.h"
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h"
#include <beginner_tutorials/EncoderMessage.h>
#include <std_srvs\SetBool.h>

// Motor Control Board
void motorSelectLedCallback(void);
void CTRLswitchCallback(void);

// Finite State Machine
enum MCBstate { statePowerUp, stateLocalIdle, stateLocalControl, stateRosInit, stateRosIdle, stateRosControl };
MCBstate PowerUP(void);
MCBstate LocalIdle(void);
MCBstate LocalControl(void);
MCBstate RosInit(void);
MCBstate RosIdle(void);
MCBstate RosControl(void);
MCBstate stepStateMachine(MCBstate stateNext);

// PID Controller
void timerPidCallback(void);

// ROS
void timerRosCallback(void);
void srvEnableMCB(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res);

// Local Control
void timerLocalControlCallback(void);

#endif // !MCB_FSM_H