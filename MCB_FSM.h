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
#include <beginner_tutorials/msgEncoderDesired.h>
#include <beginner_tutorials\srvGetGains.h>
#include <beginner_tutorials\srvSetGains.h>
#include <beginner_tutorials\srvStatusMCB.h>
#include <std_srvs\SetBool.h>

// Motor Control Board
void motorSelectLedCallback(void);
void modeSwitchCallback(void);

// Finite State Machine
enum MCBstate { statePowerUp, stateManualIdle, stateManualControl, stateRosInit, stateRosIdle, stateRosControl };
char* MCBstateToString(MCBstate currentState);
MCBstate PowerUP(void);
MCBstate ManualIdle(void);
MCBstate ManualControl(void);
MCBstate RosInit(void);
MCBstate RosIdle(void);
MCBstate RosControl(void);
MCBstate stepStateMachine(MCBstate stateNext);

// PID Controller
void timerPidCallback(void);

// ROS
void timerRosCallback(void);
void subEncoderCommandCallback(const beginner_tutorials::msgEncoderDesired & encCommands); // callback for subscriber subEncoderCommand
void srvEnableCallback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); // callback for service srvEnableMCB
void srvGetGainsCallback(const beginner_tutorials::srvGetGainsRequest &req, beginner_tutorials::srvGetGainsResponse &res); // callback for service srvGetGains
void srvSetGainsCallback(const beginner_tutorials::srvSetGainsRequest &req, beginner_tutorials::srvSetGainsResponse &res); // callback for service srvSetGains
void srvStatusMCBCallback(const beginner_tutorials::srvStatusMCBRequest &req, beginner_tutorials::srvStatusMCBResponse &res); // callback for service srvStatusMCB

// Manual Control
void timerManualControlCallback(void);
void runManualControl(void);

#endif // !MCB_FSM_H