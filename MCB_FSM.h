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
#include <std_msgs\Bool.h>
#include <std_msgs\Empty.h>
#include <std_msgs\UInt8.h>
#include <medlab_motor_control_board\EnableMotor.h>
#include <medlab_motor_control_board\McbEncoderCurrent.h>
#include <medlab_motor_control_board\McbEncoders.h>
#include <medlab_motor_control_board\McbGains.h>
#include <medlab_motor_control_board\McbStatus.h>

// Motor Control Board
void motorSelectLedCallback(void); // ISR for toggling LED of selected motor during manual control state
void modeSwitchCallback(void); // ISR for mode switch on motherboard
void limitSwitchCallback(void); // ISR for MCP23008 interrupt

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
void subEnableRosControlCallback(const std_msgs::Bool& msg); // enters or exits RosControl state
void subEnableMotorCallback(const medlab_motor_control_board::EnableMotor& msg); // enables or disables a single motor
void subEnableAllMotorsCallback(const std_msgs::Bool& msg); // enables or disables all motors
void subEncoderCommandCallback(const medlab_motor_control_board::McbEncoders& msg); // callback for subscriber subEncoderCommand
void subEncoderResetSingleCallback(const std_msgs::UInt8& msg); // resets a specific encoder (0-5) to zero
void subEncoderResetAllCallback(const std_msgs::Empty& msg); // resets all encoders to zero
void subGetStatusCallback(const std_msgs::Empty& msg);
void subSetGainsCallback(const medlab_motor_control_board::McbGains& msg);


//void srvEnableCallback(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); // callback for service srvEnableMCB
//void srvGetGainsCallback(const beginner_tutorials::srvGetGainsRequest &req, beginner_tutorials::srvGetGainsResponse &res); // callback for service srvGetGains
//void srvSetGainsCallback(const beginner_tutorials::srvSetGainsRequest &req, beginner_tutorials::srvSetGainsResponse &res); // callback for service srvSetGains
//void srvStatusMCBCallback(const beginner_tutorials::srvStatusMCBRequest &req, beginner_tutorials::srvStatusMCBResponse &res); // callback for service srvStatusMCB

// Manual Control
void timerManualControlCallback(void);
void runManualControl(void);

#endif // !MCB_FSM_H