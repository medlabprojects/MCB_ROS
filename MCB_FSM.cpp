#include "MCB_FSM.h"
#include "MCB.h"
#include "MCBmodule.h"
#include "McbRosConfiguration.h"
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h"
#include <medlab_motor_control_board/EnableMotor.h>
#include <medlab_motor_control_board/McbEncoderCurrent.h>
#include <medlab_motor_control_board/McbEncoders.h>
#include <medlab_motor_control_board/McbGains.h>
#include <medlab_motor_control_board/McbStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <EEPROM.h>

/****************** GLOBALS *********************/

// Motor Control Board
MCB MotorBoard; // construct motor control board
int8_t currentMotorSelected = 0; // for manual control using Up/Down buttons
IntervalTimer timerMotorSelectLed;
McbRosConfiguration rosConfig;
MCBstate stateCurrent;
enum modeSwitchState { Manual, Ros };
volatile modeSwitchState modeState;
bool ROSenable = false; // ROS must set this true via 'enable_ros_control' topic to control motors

// Manual Control
IntervalTimer timerManualControl; // Button read timer interrupt
volatile bool timerManualControlFlag = false; // indicates timerManualControl has been called
float frequencyManualControl = 500.0; // [Hz]
uint32_t timeStepManualControl = uint32_t(1000000.0 / frequencyManualControl); // [us]
uint32_t countStepManualControl = 50; // [counts] step size for each up/down button press

// ROS
ros::NodeHandle_<WiznetHardware> nh;
String rosNameEncoderCurrent;
String rosNameEncoderCommand;
String rosNameLimitSwitchEvent;
String rosNameStatus;
String rosNameGetStatus;
String rosNameEncoderZeroSingle;
String rosNameEncoderZeroAll;
String rosNameEnableMotor;
String rosNameEnableAllMotors;
String rosNameSetGains;
String rosNameEnableRosControl;
    
medlab_motor_control_board::McbEncoderCurrent msgEncoderCurrent; // stores most recent encoder counts to be sent via publisher
medlab_motor_control_board::McbStatus msgStatus; // stores MCB status message
medlab_motor_control_board::EnableMotor msgLimitSwitchEvent; // stores message that is sent whenever a limit switch is triggered
ros::Publisher pubEncoderCurrent("tmp", &msgEncoderCurrent); // publishes current motor positions
ros::Publisher pubStatus("tmp", &msgStatus); // publishes MCB status
ros::Publisher pubLimitSwitchEvent("tmp", &msgLimitSwitchEvent); // publishes each time a limit switch is triggered
ros::Subscriber<medlab_motor_control_board::EnableMotor> subEnableMotor("tmp", &subEnableMotorCallback); // enables or disables power to a specific motor
ros::Subscriber<std_msgs::Bool>                          subEnableAllMotors("tmp", &subEnableAllMotorsCallback); // enables or disables all motors
ros::Subscriber<medlab_motor_control_board::McbGains>    subSetGains("tmp", &subSetGainsCallback); // sets gains for a specific motor
ros::Subscriber<medlab_motor_control_board::McbEncoders> subEncoderCommand("tmp", &subEncoderCommandCallback); // receives motor commands
ros::Subscriber<std_msgs::UInt8>                         subEncoderZeroSingle("tmp", &subEncoderZeroSingleCallback); // resets a single encoder to zero
ros::Subscriber<std_msgs::Empty>                         subEncoderZeroAll("tmp", &subEncoderZeroAllCallback); // resets all encoders to zero
ros::Subscriber<std_msgs::Bool>                          subEnableRosControl("tmp", &subEnableRosControlCallback); // used to move between RosIdle and RosControl states
ros::Subscriber<std_msgs::Empty>                         subGetStatus("tmp", &subGetStatusCallback); // tells MCB to publish pubStatus

IntervalTimer timerRos; // ROS timer interrupt
volatile bool timerRosFlag = false; // indicates timerRos has been called
float frequencyRos = 500.0; // [Hz]
uint32_t timeStepRos = uint32_t(1000000.0 / frequencyRos); // [us]
uint8_t publishInterval = 4; // publish every x times timerRos is called

// PID Controller
IntervalTimer timerPid; // PID controller timer interrupt
volatile bool timerPidFlag = false; // indicates timerPid has been called
//int32_t countDesired[6]; // does this need to be volatile?
float frequencyPid = 1000.0; // [Hz]
uint32_t timeStepPid = uint32_t(1000000.0 / frequencyPid); // [us]
//float kp = 0.0004, ki = 0.000002, kd = 0.01; // work ok for 1 kHz, EC13 brushless motor (gearhead only/no drivetrain connected!)
float kp = 0.008, ki = 0.00002, kd = 0.17; // work ok for 1 kHz, EC13 brushless motor (endonasal module; translation; no tubes)
//float kp = 0.0010, ki = 0.000003, kd = 0.035; // work ok for 1 kHz, RE25 brushed motor
// float kp = 0.0002, ki = 0.000001, kd = 0.01; // work ok for 2 kHz


MCBstate stepStateMachine(MCBstate stateNext) 
{
	switch (stateNext)
	{
	case statePowerUp:
		return PowerUP();

	case stateManualIdle:
		return ManualIdle();

	case stateManualControl:
		return ManualControl();

    case stateRosInit:
		return RosInit();

	case stateRosIdle:
		return RosIdle();

	case stateRosControl:
		return RosControl();

	default:
		Serial.println(F("Error: Unrecognized State"));
        while (1) {
            MotorBoard.setLEDG(HIGH);
            delay(500);
            MotorBoard.setLEDG(LOW);
            delay(500);
        }
	}
}

MCBstate PowerUP(void)
{
	stateCurrent = statePowerUp;

	// start serial port
	Serial.begin(115200);
    Serial.setTimeout(100);

	// initialize motor control board
    Serial.println("\n********************************");
    Serial.println("Initializing Motor Control Board");
    Serial.print("Firmware Version ");
    Serial.println(MCB_VERSION);
    Serial.println("********************************\n");
	
    if (MotorBoard.init() == -1) {
        Serial.println(F("\nERROR: INCORRECT MODULE CONFIGURATION"));
        Serial.println(F("\nPower off and ensure there are no gaps between modules"));
        while (1) {
            MotorBoard.setLEDG(HIGH);
            delay(500);
            MotorBoard.setLEDG(LOW);
            delay(500);
        }
    }
    Serial.print(MotorBoard.numModules());
    Serial.println(F(" motor modules detected and configured"));

    delay(10);
    
    // initialize limit switch states
    MotorBoard.updateAmpStates();

    // ensure amps are disabled
    MotorBoard.disableAllAmps();

	// create pin change interrupt for mode switch
	attachInterrupt(MotorBoard.pins.modeSelect, modeSwitchCallback, CHANGE);
	modeSwitchCallback(); // run once to initialize modeState

    // create pin change interrupt for amplifier control/limit switch monitoring (active low)
    attachInterrupt(MotorBoard.pins.i2cInt, ampEnableISR, FALLING);
    
	// advance based on mode switch position 
	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosInit;

    default: // shouldn't ever reach here
        return stateManualIdle;
	}
}

MCBstate RosInit(void)
{
	stateCurrent = stateRosInit;

    Serial.println(F("\n\n******************"));
    Serial.println(F("ROS Initialization"));
    Serial.println(F("******************"));

    ROSenable = false;

    // read ROS configuration parameters from EEPROM
    rosConfig.getSettingsFromEeprom();

    // set to defaults if none found
    if (!rosConfig.wasSaved()) {
        rosConfig.setDefaults();
    }

    // update ROS node handle with the new config parameters
    IPAddress mcbIp(192, 168, 0, rosConfig.getIP());
    nh.getHardware()->setIP(mcbIp);
    uint8_t mcbMac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, rosConfig.getMac() };
    nh.getHardware()->setMAC(mcbMac);

    // create topic names
    String rosNamespace = rosConfig.getNamespace();
    rosNameEncoderCurrent = rosNamespace + "/encoder_current";
    rosNameEncoderCommand = rosNamespace + "/encoder_command";
    rosNameLimitSwitchEvent = rosNamespace + "/limit_switch_event";
    rosNameStatus = rosNamespace + "/status";
    rosNameGetStatus = rosNamespace + "/get_status";
    rosNameEncoderZeroSingle = rosNamespace + "/encoder_zero_single";
    rosNameEncoderZeroAll = rosNamespace + "/encoder_zero_all";
    rosNameEnableMotor = rosNamespace + "/enable_motor";
    rosNameEnableAllMotors = rosNamespace + "/enable_all_motors";
    rosNameSetGains = rosNamespace + "/set_gains";
    rosNameEnableRosControl = rosNamespace + "/enable_ros_control";

    // setup topics
    pubEncoderCurrent     = ros::Publisher(rosNameEncoderCurrent.c_str(), &msgEncoderCurrent);
    subEncoderCommand     = ros::Subscriber<medlab_motor_control_board::McbEncoders>(rosNameEncoderCommand.c_str(), &subEncoderCommandCallback);
    pubLimitSwitchEvent   = ros::Publisher(rosNameLimitSwitchEvent.c_str(), &msgLimitSwitchEvent);
    pubStatus             = ros::Publisher(rosNameStatus.c_str(), &msgStatus);
    subGetStatus          = ros::Subscriber<std_msgs::Empty>(rosNameGetStatus.c_str(), &subGetStatusCallback);
    subEncoderZeroAll     = ros::Subscriber<std_msgs::Empty>(rosNameEncoderZeroAll.c_str(), &subEncoderZeroAllCallback);
    subEncoderZeroSingle  = ros::Subscriber<std_msgs::UInt8>(rosNameEncoderZeroSingle.c_str(), &subEncoderZeroSingleCallback);
    subEnableMotor        = ros::Subscriber<medlab_motor_control_board::EnableMotor>(rosNameEnableMotor.c_str(), &subEnableMotorCallback);
    subEnableAllMotors    = ros::Subscriber<std_msgs::Bool>(rosNameEnableAllMotors.c_str(), &subEnableAllMotorsCallback);
    subSetGains           = ros::Subscriber<medlab_motor_control_board::McbGains>(rosNameSetGains.c_str(), &subSetGainsCallback);
    subEnableRosControl   = ros::Subscriber<std_msgs::Bool>(rosNameEnableRosControl.c_str(), &subEnableRosControlCallback);

	// set up Wiznet and connect to ROS server
	Serial.print(F("Configuring Ethernet Connection ... "));
	nh.initNode();

	// repeatedly attempt to setup the hardware, loop on fail, stop on success
	while ((nh.getHardware()->error() < 0) && (modeState == Ros)) {
		Serial.print(F("WIZnet error = "));
		Serial.println(nh.getHardware()->error());

		nh.initNode();
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}

	Serial.println(F("Success!"));

	// initialize ROS
	Serial.print(F("Connecting to ROS Network ... "));
	nh.advertise(pubEncoderCurrent);
    nh.advertise(pubStatus);
    nh.advertise(pubLimitSwitchEvent);
	nh.subscribe(subEncoderCommand);
    nh.subscribe(subEncoderZeroSingle);
    nh.subscribe(subEncoderZeroAll);
    nh.subscribe(subEnableMotor);
    nh.subscribe(subEnableAllMotors);
    nh.subscribe(subSetGains);
    nh.subscribe(subEnableRosControl);
    nh.subscribe(subGetStatus);
	

    // wait until connection established or mode switched to 'Manual'
	while (!nh.connected() && (modeState == Ros)) {
        noInterrupts();
		nh.spinOnce();
        interrupts();
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}

	Serial.println(F("Success!"));
	
	// initialize motors
	Serial.print(F("Initializing Motors ... "));
	for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
		MotorBoard.setGains(ii, kp, ki, kd);
        MotorBoard.setCountDesired(ii, MotorBoard.readCountCurrent(ii));
        MotorBoard.setPolarity(ii, 1);
	}
	Serial.println(F("done"));

	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosIdle;

    default: // shouldn't ever reach here
        return stateRosInit;
	}
}

MCBstate RosIdle(void)
{
	stateCurrent = stateRosIdle;

    Serial.println(F("\n\n*************************"));
	Serial.println(F("\nEntering ROS Idle state"));
    Serial.println(F("*************************"));
    Serial.println(F("\nwaiting for enable signal via enable_controller"));

    // ensure amps are off
    MotorBoard.disableAllAmps();
    MotorBoard.globalEnable(false);
    MotorBoard.updateAmpStates();

    // start ROS update timer
    timerRos.begin(timerRosCallback, timeStepRos);
    uint32_t rosLoopCount = 0;

	// wait for ROS enable command via service call
	while (!ROSenable && nh.connected() && (modeState == Ros)) {
        noInterrupts(); // prevent interrupts during SPI communication

        if (timerRosFlag) {
            if (rosLoopCount % publishInterval == 0) {
                // assemble encoder message to send out
                for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
                    if (MotorBoard.isAmpEnabled(ii)) {
                        // stepPid() updates/reads count only if amp is enabled
                        msgEncoderCurrent.measured[ii] = MotorBoard.getCountLast(ii);
                    }
                    else {
                        // must manually read current count when disabled
                        msgEncoderCurrent.measured[ii] = MotorBoard.readCountCurrent(ii);
                    }
                    msgEncoderCurrent.desired[ii] = MotorBoard.getCountDesired(ii);
                }
                // queue messages into their publishers
                pubEncoderCurrent.publish(&msgEncoderCurrent);
            }
            rosLoopCount++;

            // process pending ROS communications
            nh.spinOnce();

            timerRosFlag = false;
        }

        interrupts();
	}

	if (!nh.connected()) {
		return stateRosInit;
	}

	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosControl;

    default: // shouldn't ever reach here
        return stateRosInit;
	}
}

MCBstate RosControl(void)
{
	stateCurrent = stateRosControl;

    Serial.println(F("\n\n**************************"));
    Serial.println(F("Entering ROS Control state"));
    Serial.println(F("**************************"));

	// set desired motor position to current position (prevents unexpected movement)
	for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
		MotorBoard.setCountDesired(ii, MotorBoard.readCountCurrent(ii));
	}

	// start PID timer
	timerPid.begin(timerPidCallback, timeStepPid);

	// start ROS update timer
	timerRos.begin(timerRosCallback, timeStepRos);

    // set global enable
    MotorBoard.globalEnable(true);
    MotorBoard.updateAmpStates();

    //noInterrupts();

    uint32_t rosLoopCount = 0;

	// loop until disconnected OR ROS 'disable' command OR mode switched to 'Manual'
	while (ROSenable && nh.connected() && (modeState == Ros)) 
    { 
    // NOTE: this while loop must be able to run at least twice as fast as fastest InterruptTimer (usually timerPid)
        
        if (MotorBoard.ampEnableFlag()) {
            // update current states of limit switches and ampEnable pins
            uint8_t deviceTriggered = MotorBoard.updateAmpStates();

            // process limit switch event
            if (MotorBoard.limitSwitchTriggeredFlag()) {
                // disable motor
                MotorBoard.disableAmp(deviceTriggered);

                // publish limit switch event
                msgLimitSwitchEvent.motor = deviceTriggered;
                msgLimitSwitchEvent.enable = MotorBoard.limitSwitchState(deviceTriggered);
                pubLimitSwitchEvent.publish(&msgLimitSwitchEvent);
            }
            else if (deviceTriggered == 6) { // e-stop
                // publish limit switch event
                msgLimitSwitchEvent.motor = deviceTriggered;
                msgLimitSwitchEvent.enable = MotorBoard.eStopState();
                pubLimitSwitchEvent.publish(&msgLimitSwitchEvent);

                // exit ROS control state
                ROSenable = false;
            }

            // can reset now that we've processed events
            MotorBoard.resetLimitSwitchTriggered();
        }

        noInterrupts(); // prevent interrupts during functions using SPI      

        if (timerPidFlag) {
            // read encoders, compute PID effort, update DACs
            MotorBoard.stepPid();
            
            timerPidFlag = false;
        }

        if (timerRosFlag) {
            if (rosLoopCount % publishInterval == 0) {
                // assemble encoder message to send out
                for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
                    if (MotorBoard.isAmpEnabled(ii)) {
                        // stepPid() updates/reads count only if amp is enabled
                        msgEncoderCurrent.measured[ii] = MotorBoard.getCountLast(ii);
                    }
                    else {
                        // must manually read current count when disabled
                        msgEncoderCurrent.measured[ii] = MotorBoard.readCountCurrent(ii);
                    }
                    msgEncoderCurrent.desired[ii] = MotorBoard.getCountDesired(ii);
                }
                // queue messages into their publishers
                pubEncoderCurrent.publish(&msgEncoderCurrent);
            }
            rosLoopCount++;

            // process pending ROS communications
            nh.spinOnce();

            timerRosFlag = false;
        }

        interrupts(); // process any interrupts here
	}

	// power off motors, disable PID controller, and stop ROS timer
	MotorBoard.disableAllAmps();
	timerPid.end();
	timerRos.end();
	ROSenable = false;
    interrupts(); // now safe to re-enable since timer interrupts are stopped

	if (!nh.connected()) {
		nh.getHardware()->error() = WiznetHardware::ERROR_CONNECT_FAIL;
		Serial.println(F("ROS connection lost"));
		return stateRosInit;
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}
	else { // 'stop' command must have been received
		return stateRosIdle; 
	}
}

MCBstate ManualIdle(void)
{
    stateCurrent = stateManualIdle;

    Serial.println(F("\n\n**************************"));
    Serial.println(F("Entering Manual Idle State"));
    Serial.println(F("**************************"));

    // ensure amps are off and controller is not running
    MotorBoard.disableAllAmps();
    MotorBoard.globalEnable(false);
    timerPid.end();

    uint32_t holdTime = 2000; // [ms] how long buttons must be held before function returns
    uint32_t timeButtonsPressed = 0; // [ms] how long buttons have been held
    uint32_t timeStart = 0;
    MotorBoard.setLEDG(false); // turn off green LEDs
    bool configFinished = false;

    // wait until gains have been set via serial OR user overrides by holding buttons
    Serial.println(F("\nROS Configuration"));
    rosConfig.printMenu();
    rosConfig.printWaitCommand();
    while ((modeState == Manual) && !configFinished) {
        // check for serial commands
        if (!rosConfig.runOnce()) {
            // user has selected an exit command
            configFinished = true;
        }

        // check buttons
        if (timeButtonsPressed < holdTime)
        {
            MotorBoard.readButtons();
            if (MotorBoard.isEverythingPressed()) {
                // if just pressed
                if (timeButtonsPressed == 0)
                {
                    timeStart = millis();
                    delayMicroseconds(1000); // ensure next millis() call will be different
                }

                // light LEDs in sequence for user feedback
                if (timeButtonsPressed < (holdTime / 7)) {
                    MotorBoard.setLEDG(0, HIGH);
                }
                else if (timeButtonsPressed < (2 * holdTime / 7)) {
                    MotorBoard.setLEDG(1, HIGH);
                }
                else if (timeButtonsPressed < (3 * holdTime / 7)) {
                    MotorBoard.setLEDG(2, HIGH);
                }
                else if (timeButtonsPressed < (4 * holdTime / 7)) {
                    MotorBoard.setLEDG(3, HIGH);
                }
                else if (timeButtonsPressed < (5 * holdTime / 7)) {
                    MotorBoard.setLEDG(4, HIGH);
                }
                else if (timeButtonsPressed < (6 * holdTime / 7)) {
                    MotorBoard.setLEDG(5, HIGH);
                }
                else {
                    MotorBoard.setLEDG(LOW);
                    delayMicroseconds(50000);
                    MotorBoard.setLEDG(HIGH);
                    delayMicroseconds(50000);
                }

                timeButtonsPressed = millis() - timeStart;
            }
            else {
                timeButtonsPressed = 0;
                MotorBoard.setLEDG(LOW);
            }
        }
        else {
            // user override -> use default gains and current limits
            configFinished = true;
            delay(1000);
        }
    }

    // initialize motors
    Serial.print(F("Initializing Motors ... "));
    for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
        MotorBoard.setGains(ii, kp, ki, kd);
    }
    Serial.println(F("done"));

    // advance based on mode switch position
    switch (modeState) {
    case Manual:
        return stateManualControl;

    case Ros:
        return stateRosInit;

    default: // shouldn't ever reach here
        return stateManualIdle;
    }
}

MCBstate ManualControl(void)
{
    stateCurrent = stateManualControl;

    Serial.println(F("\n\n*****************************"));
    Serial.println(F("Entering Manual Control State"));
    Serial.println(F("*****************************\n"));

    // set desired motor position to current position (prevents unexpected movement)
    for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
        MotorBoard.setCountDesired(ii, MotorBoard.readCountCurrent(ii));
    }

    // start PID controllers
    timerPid.begin(timerPidCallback, timeStepPid);

    // start manual control timer
    timerManualControl.begin(timerManualControlCallback, timeStepManualControl);

    // flash led of currently selected motor
    timerMotorSelectLed.begin(motorSelectLedCallback, 350000);

    // set global enable for amps
    MotorBoard.globalEnable(true);

    // power on first motor
    MotorBoard.enableAmp(currentMotorSelected);

    MotorBoard.setLEDG(LOW);

    // keep running until mode switch changed to Ros OR serial command detected
    while (modeState == Manual)
    {
        // NOTE: this while loop must be able to run at least twice as fast as the fastest InterruptTimer (usually timerPid)

        if (MotorBoard.ampEnableFlag()) {
            uint8_t device = MotorBoard.updateAmpStates(); // module triggered indicated by green LED

            if (device == 7) {
                Serial.println(F("\nE-Stop Engaged! \nExiting Manual Control State"));

                // stop timers and disable amps
                timerManualControl.end();
                timerMotorSelectLed.end();
                timerPid.end();
                MotorBoard.disableAllAmps();

                return stateManualIdle; // leave control state
            }
            else if (MotorBoard.limitSwitchTriggeredFlag()) {
                // limit switch was triggered
                Serial.print(F("limit switch "));
                Serial.print(device);
                Serial.print(F(" triggered (switch is "));
                if (MotorBoard.limitSwitchState(device)) {
                    Serial.println(F("closed)"));
                }
                else {
                    Serial.println(F("open)"));
                }

                // disable this motor
                MotorBoard.disableAmp(device);

                // reset limit switch flag
                MotorBoard.resetLimitSwitchTriggered();
            }
        }
        
        noInterrupts(); // prevents interruption during critical functions

        if (timerPidFlag) {
            // read encoders, compute PID effort, update DACs
            MotorBoard.stepPid();

            timerPidFlag = false;
        }

        // check serial for commands
        //if (Serial.available())
        //{
        //    char cmd = Serial.read();
        //    if ((cmd == 'x') || (cmd == 'X'))  // 'stop' command
        //    {
        //        Serial.println("'Stop' command received");
        //        // stop checking buttons
        //        timerManualControl.end();
        //        timerMotorSelectLed.end();

        //        // power off motors and disable PID controller
        //        MotorBoard.disableAllAmps();
        //        timerPid.end();

        //        // turn off green LEDs
        //        MotorBoard.setLEDG(LOW);

        //        // return to Manual Idle state to process command
        //        return stateManualIdle;
        //    }
        //}


        // run manual control on a timer so a held button produces a constant velocity
        if (timerManualControlFlag) {
            runManualControl();
            timerManualControlFlag = false;
        }

        interrupts(); // now safe to process any interrupts
    }

    // stop checking buttons
    timerManualControl.end();
    timerMotorSelectLed.end();

    // power off motors and disable PID controller
    MotorBoard.disableAllAmps();
    timerPid.end();

    // turn off green LEDs
    MotorBoard.setLEDG(LOW);

    return stateRosInit; // while() only exits if mode switched to 'Ros'
}

//--------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------//

void ampEnableISR(void)
{
    MotorBoard.setAmpEnableFlag();
}


void modeSwitchCallback(void)
{
    // HIGH = ROS; LOW = Manual
	modeState = (digitalReadFast(MotorBoard.pins.modeSelect) ? Ros: Manual);
}

void timerPidCallback(void)
{
    timerPidFlag = true;
}

void timerRosCallback(void)
{
    timerRosFlag = true;
}

void subEnableRosControlCallback(const std_msgs::Bool & msg)
{
    if (msg.data) {
        if (!ROSenable) {
            if (stateCurrent == stateRosIdle) {
                ROSenable = true;
            }
        }
    }
    else { // 'stop' command
        ROSenable = false;
    }
}

void subEnableMotorCallback(const medlab_motor_control_board::EnableMotor & msg)
{
    // MUST be in ROS Control state before enabling (otherwise PID isn't running)
    if (stateCurrent != stateRosControl) {
        return;
    }

    // check that requested motor has been configured
    if (!MotorBoard.isModuleConfigured(msg.motor)) {
        return;
    }

    if (msg.enable) 
    {
        // enable motor
        MotorBoard.enableAmp(msg.motor);
    }
    else
    {
        // disable motor
        MotorBoard.disableAmp(msg.motor);
    }
}

void subEnableAllMotorsCallback(const std_msgs::Bool & msg)
{
    // MUST be in ROS Control state before enabling (otherwise globalEnable is still false)
    if (stateCurrent != stateRosControl) {
        return;
    }

    if (msg.data)
    {
        // enable all motors
        MotorBoard.enableAllAmps();
    }
    else
    {
        // disable all motors
        MotorBoard.disableAllAmps();
    }
}

void subEncoderCommandCallback(const medlab_motor_control_board::McbEncoders& msg)
{
	// set desired motor positions with values received over ROS
	for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
		MotorBoard.setCountDesired(ii, msg.count[ii]);
	}
}

void subEncoderZeroSingleCallback(const std_msgs::UInt8 & msg)
{
    // ensure request is valid
    if (msg.data <= MotorBoard.numModules()) {
        // disable amp and reset the encoder count to zero for the desired motor
        MotorBoard.resetCount(msg.data);
    }
}

void subEncoderZeroAllCallback(const std_msgs::Empty & msg)
{
    // disables all amps and resets encoder counts to zero
    MotorBoard.resetCounts();
}

void subGetStatusCallback(const std_msgs::Empty & msg)
{
    // assemble status message
    msgStatus.number_modules = MotorBoard.numModules();
    msgStatus.current_state = MCBstateToString(stateCurrent);
    uint32_t tmpIP = (uint32_t)nh.getHardware()->wiznet_ip;
    memcpy(msgStatus.ip, &tmpIP, 4);
    memcpy(msgStatus.mac, nh.getHardware()->wiznet_mac, 6);
    for (int ii = 0; ii < 6; ii++) {
        msgStatus.count_commanded[ii] = MotorBoard.getCountDesired(ii);
        msgStatus.count_current[ii] = MotorBoard.getCountLast(ii);
        msgStatus.control_effort[ii] = MotorBoard.getEffort(ii);
        msgStatus.motor_enabled[ii] = MotorBoard.isAmpEnabled(ii);
        msgStatus.limit_switch[ii] = MotorBoard.limitSwitchState(ii);
        FloatVec gains = MotorBoard.getGains(ii);
        msgStatus.p[ii] = gains.at(0);
        msgStatus.i[ii] = gains.at(1);
        msgStatus.d[ii] = gains.at(2);
    }
    
    // publish status
    pubStatus.publish(&msgStatus);
}

void subSetGainsCallback(const medlab_motor_control_board::McbGains & msg)
{
    // ensure a module exists at location requested
    if (MotorBoard.isModuleConfigured(msg.motor)) {
        MotorBoard.setGains(msg.motor, msg.p, msg.i, msg.d);
    }
}

void motorSelectLedCallback(void)
{
    MotorBoard.toggleLEDG(currentMotorSelected);
}

void timerManualControlCallback(void)
{
    timerManualControlFlag = true;
}

void runManualControl(void)
{
    // check buttons
	MotorBoard.readButtons();

	if (MotorBoard.isMenuPressed()) {
		MotorBoard.disableAllAmps(); // stop motors during user selection
		for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
			MotorBoard.setLEDG(ii, false);
		}
		MotorBoard.setLEDG(currentMotorSelected, true);

		while (MotorBoard.isMenuPressed()) {
			if (MotorBoard.isUpPressed()) {
				MotorBoard.setLEDG(currentMotorSelected, false);
				currentMotorSelected++;
				if (currentMotorSelected > (MotorBoard.numModules()-1)) {
					currentMotorSelected = 0; }
				MotorBoard.setLEDG(currentMotorSelected, true);
			}
			else if (MotorBoard.isDownPressed()) {
				MotorBoard.setLEDG(currentMotorSelected, false);
				currentMotorSelected--;
				if (currentMotorSelected < 0) { 
					currentMotorSelected = (MotorBoard.numModules()-1); }
				MotorBoard.setLEDG(currentMotorSelected, true);
			}

            MotorBoard.setCountDesired(currentMotorSelected, MotorBoard.getCountLast(currentMotorSelected)); // ensure we drive relative to current position

			delayMicroseconds(400000); // wait for human's slow reaction time
			MotorBoard.readButtons();
		}
		MotorBoard.globalEnable(true);  
    MotorBoard.enableAmp(currentMotorSelected);
	}
	else if (MotorBoard.isUpPressed()) {
        MotorBoard.setCountDesired(currentMotorSelected, MotorBoard.getCountDesired(currentMotorSelected) + countStepManualControl);
    }
	else if (MotorBoard.isDownPressed()) {
        MotorBoard.setCountDesired(currentMotorSelected, MotorBoard.getCountDesired(currentMotorSelected) - countStepManualControl);
	}
}

const char* MCBstateToString(MCBstate currentState) {
    const char* stateName;

    switch (currentState)
    {
    case statePowerUp:
        stateName = "Power Up";
        break;
    case stateManualIdle:
        stateName = "Manual Idle";
        break;
    case stateManualControl:
        stateName = "Manual Control";
        break;
    case stateRosInit:
        stateName = "ROS Init";
        break;
    case stateRosIdle:
        stateName = "ROS Idle";
        break;
    case stateRosControl:
        stateName = "ROS Control";
        break;
    default:
        stateName = "Error determining state";
        break;
    }

    return stateName;
}
