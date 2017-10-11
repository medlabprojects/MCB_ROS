#include "MCB_FSM.h"
#include "MCB.h"
#include "MCBmodule.h"
#include "McbRosConfiguration.h"
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h"
#include <medlab_motor_control_board\EnableMotor.h>
#include <medlab_motor_control_board\McbEncoderCurrent.h>
#include <medlab_motor_control_board\McbEncoders.h>
#include <medlab_motor_control_board\McbGains.h>
#include <medlab_motor_control_board\McbStatus.h>
#include <std_msgs\Bool.h>
#include <std_msgs\Empty.h>
#include <std_msgs\UInt8.h>
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
uint32_t countStepManualControl = 200; // [counts] step size for each up/down button press

// Limit switch and amplifier power control
volatile bool limitSwitchFlag = false; // indicates ampCtrlIntCallback was triggered

// ROS
ros::NodeHandle_<WiznetHardware> nh;
String rosNamespace = "configure_me_over_serial";
String rosNameEncoderCurrent = "/encoder_current";
medlab_motor_control_board::McbEncoderCurrent msgEncoderCurrent; // stores most recent encoder counts to be sent via publisher
medlab_motor_control_board::McbStatus msgStatus; // stores MCB status message
ros::Publisher pubEncoderCurrent("tempname", &msgEncoderCurrent); // publishes current motor positions
ros::Publisher pubStatus("tempname", &msgStatus); // publishes MCB status
ros::Subscriber<medlab_motor_control_board::EnableMotor> subEnableMotor("tempname", &subEnableMotorCallback); // enables or disables power to a specific motor
ros::Subscriber<std_msgs::Bool>                          subEnableAllMotors("tempname", &subEnableAllMotorsCallback); // enables or disables all motors
ros::Subscriber<medlab_motor_control_board::McbGains>    subSetGains("tempname", &subSetGainsCallback); // sets gains for a specific motor
ros::Subscriber<medlab_motor_control_board::McbEncoders> subEncoderCommand("tempname", &subEncoderCommandCallback); // receives motor commands
ros::Subscriber<std_msgs::UInt8>                         subEncoderResetSingle("tempname", &subEncoderResetSingleCallback); // resets a single encoder to zero
ros::Subscriber<std_msgs::Empty>                         subEncoderResetAll("tempname", &subEncoderResetAllCallback); // resets all encoders to zero
ros::Subscriber<std_msgs::Bool>                          subEnableRosControl("tempname", &subEnableRosControlCallback); // used to move between RosIdle and RosControl states
ros::Subscriber<std_msgs::Empty>                         subGetStatus("tempname", &subGetStatusCallback); // tells MCB to publish pubStatus

IntervalTimer timerRos; // ROS timer interrupt
volatile bool timerRosFlag = false; // indicates timerRos has been called
float frequencyRos = 500.0; // [Hz]
uint32_t timeStepRos = uint32_t(1000000.0 / frequencyRos); // [us]
uint8_t publishInterval = 8; // publish every x times timerRos is called

// PID Controller
IntervalTimer timerPid; // PID controller timer interrupt
volatile bool timerPidFlag = false; // indicates timerPid has been called
int32_t countDesired[6]; // does this need to be volatile?
float frequencyPid = 1000.0; // [Hz]
uint32_t timeStepPid = uint32_t(1000000.0 / frequencyPid); // [us]
float kp = 0.0010, ki = 0.000003, kd = 0.035; // work ok for 1 kHz, RE25 brushed motor
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
		Serial.println("Error: Unrecognized State");
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
	Serial.println("\nInitializing Motor Control Board ... ");
    if (MotorBoard.init() == -1) {
        Serial.println("\nERROR: INCORRECT MODULE CONFIGURATION");
        Serial.println("\nPower off and ensure there are no gaps between modules");
        while (1) {
            MotorBoard.setLEDG(HIGH);
            delay(500);
            MotorBoard.setLEDG(LOW);
            delay(500);
        }
    }
    Serial.print(MotorBoard.numModules());
    Serial.println(" motor modules detected and configured");

	// create pin change interrupt for mode switch
	attachInterrupt(MotorBoard.pins.modeSelect, modeSwitchCallback, CHANGE);
	modeSwitchCallback(); // run once to initialize modeState

    // create pin change interrupt for amplifier control/limit switch monitoring
    attachInterrupt(MotorBoard.pins.i2cInt, limitSwitchCallback, CHANGE);
    
	// advance based on mode switch position 
	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosInit;
	}
}

MCBstate RosInit()
{
	stateCurrent = stateRosInit;

    ROSenable = false;

    // set up topics
    rosNameEncoderCurrent = rosNamespace + rosNameEncoderCurrent;
    //pubEncoderCurrent = ros::Publisher(rosNameEncoderCurrent.c_str(), &msgEncoderCurrent);
    pubEncoderCurrent     = ros::Publisher("encoder_current", &msgEncoderCurrent);
    pubStatus             = ros::Publisher("status", &msgStatus);
    subEncoderCommand     = ros::Subscriber<medlab_motor_control_board::McbEncoders>("encoder_command", &subEncoderCommandCallback);
    subEncoderResetSingle = ros::Subscriber<std_msgs::UInt8>("encoder_reset_single", &subEncoderResetSingleCallback);
    subEncoderResetAll    = ros::Subscriber<std_msgs::Empty>("encoder_reset_all", &subEncoderResetAllCallback);
    subEnableMotor        = ros::Subscriber<medlab_motor_control_board::EnableMotor>("enable_motor", &subEnableMotorCallback);
    subEnableAllMotors    = ros::Subscriber<std_msgs::Bool>("enable_all_motors", &subEnableAllMotorsCallback);
    subSetGains           = ros::Subscriber<medlab_motor_control_board::McbGains>("set_gains", &subSetGainsCallback);
    subEnableRosControl   = ros::Subscriber<std_msgs::Bool>("enable_ros_control", &subEnableRosControlCallback);
    subGetStatus          = ros::Subscriber<std_msgs::Empty>("get_status", &subGetStatusCallback);


	// set up Wiznet and connect to ROS server
	Serial.print("Setting up ethernet connection ... ");
	nh.initNode();

	// repeatedly attempt to setup the hardware, loop on fail, stop on success
	while ((nh.getHardware()->error() < 0) && (modeState == Ros)) {
		Serial.print("WIZnet eror = ");
		Serial.println(nh.getHardware()->error());

		nh.initNode();
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}

	Serial.println("Success!");

	// initialize ROS
	Serial.print("Connecting to ROS Network ... ");
	nh.advertise(pubEncoderCurrent);
    nh.advertise(pubStatus);
	nh.subscribe(subEncoderCommand);
    nh.subscribe(subEncoderResetSingle);
    nh.subscribe(subEncoderResetAll);
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

	Serial.println("Success!");
	
	// initialize motors
	Serial.print("Initializing Motors ... ");
	for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
		MotorBoard.setGains(ii, kp, ki, kd);
		MotorBoard.setCountDesired(ii, countDesired[ii]);
        MotorBoard.setPolarity(ii, 1);
	}
	Serial.println("done");

	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosIdle;
	}
}

MCBstate RosIdle()
{
	stateCurrent = stateRosIdle;

	Serial.println("\nEntering ROS Idle state");
	Serial.println("waiting for enable signal via enable_controller");

	// wait for ROS enable command via service call
	while (!ROSenable && nh.connected() && (modeState == Ros)) {
        noInterrupts();
		nh.spinOnce();
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
	}
}

MCBstate RosControl()
{
	stateCurrent = stateRosControl;

	Serial.println("\nEntering ROS Control state");

	// set desired motor position to current position (prevents unexpected movement)
	for (int ii = 0; ii < MotorBoard.numModules(); ii++)
	{
		countDesired[ii] = MotorBoard.getCountLast(ii);
		MotorBoard.setCountDesired(ii, countDesired[ii]);
	}

	// enable motor amps
	//MotorBoard.enableAllAmps();

	// start PID timer
	timerPid.begin(timerPidCallback, timeStepPid);

	// start ROS update timer
	timerRos.begin(timerRosCallback, timeStepRos);

    noInterrupts();
    uint32_t rosLoopCount = 0;

	// loop until disconnected OR ROS 'disable' command OR mode switched to 'Manual'
	while (ROSenable && nh.connected() && (modeState == Ros)) 
    { 
    // NOTE: this while loop must be able to run at least twice as fast as fastest InterruptTimer (usually timerPid)
        
        //MotorBoard.setLEDG(1, HIGH);

        interrupts();
        // process any interrupt calls here
        noInterrupts();

        //MotorBoard.setLEDG(1, LOW);

        //if (limitSwitchFlag) {
        //    // determine which device triggered the interrupt
        //    int8_t device = MotorBoard.whichLimitSwitch();

        //    // check if triggering device is a limit switch
        //    if ((device >= 0) && (device <= 5)) {
        //        // publish ROS message

        //        // zero out encoder to prevent movement when power is restored
        //        MotorBoard.setCountDesired(device, MotorBoard.getCountLast(device));

        //        // reset PID controller to prevent windup
        //        MotorBoard.restartPid(device);
        //    }
        //    // check if more than one device was triggered
        //    else if (device == -1) {
        //        Int8Vec devices = MotorBoard.whichLimitSwitches();

        //        for (int ii = 0; ii < devices.size(); ii++) {
        //            // publish ROS message

        //            // zero out encoder to prevent movement when power is restored
        //            MotorBoard.setCountDesired(devices.at(ii), MotorBoard.getCountLast(devices.at(ii)));

        //            // restart PID controller to prevent windup
        //            MotorBoard.restartPid(devices.at(ii));
        //        }
        //    }
        //    // check if it is the E-stop
        //    else if (device == 6){
        //        // publish ROS message

        //        for (int ii = 0; ii < 6; ii++) {
        //            // publish ROS message

        //            // zero out encoder to prevent movement when power is restored
        //            MotorBoard.setCountDesired(ii, MotorBoard.getCountLast(ii));

        //            // restart PID controller to prevent windup
        //            MotorBoard.restartPid(ii);
        //        }
        //    }
        //    
        //}

        if (timerPidFlag) {
            //MotorBoard.setLEDG(2, HIGH);

            // read encoders, compute PID effort, update DACs
            MotorBoard.stepPid();
            
            timerPidFlag = false;
            //MotorBoard.setLEDG(2, LOW);
        }

        if (timerRosFlag) {
            //MotorBoard.setLEDG(2, HIGH);
            
            if (rosLoopCount % publishInterval == 0) {
                // assemble encoder message to send out
                for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
                    msgEncoderCurrent.measured[ii] = MotorBoard.getCountLast(ii);
                    msgEncoderCurrent.desired[ii] = MotorBoard.getCountDesired(ii);
                }
                // queue messages into their publishers
                pubEncoderCurrent.publish(&msgEncoderCurrent);
            }
            rosLoopCount++;

            //MotorBoard.setLEDG(2, LOW);

            // process pending ROS communications
            nh.spinOnce();

            timerRosFlag = false;
        }
	}

	// power off motors, disable PID controller, and stop ROS timer
	MotorBoard.disableAllAmps();
	timerPid.end();
	timerRos.end();
	ROSenable = false;
    interrupts(); // now safe to re-enable since timer interrupts are stopped

	if (!nh.connected()) {
		nh.getHardware()->error() = WiznetHardware::ERROR_CONNECT_FAIL;
		Serial.println("ROS connection lost");
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

    Serial.println("\nEntering Manual Idle State");

    // ensure amps are off and controller is not running
    MotorBoard.disableAllAmps();
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
                MotorBoard.setLEDG(false);
            }
        }
        else {
            // user override -> use default gains and current limits
            configFinished = true;
        }
    }

    // initialize motors
    Serial.print("Initializing Motors ... ");
    for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
        MotorBoard.setGains(ii, kp, ki, kd);
        MotorBoard.setCountDesired(ii, countDesired[ii]);
    }
    Serial.println("done");

    // advance based on mode switch position
    switch (modeState) {
    case Manual:
        return stateManualControl;

    case Ros:
        return stateRosInit;
    }
}

MCBstate ManualControl()
{
    stateCurrent = stateManualControl;

    Serial.println("\nEntering Manual Control State");

    // set desired motor position to current position (prevents unexpected movement)
    for (int ii = 0; ii < MotorBoard.numModules(); ii++)
    {
        countDesired[ii] = MotorBoard.getCountLast(ii);
        MotorBoard.setCountDesired(ii, countDesired[ii]);
    }

    // enable motor amps
    MotorBoard.enableAllAmps();

    // start PID controllers
    timerPid.begin(timerPidCallback, timeStepPid);

    // start manual control
    timerManualControl.begin(timerManualControlCallback, timeStepManualControl);

    // flash led of currently selected motor
    timerMotorSelectLed.begin(motorSelectLedCallback, 350000);

    // keep running until mode switch changed to Ros OR serial command detected
    while (modeState == Manual)
    {
        // NOTE: this while loop must be able to run at least twice as fast as fastest InterruptTimer (usually timerPid)
        noInterrupts();

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

        interrupts();
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

void limitSwitchCallback(void)
{
    limitSwitchFlag = true;
}


void modeSwitchCallback(void)
{
	modeState = (digitalReadFast(MotorBoard.pins.modeSelect) ? Manual : Ros);
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

void subEncoderResetSingleCallback(const std_msgs::UInt8 & msg)
{
    // ensure request is valid
    if (msg.data <= MotorBoard.numModules()) {
        // reset the encoder count for the desired motor
        MotorBoard.resetCount(msg.data);
    }
}

void subEncoderResetAllCallback(const std_msgs::Empty & msg)
{
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
        msgStatus.limit_switch[ii] = false;
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
			delayMicroseconds(400000); // wait for human's slow reaction time
			MotorBoard.readButtons();
		}
		countDesired[currentMotorSelected] = MotorBoard.getCountLast(currentMotorSelected); // ensure we drive relative to current position
		MotorBoard.enableAllAmps();
	}
	else if (MotorBoard.isUpPressed()) {
		countDesired[currentMotorSelected] += countStepManualControl;
		MotorBoard.setCountDesired(currentMotorSelected, countDesired[currentMotorSelected]);
	}
	else if (MotorBoard.isDownPressed()) {
		countDesired[currentMotorSelected] -= countStepManualControl;
		MotorBoard.setCountDesired(currentMotorSelected, countDesired[currentMotorSelected]);
	}
}

//void srvEnableCallback(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res) 
//{
//	if (req.data) { // 'run' command
//		if (ROSenable) {
//			res.success = true;
//			res.message = "Note: motor control already enabled";
//		}
//		else {
//			if (stateCurrent == stateRosIdle) {
//				ROSenable = true;
//				res.success = true;
//				res.message = "motor control enabled";
//			}
//			else {
//				res.success = false;
//				res.message = "Error: MCB not in ROS Idle state";
//			}
//		}
//	}
//	else { // 'stop' command
//		ROSenable = false;
//		if (stateCurrent == stateRosControl) {
//			res.success = true;
//			res.message = "motor control disabled";
//		}
//		else {
//			res.success = false;
//			res.message = "Error: MCB not in ROS Control state";
//		}
//	}
//}

//void srvGetGainsCallback(const beginner_tutorials::srvGetGainsRequest & req, beginner_tutorials::srvGetGainsResponse & res)
//{
//    Serial.print("Entered GetGains CB ... ");
//    //noInterrupts();
//
//    // ensure a module exists at location requested
//    if (!MotorBoard.isModuleConfigured(req.module)) {
//        res.success = -1;
//        res.errorMessage = "No module at location requested";
//    }
//    else {
//        // retrieve gains
//        FloatVec gains(MotorBoard.getGains(req.module));
//
//        // assemble response
//        res.kp = gains.at(0);
//        res.ki = gains.at(1);
//        res.kd = gains.at(2);
//        res.success = 1;
//        res.errorMessage = "no error";
//    }
//
//    //interrupts();
//    Serial.println("returned");
//}

//void srvSetGainsCallback(const beginner_tutorials::srvSetGainsRequest & req, beginner_tutorials::srvSetGainsResponse & res)
//{
//    //noInterrupts();
//
//    // ensure a module exists at location requested
//    if (!MotorBoard.isModuleConfigured(req.moduleToSet)) {
//        res.success = -1;
//        res.errorMessage = "No module at location requested";
//    }
//    else {
//        MotorBoard.setGains(req.moduleToSet, req.kp, req.ki, req.kd);
//        res.success = 1;
//    }
//
//    //interrupts();
//}

//void srvStatusMCBCallback(const beginner_tutorials::srvStatusMCBRequest & req, beginner_tutorials::srvStatusMCBResponse & res)
//{
//    //noInterrupts();
//
//    res.numModules = MotorBoard.numModules();
//    res.currentState = MCBstateToString(stateCurrent);
//
//    for (int ii = 0; ii < 6; ii++) {
//        res.encoderCommand[ii] = MotorBoard.getCountDesired(ii);
//        res.encoderCurrent[ii] = MotorBoard.getCountLast(ii);
//        res.controllerEffort[ii] = MotorBoard.getEffort(ii);
//    }
//
//    uint32_t tmpIP = (uint32_t)nh.getHardware()->wiznet_ip;
//    memcpy(res.ip, &tmpIP, 4);
//    memcpy(res.mac, nh.getHardware()->wiznet_mac, 6);
//    res.limitSwitchTriggered = false;
//
//    //interrupts();
//}

char* MCBstateToString(MCBstate currentState) {
    char* stateName;

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
