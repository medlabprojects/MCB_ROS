#include "MCB_FSM.h"
#include "MCB.h"
#include "MCBmodule.h"
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h" 
#include <beginner_tutorials/msgEncoder.h>
#include <std_srvs\SetBool.h>

/****************** GLOBALS *********************/

// Motor Control Board
int8_t numberOfModules = 6; // number of modules (i.e. motors) plugged into this board
MCB MotorBoard(numberOfModules);	// construct motor control board
float maxMotorAmps = 1.0;
int8_t currentMotorSelected = 0; // for manual control using Up/Down buttons
IntervalTimer timerMotorSelectLed;
volatile bool motorSelectLedState = false;
MCBstate stateCurrent;
enum CTRLswitchState { local, remote };
volatile CTRLswitchState stateCTRL;
bool ROSenable = false; // ROS must set this true via 'enableMCB_srv' to control motors

// ROS
ros::NodeHandle_<WiznetHardware> nh;
beginner_tutorials::msgEncoder msgEncoderCurrent; // stores most recent encoder counts to be sent via publisher
ros::Publisher pubEncoderCurrent("topicEncoderCurrent", &msgEncoderCurrent); // publisher for sending out updated motor positions
ros::Subscriber<beginner_tutorials::msgEncoder> subEncoderCommand("topicEncoderCommand", &subEncoderCommandCallback); // subscriber for new motor commands
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> srvEnableMCB("serviceEnableMCB", &srvEnableCallback); // service to handle MCB enable/disable commands

// PID Controller
IntervalTimer timerPid; // PID controller timer interrupt
int32_t countDesired[6]; // does this need to be volatile?
float frequencyPid = 1000.0; // [Hz]
uint32_t timeStepPid = uint32_t(1000000.0 / frequencyPid); // [us]
float kp = 0.0004, ki = 0.000002, kd = 0.01; // work well for 1 kHz
// float kp = 0.0002, ki = 0.000001, kd = 0.01; // work ok for 2 kHz

// ROS
IntervalTimer timerRos; // ROS timer interrupt
float frequencyRos = 500.0; // [Hz]
uint32_t timeStepRos = uint32_t(1000000.0 / frequencyRos); // [us]

// Local Control
IntervalTimer timerLocalControl; // Button read timer interrupt
float frequencyLocalControl = 100.0; // [Hz]
uint32_t timeStepLocalControl = uint32_t(1000000.0 / frequencyLocalControl); // [us]
volatile uint32_t countStepLocalControl = 500; // [counts] step size for each up/down button press


MCBstate stepStateMachine(MCBstate stateNext) 
{
	
	switch (stateNext)
	{
	case statePowerUp:
		return PowerUP();

	case stateLocalIdle:
		return LocalIdle();

	case stateLocalControl:
		return LocalControl();

	case stateRosInit:
		return RosInit();

	case stateRosIdle:
		return RosIdle();

	case stateRosControl:
		return RosControl();

	default:
		Serial.println("Error: Unrecognized State");
		while (1);
	}
}

MCBstate PowerUP(void)
{
	stateCurrent = statePowerUp;

	// start serial port
	Serial.begin(115200);

	// initialize motor control board
	Serial.print("Initializing Motor Control Board ... ");
	MotorBoard.init();

	// create pin change interrupt for CTRL switch
	attachInterrupt(MotorBoard.pins.CTRL, CTRLswitchCallback, CHANGE);

	// run callback once to initialize stateCTRL 
	CTRLswitchCallback();

	Serial.println("Success!");

	// advance based on CTRL switch position 
	switch (stateCTRL) {
	case local:
		return stateLocalIdle;

	case remote:
		return stateRosInit;
	}
}

MCBstate LocalIdle(void)
{
	stateCurrent = stateLocalIdle;

	Serial.println("\nEntering Local Idle State");

	// ensure amps are off and controller is not running
	MotorBoard.disableAllAmps();
	timerPid.end();

	uint32_t holdTime = 2000; // [ms] how long buttons must be held before function returns
	uint32_t timeButtonsPressed = 0; // [ms] how long buttons have been held
	uint32_t timeStart = 0;
	MotorBoard.setLEDG(false); // turn off green LEDs
	bool configFinished = false;

	// wait until gains have been set via serial OR user overrides by holding buttons
	while ((stateCTRL == local) && !configFinished) {
		// check for serial commands
		if (Serial.available() > 0) {
			char cmd = Serial.read();

			// TO DO: add other commands
			switch (cmd) {
			case 'r':
			case 'R': // 'run' command
				Serial.println("Command = 'Run'");
				configFinished = true;
				break;

			default:
				Serial.println("Command not recognized");
			}
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
	for (int ii = 0; ii < numberOfModules; ii++) {
		MotorBoard.setGains(ii, kp, ki, kd);
		MotorBoard.setCount(ii, countDesired[ii]);
		MotorBoard.setMaxAmps(ii, maxMotorAmps);
	}
	Serial.println("done");

	// advance based on CTRL switch position
	switch (stateCTRL) {
	case local:
		return stateLocalControl;

	case remote:
		return stateRosInit;
	}
}

MCBstate LocalControl()
{
	stateCurrent = stateLocalControl;

	Serial.println("\nEntering Local Control State");

	// set desired motor position to current position (prevents unexpected movement)
	for (int ii = 0; ii < numberOfModules; ii++) 
	{
		countDesired[ii] = MotorBoard.getCount(ii);
		MotorBoard.setCount(ii, countDesired[ii]);
	}

	// enable motor amps
	MotorBoard.enableAllAmps();

	// start PID controllers
	timerPid.begin(timerPidCallback, timeStepPid);

	// start local control
	timerLocalControl.begin(timerLocalControlCallback, timeStepLocalControl);

	// flash led of currently selected motor
	timerMotorSelectLed.begin(motorSelectLedCallback, 350000);

	// keep running until CTRL switch changed to remote OR serial command detected
	while (stateCTRL == local) 
	{
		// check serial for commands
		if (Serial.available()) 
		{
			char cmd = Serial.read();
			if ((cmd == 'x') || (cmd == 'X'))  // 'stop' command
			{
				Serial.println("'Stop' command received");
				// stop checking buttons
				timerLocalControl.end();
				timerMotorSelectLed.end();

				// power off motors and disable PID controller
				MotorBoard.disableAllAmps();
				timerPid.end();

				// turn off green LEDs
				MotorBoard.setLEDG(LOW);

				// return to Local Idle state to process command
				return stateLocalIdle;
			}
			
		}
	}

	// stop checking buttons
	timerLocalControl.end();
	timerMotorSelectLed.end();

	// power off motors and disable PID controller
	MotorBoard.disableAllAmps();
	timerPid.end();

	// turn off green LEDs
	MotorBoard.setLEDG(LOW);

	return stateRosInit; // while() only exits if CTRL switched to 'remote'
}

MCBstate RosInit()
{
	stateCurrent = stateRosInit;

	Serial.println("\nEntering ROS Initialization state");

	// set up Wiznet and connect to ROS server
	Serial.print("Setting up ethernet connection ... ");
	nh.initNode();

	// repeatedly attempt to setup the hardware, loop on fail, stop on success
	while ((nh.getHardware()->error() < 0) && (stateCTRL == remote)) {
		Serial.print("WIZnet eror = ");
		Serial.println(nh.getHardware()->error());

		nh.initNode();
	}

	if (stateCTRL == local) {
		return stateLocalIdle;
	}

	Serial.println("Success!");

	// initialize ROS connection
	Serial.print("Connecting to ROS Network ... ");
	nh.advertise(pubEncoderCurrent);
	nh.subscribe(subEncoderCommand);
	nh.advertiseService(srvEnableMCB);
	while (!nh.connected() && (stateCTRL == remote)) {
		nh.spinOnce();
	} // wait until connection established or CTRL switched to 'local'

	if (stateCTRL == local) {
		return stateLocalIdle;
	}

	Serial.println("Success!");

	// grab default parameters from server
	Serial.println("Checking server for default parameters:");
	if (nh.getParam("P", &kp)) {
		Serial.print("P = ");
		Serial.print(kp, 8);
	}
	if (nh.getParam("I", &ki)) {
		Serial.print(", I = ");
		Serial.print(ki, 8);
	}
	if (nh.getParam("D", &kd)) {
		Serial.print(", D = ");
		Serial.println(kd, 8);
	}
	if (nh.getParam("controller_frequency", &frequencyPid)) {
		timeStepPid = uint32_t(1000000.0 / frequencyPid); // [us]
		Serial.print("controller_frequency = ");
		Serial.println(frequencyPid);
	}
	if (nh.getParam("communication_frequency", &frequencyRos)) {
		timeStepRos = uint32_t(1000000.0 / frequencyRos); // [us]
		Serial.print("communication_frequency = ");
		Serial.println(frequencyRos);
	}
	if (nh.getParam("current_limit", &maxMotorAmps)) {
		Serial.print("current_limit = ");
		Serial.println(maxMotorAmps);
	}

	// initialize motors
	Serial.print("Initializing Motors ... ");
	for (int ii = 0; ii < numberOfModules; ii++) {
		MotorBoard.setGains(ii, kp, ki, kd);
		MotorBoard.setCount(ii, countDesired[ii]);
		MotorBoard.setMaxAmps(ii, maxMotorAmps);
	}
	Serial.println("done");

	switch (stateCTRL) {
	case local:
		return stateLocalIdle;

	case remote:
		return stateRosIdle;
	}
}

MCBstate RosIdle()
{
	stateCurrent = stateRosIdle;

	Serial.println("\nEntering ROS Idle state");
	Serial.println("waiting for enable signal via enableMCB_srv");

	// wait for ROS enable command via service call
	while (!ROSenable && nh.connected() && (stateCTRL == remote)) {
		nh.spinOnce();
	}

	if (!nh.connected()) {
		return stateRosInit;
	}

	switch (stateCTRL) {
	case local:
		return stateLocalIdle;

	case remote:
		return stateRosControl;
	}
}

MCBstate RosControl()
{
	stateCurrent = stateRosControl;

	Serial.println("\nEntering ROS Control state");

	// set desired motor position to current position (prevents unexpected movement)
	for (int ii = 0; ii < numberOfModules; ii++)
	{
		countDesired[ii] = MotorBoard.getCount(ii);
		MotorBoard.setCount(ii, countDesired[ii]);
	}

	// enable motor amps
	MotorBoard.enableAllAmps();

	// start PID controllers
	timerPid.begin(timerPidCallback, timeStepPid);

	// start ROS update timer
	timerRos.begin(timerRosCallback, timeStepRos);

	// loop until disconnected OR ROS 'disable' command OR CTRL switched to 'local'
	while (ROSenable && nh.connected() && (stateCTRL == remote)) {
		//MotorBoard.getMaxAmps(0);
		MotorBoard.toggleLEDG(0);
		//if (!nh.connected()) {
		//	MotorBoard.toggleLEDG(1);
		//	break;
		//}
	}

	//while (nh.getHardware()->isConnected() && (stateCTRL == remote));

	// power off motors, disable PID controller, and stop ROS timer
	MotorBoard.disableAllAmps();
	timerPid.end();
	timerRos.end();
	ROSenable = false;

	if (!nh.connected()) {
		nh.getHardware()->error() = WiznetHardware::ERROR_CONNECT_FAIL;
		Serial.println("ROS connection lost");
		return stateRosInit;
	}

	if (stateCTRL == local) {
		return stateLocalIdle;
	}
	else { // 'stop' command must have been received
		return stateRosIdle; 
	}
}

void CTRLswitchCallback(void)
{
	stateCTRL = (digitalReadFast(MotorBoard.pins.CTRL) ? local : remote);
}

void timerPidCallback(void)
{
	noInterrupts(); // disable all interrupts to ensure this process completes sequentially

	MotorBoard.setLEDG(2, HIGH);
	
	MotorBoard.stepPID();
	
	MotorBoard.setLEDG(2, LOW);

	interrupts();
}

void timerRosCallback(void)
{
	noInterrupts(); // disable all interrupts to ensure this process completes sequentially

	MotorBoard.setLEDG(1, HIGH);

	// assemble encoder positions message and publish()
	for (int ii = 0; ii < numberOfModules; ii++) {
		msgEncoderCurrent.val[ii] = MotorBoard.getCount(ii);
	}
	pubEncoderCurrent.publish(&msgEncoderCurrent);

	nh.spinOnce();

	MotorBoard.setLEDG(1, LOW);

	interrupts();

}

void subEncoderCommandCallback(const beginner_tutorials::msgEncoder& encCommands)
{
	MotorBoard.setLEDG(3, HIGH);

	// set desired motor positions with values received over ROS
	for (int ii = 0; ii < numberOfModules; ii++) {
		MotorBoard.setCount(ii, encCommands.val[ii]);
	}
	
	//for (int ii = 0; ii < numberOfModules; ii++) {
	//	MotorBoard.modules.at(ii).setCountDesired(encCommands.val[ii]);
	//}

	MotorBoard.setLEDG(3, LOW);
}

void motorSelectLedCallback(void)
{
	motorSelectLedState = !motorSelectLedState;
	MotorBoard.setLEDG(currentMotorSelected, motorSelectLedState);
}

void timerLocalControlCallback(void)
{
	MotorBoard.readButtons();

	if (MotorBoard.isMenuPressed()) {
		MotorBoard.disableAllAmps(); // stop motors momentarily
		for (int ii = 0; ii < numberOfModules; ii++) {
			MotorBoard.setLEDG(ii, false);
		}
		MotorBoard.setLEDG(currentMotorSelected, true);

		while (MotorBoard.isMenuPressed()) {
			if (MotorBoard.isUpPressed()) {
				MotorBoard.setLEDG(currentMotorSelected, false);
				currentMotorSelected++;
				if (currentMotorSelected >(numberOfModules-1)) { 
					currentMotorSelected = 0; }
				MotorBoard.setLEDG(currentMotorSelected, true);
			}
			else if (MotorBoard.isDownPressed()) {
				MotorBoard.setLEDG(currentMotorSelected, false);
				currentMotorSelected--;
				if (currentMotorSelected < 0) { 
					currentMotorSelected = (numberOfModules-1); }
				MotorBoard.setLEDG(currentMotorSelected, true);
			}
			delayMicroseconds(400000); // wait for human's slow reaction time
			MotorBoard.readButtons();
		}
		countDesired[currentMotorSelected] = MotorBoard.getCount(currentMotorSelected); // ensure we drive relative to current position
		MotorBoard.enableAllAmps();
	}
	else if (MotorBoard.isUpPressed()) {
		countDesired[currentMotorSelected] += countStepLocalControl;
		MotorBoard.setCount(currentMotorSelected, countDesired[currentMotorSelected]);
	}
	else if (MotorBoard.isDownPressed()) {
		countDesired[currentMotorSelected] -= countStepLocalControl;
		MotorBoard.setCount(currentMotorSelected, countDesired[currentMotorSelected]);
	}
}

void srvEnableCallback(const std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res) {
	if (req.data) { // 'run' command
		if (ROSenable) {
			res.success = true;
			res.message = "Note: motor control already enabled";
		}
		else {
			if (stateCurrent == stateRosIdle) {
				ROSenable = true;
				res.success = true;
				res.message = "motor control enabled";
			}
			else {
				res.success = false;
				res.message = "Error: MCB not in ROS Idle state";
			}
		}
	}
	else { // 'stop' command
		ROSenable = false;
		if (stateCurrent == stateRosControl) {
			res.success = true;
			res.message = "motor control disabled";
		}
		else {
			res.success = false;
			res.message = "Error: MCB not in ROS Control state";
		}
	}
}