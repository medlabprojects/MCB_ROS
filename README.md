# MCB_ROS

**Teensy firmware for closed-loop motor control over Ethernet using ROS**

# Getting Started

**Follow these steps to setup the toolchain for compiling and uploading firmare to the Teensy.**

### Prerequisites
- [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
- WIZnet ethernet module ([WIZ820io](http://www.wiznet.co.kr/product-item/wiz820io/) or [WIZ850io](http://www.wiznet.co.kr/product-item/wiz850io/)) and [adapter board](https://www.pjrc.com/store/wiz820_sd_adaptor.html) (Rev 2) for Teensy
- Fully assembled Motor Control Board (current version: 1.4) and at least one daughterboard
- Motor amp (e.g. Maxon ESCON module 50/5)
- Micro-USB to USB cable
- A ROS workstation with [Kinetic](http://wiki.ros.org/kinetic/Installation) installed
   - Note: not required to be the same machine you will be setting up for programming the Teensy
- medlab_motor_control_board ROS package
   - Contains the msg definitions used by MCB_ROS

### Toolchain Installation and Setup

1. Download and install [Arduino 1.8.2](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous/)  
   - Scroll down the page and locate the 1.8.2 install file for your OS (Windows/Linux)
   - To make things simpler, install as 'portable'. This will keep all Arduino files within a single directory and is particularly useful if you have another Arduino install that you want to keep intact. Follow the instructions [here](https://www.arduino.cc/en/Guide/PortableIDE).

2. Download and install Teensyduino 1.3.6 ([Windows](https://www.pjrc.com/teensy/td_136/TeensyduinoInstall.exe)) ([Linux 64 bit](https://www.pjrc.com/teensy/td_136/TeensyduinoInstall.linux64))  
   - Full instructions available [here](https://www.pjrc.com/teensy/td_download.html)
   - Note that you must install the udev rules for Linux

3. Download current MCB_ROS firmware  
   - Ensure you have selected the correct branch  
   - Use the 'Clone or Download' button to save as a .zip file  
   - Extract the .zip file to <arduino_directory>/portable/sketchbook  
   - Rename the folder 'MCB_ROS' (Arduino requires the folder name to match the .ino filename)  

4. Download additional libraries
   - Modified version of Adafruit_MCB23008
   - ArduinoSTL
   - Place both folders in <arduino_directory>/portable/sketchbook/libraries

5. Modify compile options  
   - open up <arduino_directory>\hardware\teensy\avr\boards.txt
   - locate the following lines:
   ```
   teensy31.menu.opt.o2std=Faster
   teensy31.menu.opt.o2std.build.flags.optimize=-O2
   ```
   and add **--specs=nosys.specs**:
   ```
   teensy31.menu.opt.o2std=Faster
   teensy31.menu.opt.o2std.build.flags.optimize=-O2 --specs=nosys.specs
   ```
   - save and close
### ROS Setup/Configuration
Communication between ROS and the Teensy is accomplished using rosserial. Follow these steps to install the rosserial binaries and generate the Arduino header files. Full instructions and more information can be found [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).

1. Install rosserial
- On your ROS workstation, open a terminal and run the following commands: 
   ```
   sudo apt-get install ros-kinetic-rosserial-arduino
   sudo apt-get install ros-kinetic-rosserial
   ```
2. Install ros_lib into the Arduino environment  
- If you set up the Arduino toolchain above on your ROS workstation, run:
```
cd <arduino_directory>/portable/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
- If you set up the Arduino toolchain on another (e.g. Windows) machine, create ros_lib in an empty directory and then copy that folder into the library folder on the other machine
```
cd <some_empty_directory>
rosrun rosserial_arduino make_libraries.py .
```
   *Note: if you get error messages about not finding rosserial-arduino, make sure you've re-sourced your ROS workspace. If you've followed the ROS setup tutorials and added this to your .bashrc script, you can just open a new terminal.*

### Compiling and Uploading Firmware
1. Start the Arduino IDE and open MCB_ROS.ino
2. Connect Teensy via USB cable
3. Configure Arduino for the Teensy
   - Tools -> Board -> Teensy 3.2/3.1
   - Tools -> CPU Speed -> 96 MHz (overclock)
   - Tools -> Optimize -> Faster (this ensures we use the compile options we changed in boards.txt)
   - Tools -> Port -> (depends on OS; select whichever Teensy is connected to)
4. Press the upload button
   - This will compile and then open the Teensy Loader application
   - If Teensy Loader does not automatically upload, press the white button labeled 'PGM' on the Motor Control Board to manually enter Program Mode  

   *Hint: if there are errors during compilation, you can get more information by turning on verbose output. File -> Preferences -> Settings -> Show verbose output during: compilation*

### ESCON Configuration
Maxon's ESCON line of servo controllers have many nice features and can provide a lot of power in a small form-factor. A daughterboard has been designed for the [ESCON Module 50/5](https://www.maxonmotorusa.com/maxon/view/product/control/4-Q-Servokontroller/438725). It can drive brushed or brushless motors up to 250W. Configuration will depend on the specific motor used, but is a simple procedure done over USB.

1. Download and install the [ESCON Studio](https://www.maxonmotorusa.com/maxon/view/content/ESCON-Detailsite) software
   - Download button is at the bottom of the page or use [direct download link](https://www.maxonmotorusa.com/medias/sys_master/root/8825156173854/ESCON-Setup.zip)
2. Open ESCON Studio and connect the module via USB
3. If you already have a configuration file for you motor (e.g. *Configuration_EC13_2018-05-22.edc*) you can use that and skip the remaining steps
   - File -> Download Parameters
   - Select the .edc file for your motor
   - Wait for the parameters to be loaded and confirm on the 'Controller Monitor' window that they are correct
   - Disconnect USB cable to finish
4. Otherwise, locate the datasheet for your motor
5. Open the ESCON Studio 'Startup Wizard'
   - Tools -> Startup Wizard
6. Step through the wizard, making sure the parameters match the motor datasheet. **The critical paramaters to ensure compatibility with the MCB are the following:**
   - Mode of Operation -> *Current Controller*
   - Enable Functionality -> *Enable* -> *Digital Input 2* -> *High active*
   - Set Value -> *Analog Set Value* -> *Analog Input 1*
      - Current at -> *0.0 V* : *0.0 A*
      - Current at -> *10.0 V* : *max desired current for your motor*
   - Offset -> *Fixed Offset* -> *0.0 A*
   - Digital Inputs & Outputs
      - Digital Input 1 -> *None*
      - Digital Input 2 -> *Enable*
      - Digital Output 3 -> *Ready*
      - Digital I/O 4 -> *None*
   - Analog Inputs
      - Analog Input 1 -> *Set Value*
      - Analog Input 2 -> *None*
      - Potentiometer 1 -> *None*
   - Analog Outputs -> *None* for all
   - Digital Output 3 -> Polarity -> *High active*
7. Once finished with Startup Wizard, click Finish
8. If you need to configure multiple modules for the same type of motor, you can save this configuration
   - File -> Upload Parameters -> Type filename and save the .edc file somewhere
   - Follow steps 2 & 3 for the remaining modules
      
### MCB Serial Configuration
Before we can connect to the ROS Master, we must first set a few parameters to make each MCB unique. This will prevent any conflicts/issues with other devices on the network. Once set, they are stored in the Teensy's builtin EEPROM. This is non-volatile memory and does not get reset after powerdown. Once set, the MCB will retrive the last settings from memory upon startup.

1. Open a serial terminal program (e.g. 'Serial Monitor' in the Arduino IDE)
2. Ensure MCB is powered down
   - No external power to screw terminals
   - No USB cable connected
3. Connect USB cable to the Teensy on the MCB and open/start the serial connection
4. Move the Mode switch on the motherboard from ROS to Manual
   - If it was already on Manual, toggle it to ROS and then back again to reset the state machine
   - You should now see the serial configuration instructions in your serial terminal
5. Set a unique namespace for this MCB. Its ROS topics will apear under this label (e.g. "*namespace*/encoder_command").
   - Type 'n' into the terminal and press enter
   - Type you desired namespace (NO punctuation or spaces!) and press enter
6. Set a unique IP address for this MCB
   - Type 'i' into the terminal and press enter
   - Type the last octet of your desired IP (e.g. '*40*' will yield 192.168.0.*40*) and press enter
7. Set a unique MAC address for this MCB
   - Type 'm' into the terminal and press enter
   - Type the last octet of your desired MAC (e.g. '*ED*' will yield DE:AD:BE:EF:FE:*ED*) and press enter
8. Review our new settings
   - Type 'c' into the terminal and press enter
   - Confirm the settings are correct
9. Save the new settings into the EEPROM
   - Type 's' into the terminal and press enter
10. You are now done and may resume operation as usual

# How to Use

There are two ways you can use the MCB to drive your motors: *manually* (using buttons on the motherboard) or via *ROS* (over ethernet).

### Initial Powerup

Follow these steps when powering a newly assembled MCB for the first time.

1. If you haven't already, go through the Getting Started guide found above
2. Connect at least one motor via a daughterboard
   - Ensure daughterboards are plugged in consecutively (i.e. starting at M0 with no gaps)
3. You must connect a normally-closed switch to the 'E-STOP' pins on the motherboard
   - Any break in the connection will disable all amps and return the MCB to an idle state
4. Power on the MCB via the screw terminals (USB will only power the Teensy, not the amps)
   - Verify that polarity matches the labels on the motherboard (positive terminal is closest to the edge)
      - Versions 1.4 and up include reverse polarity protection
   - It is recommended to use a current-limited power supply, especially when powering on for the first time after assembly
   - Start with the limit set to ~100-200 mA
   - If the MCB is trying to draw more current, there is likely a short circuit somewhere
      - You can verify this by disconnecting power and measuring resistance between each power rail (Vin/5V/3.3V) and ground
   - There is a green LED near the screw terminals that indicates proper operation of the 5V regulator
      - If this LED is not lit, check polarity of your input
   - Once you have tested for shorts, you can increase the current limit to whatever is reasonable for your motor
5. Upon powerup, the MCB will go through a brief initialization sequence where it will detect how many daughterboards are connected and configure all the ICs
   - Any errors during initialization are reported over the USB serial connection

### Manual Control
Manual Control is a good way to verify everything has been connected and configured correctly. It allows you to indivually select and jog motors. This mode is also useful for quickly moving your robot to a desired pose without requiring an external connection to a computer. Only power is necessary, no USB or ethernet.

1. Assuming you have completed the Initial Powerup steps above, connect your motor(s)/daughterboard(s) and then turn on power
2. Set the mode switch on the motherboard to 'MANUAL'
3. It is now in an idle state, waiting for user input
   - All amps are disabled (i.e. motors are not powered)
4. Enter into the Manual Control state
   - Simultaneously press and hold all three capacitive buttons on the motherboard (Up/Down/Menu)
   - The green LEDs under each daughterboard will light in succession
   - Hold until all six have been lit and they begin flashing together, then release
   - The green LED for Motor 0 should now be flashing to indicate that it is selected, while its red LED indicates that it is powered
5. Use the Up/Down buttons to jog the motor
   - ***CAUTION:*** jog speed is dependent on the encoder resolution and any gearhead/gear reduction
      - This can be changed by editing MCB_FSM.cpp and changing the countStepManualControl variable (currently line 36)
      - Be sure to recompile and upload after modifying
6. Select a different motor by holding the Menu button and then pressing/holding Up or Down
   - All amps are disabled while menu button is held
   - Once menu is released, the selected motor will be enabled and powered
7. If a limit switch is triggered, the motor will be automatically disabled
   - You can reset it by brielfy pressing/releasing the Menu button
8. When finished, you have two option for disabling power to all motors:
   - Disconnect/turn off main power input
   - Toggle the Mode switch to ROS and then back to Manual (putting it back in the Manual Idle state)

### ROS Control
ROS control requires an ethernet connection to a machine running the [rosserial_server](http://wiki.ros.org/rosserial_server) node. The MCB can then be controlled via it's topics (e.g. /encoder_command) using either a custom node or the command line. These steps will demonstrate basic operation using ROS. A full description of each topic can be found in the ROS Topics section below.

1. Assuming you have completed the Initial Powerup steps above, connect your motor(s)/daughterboard(s) *and ethernet cable* and then turn on power
   - It can be helpful to use the USB serial connection to receive status/error messages
2. Set the mode switch on the motherboard to 'ROS'
   - It will now initialize the WIZnet ethernet board and attempt to connect to a rosserial_server node
3. On your ROS machine, open a terminal and start the rosserial_server node
   ```
   roslaunch rosserial_server socket.launch
   ```
   - After a few seconds, you should see (in the terminal window and serial monitor) that a connection has been established
   - If it cannot connect to ROS, ping the MCB to verify that it is visible on the network
      ```
      ping 192.168.0.XX
      ```
4. Verify proper connection by checking that MCB topics are available (e.g. '*namespace*/encoder_command')
   ```
   rostopic list
   ```
5. The MCB is now in the ROS Idle state and awaiting commands
   - All amps are disabled (i.e. motors are not powered)
6. Enter the ROS Control state by publishing to /enable_ros_control topic
   ```
   rostopic pub -1 /namespace/enable_ros_control std_msgs/Bool 1
   ```
7. All motors are disabled by default when first entering ROS Control state
   - Enable a single motor: `rostopic pub -1 /namespace/enable_motor medlab_motor_control_board/EnableMotor XXXXXX`
   - Enable all motors: `rostopic pub -1 /namespace/enable_all_motors std_msgs/Bool 1`
   - Red LEDs indicate which motors are enabled
8. We can move a motor by sending a new encoder position to the /encoder_command topic

# ROS Topics
Each topic name will be preceded by the namespace you chose during the 'MCB Serial Configuration' section above. Thus, you must include that namespace when pubishing to the following topics (e.g. '/*MCB1*/encoder_command').

| Topic Name | Input (*to MCB*) or Output (*from MCB*) | Description | Message Type | Example |
| ---------- | --------------------------------------- | ----------- | ------------ | ------- |
| **/status** | Output | Receives information about current MCB status | medlab_motor_control_board::McbStatus | `rostopic echo namespace/status` |
| **/get_status** | Input | Requests a new update from the /status topic | std_msgs::Empty | `rostopic pub -1 /namespace/get_status std_msgs/Empty` |
| **/enable_ros_control** | Input | Enters/leaves ROS Control state | std_msgs::Bool | `rostopic pub -1 /namespace/enable_ros_control std_msgs/Bool 1` |
| **/enable_motor** | Input | Enables/disables power for a single motor | medlab_motor_control_board::EnableMotor | `rostopic pub -1 /namespace/enable_motor medlab_motor_control_board/EnableMotor XXXXX` |
| **/enable_all_motors** | Input | Enables/disables power for all motors | std_msgs::Bool | `rostopic pub -1 /namespace/enable_all_motors std_msgs/Bool 1` |
| **/encoder_current** | Output | Receive the most recent encoder positions for all motors | medlab_motor_control_board::McbEncoderCurrent | `rostopic echo /namespace/encoder_current` |
| **/encoder_command** | Input | Send desired encoder positions for all motors | medlab_motor_control_board::McbEncoders | `rostopic pub -1 /namespace/encoder_command medlab_motor_control_board/McbEncoders XXXX` |
| **/limit_switch_event** | Output | Receive a message whenever a limit switch is triggered | medlab_motor_control_board::EnableMotor | `rostopic echo /namespace/limit_switch_event` |
| **/encoder_zero_single** | Input | Resets a motor's current position to zero | std_msgs::UInt8 | `rostopic pub -1 /namespace/encoder_zero_single std_msgs/UInt8 3`
| **/encoder_zero_all** | Input | Resets all motors current positions to zero | std_msgs::Empty | `rostopic pub -1 /namespace/encoder_zero_all std_msgs/Empty`
| **/set_gains** | Input | Sets new PID gain values for each motor | medlab_motor_control_board::McbGains | `rostopic pub -1 /namespace/set_gains medlab_motor_control_board::McbGains '{motor: 1, p: 0.1, i: 0.0002, d: 0.01}'`

## Author
**Trevor Bruns**
