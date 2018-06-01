# MCB_ROS

Teensy firmware for closed-loop motor control over Ethernet using ROS

## Getting Started

Follow these steps to setup the toolchain for compiling and uploading firmare to the Teensy.

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

## How to Use
### Serial


## Author
**Trevor Bruns**
