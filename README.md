# MCB_ROS

Teensy firmware for closed-loop motor control over Ethernet using ROS

## Getting Started

Follow these steps to setup the toolchain for compiling and uploading firmare to the Teensy.

### Prerequisites
- [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
- Micro-USB to USB cable

### Toolchain Installation and Setup

1. Download and install [Arduino 1.8.2](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous/)  
   - Scroll down the page and locate the 1.8.2 install file for your OS (Windows/Linux)

2. Download and install Teensyduino 1.3.6 ([Windows](https://www.pjrc.com/teensy/td_136/TeensyduinoInstall.exe)) ([Linux 64 bit](https://www.pjrc.com/teensy/td_136/TeensyduinoInstall.linux64))  
   - Full instructions available [here](https://www.pjrc.com/teensy/td_download.html)
   - Note that you must install the udev rules for Linux

3. Download current MCB_ROS firmware  
   - Ensure you have selected the correct branch  
   - Use the 'Clone or Download' button to save as a .zip file  
   - Extract the .zip file to the desired location on your machine  
   - Rename the folder 'MCB_ROS' (Arduino requires the folder name to match the .ino filename)  

4. Modify compile options  
   - open up \Arduino\hardware\teensy\avr\boards.txt
   - locate the following line and add the **bolded** text:  
      teensy31.menu.opt.o2std.build.flags.optimize=-O2 **--specs=nosys.specs**  
   - save and close

## Compiling and Uploading Firmware
1. Start the Arduino IDE and open MCB_ROS.ino
2. Connect Teensy via USB cable
3. Configure Arduino for the Teensy
   - Tools -> Board -> Teensy 3.2/3.1
   - Tools -> CPU Speed -> 96 MHz (overclock)
   - Tools -> Optimize -> Faster (this ensures we use the compile options we changed in boards.txt)
   - Tools -> Port -> (depends on OS; select whichever Teensy is connected to)

## ROS Setup/Configuration

## Author
**Trevor Bruns**
