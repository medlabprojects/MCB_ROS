# MCB_ROS

Teensy firmware for closed-loop motor control over Ethernet using ROS

## Getting Started

Follow these steps to setup the toolchain for compiling and uploading firmare to the Teensy.

### Prerequisites
```
Teensy 3.2
Micro-USB to USB cable
```

### Installation and Setup

Download and install [Arduino 1.8.2](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous/)  

Download and install [Teensyduino 1.3.6](https://www.pjrc.com/teensy/td_136/TeensyduinoInstall.exe)  

Download current MCB_ROS firmware
```
Ensure you have selected the correct branch  

Use the 'Clone or Download' button to save as a .zip file  

Extract the .zip file to the desired location on your machine  

Rename the folder 'MCB_ROS' (Arduino requires the folder name to match the .ino filename)  

```

Modify compile options
```
open up \Arduino\hardware\teensy\avr\boards.txt
locate the following line and add the **bolded** text
teensy31.menu.opt.o2std.build.flags.optimize=-O2 **--specs=nosys.specs**
save and close
```


## Compiling and Uploading Firmware
To Do

## Authors

**Trevor Bruns**
