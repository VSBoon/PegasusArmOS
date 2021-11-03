# PegasusArmOS
The goal of this repository is to create an open-source code capable of controlling a Amatrol Pegasus robotic arm. \n\n
Based on the design constraints given for this project, it is chosen to use a cascaded microcontroller system: \n
A Raspberry Pi Model 3 B+ is used as the main microcontroller, with its code written in Python 3. In order to \n
secure rapid local control feedback loops and ensuring all encoder changes are registered, a Teensy 4.0 is used, \n
an ARM-based microcontroller with an impressive 700 MHz clock speed that can utilize C++ Arduino code. \n\n
The control code of the RPi is based on the theory explained in the book *Modern Robotics: Mechanics, Planning, \n 
and Control*. Therefore, many functions are also derived from the *modern_robotics* Python library. However, \n 
PegasusArm OS offers additional error handling, test files, as well as additional functionality. Moreover, \n
as PegasusArm OS is fully dedicated to the control of the Amatrol Pegasus robot arm, it also exhibits work-arounds \n 
that come from the unique design of this robot, like its differential drive at the wrist axis.

## Dependencies
- [modern_robotics](https://github.com/NxRLab/ModernRobotics)
- numpy 1.21.2
- PySerial 3.5
- sys
- os
- time

## License
The PegasusArm OS repository falls under the **GNU General Public License v3.0**. For more information, please see *LICENSE.md*.

##Acknowledgements
Main contributor: V.S. Boon
Additional contributors: M.M. Gichane
Written on behalf of Dedan Kimathi University of Technology.