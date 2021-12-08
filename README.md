# PegasusArm OS
# Codebase for the open-source control of the Amatrol Pegasus Robot
The goal of this repository is to create an open-source code capable of controlling a Amatrol Pegasus robotic arm.
Based on the design constraints given for this project, it is chosen to use a cascaded microcontroller system: A Raspberry Pi Model 3 B+ is used as the main microcontroller, with its code written in Python 3. In order to secure rapid local control feedback loops and ensuring all encoder changes are registered, a Teensy 4.0 is used, an ARM-based microcontroller with an impressive 700 MHz clock speed that can utilize C++ Arduino code. The control code of the RPi is based on the theory explained in the book *Modern Robotics: Mechanics, Planning, and Control*. Therefore, many functions are also derived from the *modern_robotics* Python library. However,
PegasusArm OS offers additional error handling, test files, as well as additional functionality. Moreover, as PegasusArm OS is fully dedicated to the control of the Amatrol Pegasus robot arm, it also exhibits work-arounds that come from the unique design of this robot, like its differential drive at the wrist axis.

The current state of the codebase comes with four elementary types of control: Position-, velocity-, force-, and impedance control. Note, however, that due to the brief nature of the project timeline, field testing of the code has been minimal. Likely, for real control, several model parameters have to be optimized. The model can be found in 'robot_init.py'. If one would like to know more about this project, the final report with supplementary materials can be found [here](https://drive.google.com/drive/folders/1nO_QL9e1zpBhKMMl1qTbNvlxqkCx4495?usp=sharing).

## Dependencies
### Python
- [modern_robotics](https://github.com/NxRLab/ModernRobotics)
- numpy 1.21.2
- PySerial 3.5
- sys
- os
- time

## License
The PegasusArm OS repository falls under the **GNU General Public License v3.0**. For more information, please see *LICENSE.md*.

## Acknowledgements
Main contributor: V.S. Boon
Code commissioned by the mechatronics department of the Dedan Kimathi University of Technology.

## Closing words
Any contribution are more than welcomed, as I hope that the project was not performed in vain. However, I (the main contributor of this repository) doesn't vouch for the reliability of any continuations of the PegasusArm OS. If you have completed a project that utilizes the PegasusArm OS, you could make me more than happy by sending a some results of what you did to [v.s.boon@student.utwente.nl](mailto:v.s.boon@student.utwente.nl) 
