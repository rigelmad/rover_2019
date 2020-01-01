# Life Detection

## Overview

This package translates `joy` commands from a joystick into commands to control the life detection system. The system features 20 GPIO solenoids, 4 I2C controlled pumps, and 3 GPIO ball valves, as well as a linear actuator, and a DC motor to drive a scoop to collect soil samples.

The operation of all peripherals was broken down into 12 subroutines, dictated by the mechanical team. For example, _Top Add Water_ enabled solenoids \#2 and \#19 for 7 seconds, before disabling them. During the 7 seconds, a user may not trigger another routine unless they _Hard Stop_ the current routine. 

## Communication

ROS `joy` command *--UART-->* Master (Core) ATmega328P *--I2C-->* Slave ATmega328p (x3) *--(Various)-->* Peripherals

_Left Stick_: Scoop Up/Down
_Right Stick_: Actuator Up/Down
_RB_: Top Add Water
_RIGHT_: Move Slurry
_A_: Site 1
_X_: Site 2
_Y_: Site 3
_B_: Site 4
_LB_: Site 5
_LEFT_: Clean 1
_UP_: Clean 2
_DOWN_: Clean 3
_START_: Pred Add Water
_BACK_: Purge
_MIDDLE_: Hard Stop
