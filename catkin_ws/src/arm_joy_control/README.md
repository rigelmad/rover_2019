# Binary/Analog Arm Control Scheme

## Overview

This package translates `joy` commands from a joystick into joint-by-joint control commands for each joint of our 5-DOF + end effector arm. Constricted by the number of analog/digital inputs on an Xbox 360 controller, we opted to make three of the joints analog controllable, and three binary controllable.

## Communication

ROS `joy` command *--UART-->* Master ATmega328P *--I2C-->* Slave ATmega328p (x6) *--GPIO-->* Motor Driver

## Mapping
_JOY-L-X_: Shoulder Positive/Negative

_B_: UpperArm Positive

_X_: UpperArm Negative

_UP_: MidArm Positive

_DOWN_: MidArm Negative

_JOY-R-Y_: LowerArm Positive/Negative

_RT_: Wrist Positive

_LT_: Wrist Negative

_START_: Finger Positive

_BACK_: Finger Negative

_MIDDLE_: Engage/Disengage software softstop
