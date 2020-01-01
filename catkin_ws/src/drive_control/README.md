# Drive Control

## Overview

This package translates `joy` commands from a joystick into L/R commands to drive the 6 motors in the wheels of the rover. Wheels are constricted to tank drive (individual Left/Right side control). Stick sensitivity with 4 levels was implemented to allow for more precise driving. For example, at 25% sensitivity, full forward on both sticks will result in all wheels driving forward at 25% full speed.

*One stick opposite drive* is a feature that, when enabled, will result in the right stick being able to turn both sides of the rover in _opposite_ direction.

*One stick tandem drive* is a feature that, when enabled, will result in the right stick being able to turn both sides of the rover in _the same_ direction.

## Communication

ROS `joy` command *--UART-->* Master ATmega328P *--I2C-->* Slave ATmega328p (x6) *--GPIO-->* Motor Driver

## Mapping
_JOY-L-Y_: Left wheels forward/reverse

_JOY-R-Y_: Right wheels forward/reverse

_X_: Engage/Disengage software softstop

_MIDDLE_: Engage/Disengage software softstop

_A_: Increase stick sensitivity

_X_: Decrease stick sensitivity

_LB (hold)_: One stick opposite drive

_RB (hold)_: One stick tandem drive
