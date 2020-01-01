**See README.md on `drive_control_test` project for more context on how to use this node**

This is the MASTER code for the drive system. It handles the polling of encoders (which don't happen right now), but primarily
the sending of new targets from the `drive_commands` topic that it subscribes to.

**To upload:** 
0. `roscore`
1. `cd catkin_ws`
2. `catkin_make drive_master_avr_firmware_move-upload`. This uplaods the code to the Arduino (which should be plugged in, on `/dev/ttyACM0`. If you run into a permissions issue, `chmod +777 /dev/ttyACM0`
3. `rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600`. This starts `rosserial`, allowing the node to receive messages from `drive_commands`. Currently, rosserial requires 57600 baud rate.


