# Drive_Slave_AVR

Code for the slave AVR on the drive. This code utilizes I2C communication protocol provided by Arduino's Wire.h. 

**To upload:** Simply open `arduino_test/i2c_test_slave/i2c_test_slave.ino` in the Arduino IDE. 
On one of the first lines, you'll see `#define MOTOR_ADDR x` where x should be a number between 1 and 6. Each slave can be individually addressed using 
this address. **Currently the layout of the master node dictates the following:**

| Address | Motor Desc. |
| ------ | ------ |
| 1 | left |
| 2 | left | 
| 3 | left | 
| 4 | right | 
| 5 | right | 
| 6 | right | 

Basis of the code is simple - when the Slave receives the byte that indicates that an encoder poll has been requested, it will send its encoder posisiton to the 
master over I2C. The only other byte code defined dictates that a new target is coming through. Slave will read that target 