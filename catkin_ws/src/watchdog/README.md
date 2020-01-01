The watchdog system.
This node will log and keep track of system information
It will also track and maintain the system in the form of kill commands.

This is onboard diagnostic and system signals.

Test topic publishing

```
rostopic pub /drive_current_sense rover_diagnostics/current_sense '{motor1: 1, motor2: 2, motor3: 3, motor4: 4, motor5: 5, motor6: 6}'
```

```
rostopic pub /power_board_sensors power_sensor/PowerMsg '{adc1: 1, adc2: 2, adc3: 3, adc4: 4, latitude: 5, altitude: 6, longitude: 7, orientX: 8, orientY: 9, orientZ: 10, ultrasonic1: 1, ultrasonic2: 2, ultrasonic3: 3, ultrasonic4: 4, temperatureC: 56}'
```