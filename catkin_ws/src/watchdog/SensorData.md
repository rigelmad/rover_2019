# Sensor Data Being Read By The Watchdog

- current sense Drive
- current sense Arm
- sensor data PowerBoard

| Data Source | type | rostopic | unit |
| ---- | ----| --- | ---| 
| current_sense drive_board | 6 int8 | /drive_current_sense | mA |
| current_sense arm_board | 6 int8 | /arm_current_sense | mA |
| PowerMsg power_board | | /power_board_sensors | |

## Current_Sense

TODO need to handle peaks vs continuous

| field | unit |  threshold high | threshold low |
| -- | -- | -- | -- |
| | | | |

## PowerMsg

|field|data|unit| threshold high | threshold low |
|---| ---| ---| -- | -- |
| adc1 | | |
| adc2 | | |
| adc2 | | |
| adc4 | | |
| latitude | | degrees |
| altitude | | |
| longitude | | degrees |
| orientX | | |
| orientY | | |
| orientZ | | |
| ultrasonic1 | | |
| ultrasonic2 | | |
| ultrasonic3 | | | 
| ultrasonic4 | | | 
| temperatureC | | Celsius |


TODO Need to Include Threshold data for high and low for warnings
