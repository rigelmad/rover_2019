#define MOTOR_AVR_ADDR 0x02
#define GPS_AVR_ADDR 0x03
#define LED_AVR_ADDR 0x04
#define TEMP_SENSE_ADDR 0x19
#define CURRENT_SENSE_ADDR 0x44

// GPS Data Enums
#define FLAG_LAT 0
#define FLAG_LONG 1
#define FLAG_ALT 2
#define FLAG_SATS 3

//LED Defines bc I'm a neatfreak
#define RED 0
#define GREEN 1
#define BLUE 2

// Custom I2C Defines
#define ENCODER_REQ   1
#define TARGET_CHANGE 2
#define PELTIER_TOGGLE_BIT 3
