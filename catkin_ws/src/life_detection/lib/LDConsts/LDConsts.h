// ADDRESSES
#define U4_ADDR 1 // Boils down to addr 0x21 (0d31)
#define U3_ADDR 2 // Boils down to I2C addr 0x22 (0d32)
#define NUM_ADC_CHANS 10

#define CORE_AVR_ADDR 0x45 // (Hex for 69 ;)
#define SENSOR_AVR_ADDR 0x02
#define MOTOR_AVR_ADDR 0x03
#define ACTUATOR_AVR_ADDR 0x04

#define EXO_DEFAULT_ADDR 0x67
#define PUMP1_ADDR 0x10
#define PUMP2_ADDR 0x11
#define PUMP3_ADDR 0x12
#define PUMP4_ADDR 0x13

// MAX ADC Pins
#define ADC_WATER_LEVEL 0
#define ADC_AMB_TEMP 1
#define ADC_TANK_TEMP 2
#define ADC_PELT_TEMP 3
#define ADC_PHOT1 4
#define ADC_PHOT2 5
#define ADC_PHOT3 6
#define ADC_PHOT4 7
#define ADC_PHOT5 8
#define ADC_PHOT6 9

/******* I/O Expanders **********/
// highByte(pin) -> I/O Selector (U3 = 0x03 or U4 = 0x04)
// lowByte(pin) -> Pin selector (0x00 - 0x0F == 0 - 15 == A0 - B7)
#define NUM_IO_PINS 30
// I/O Exapnder U4 pins
#define IO_SOL1 0x0400
#define IO_SOL2 0x0401
#define IO_SOL3 0x0402
#define IO_SOL4 0x0403
#define IO_SOL5 0x0404
#define IO_SOL6 0x0405
#define IO_SOL7 0x0406
#define IO_SOL8 0x0407
#define IO_SOL9 0x0408 // Start Bank B - B0 (under assumption that since A7 = 7, B0 = 8)
#define IO_SOL10 0x0409 // B1
#define IO_BV1 0x040A
#define IO_BV2 0x040B
#define IO_BV3 0x040C // B4
#define IO_PUMP1 0x040D
#define IO_PUMP2 0x040E
#define IO_PUMP3 0x040F // B7

// I/O Expander U3 pins
#define IO_SOL11 0x0300
#define IO_SOL12 0x0301
#define IO_SOL13 0x0302
#define IO_SOL14 0x0303
#define IO_SOL15 0x0304
#define IO_SOL16 0x0305
#define IO_SOL17 0x0306
#define IO_SOL18 0x0307
#define IO_SOL19 0x0308
#define IO_SOL20 0x0309
#define IO_PUMP4_FWD 0x030A
#define IO_PUMP4_REV 0x030B
#define IO_PUMP4 0x030C
// Schematic skips GPB5, idk why
#define IO_WHITE_LEDS 0x030E

// Custom I2C Defines
#define ENCODER_REQ   1
#define TARGET_CHANGE 2
#define PELTIER_TOGGLE_BIT 3
