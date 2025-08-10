#include "globals.h"
#include <stdint.h>
/*
* GLOBAL VARIABLES
*/
uint16_t rxBufferCounter = 0;
uint32_t calculated_CRC_32 = 0;
uint32_t recieved_CRC_32 = 0;
uint16_t pwm = 0;
uint8_t flagUart = 0;
uint16_t gripper_position = 666;
uint8_t stop_force_control_flag = 0;

float pwm_calculations = 0;

uint8_t force_sensor_adress = 10;
uint8_t command_to_read_force = 'r';


// Current PWM BORDERS
uint16_t PWM_MIN = 0;
uint16_t PWM_MAX = 400;

/*
* ARRAYS
*/
uint32_t CRC_uint32_calc_buffer[4] = {0};
uint16_t force_sensor_4_value[4] = {0};
uint8_t USB_Tx_buffer[100] = {0};
uint8_t test_usb_buffer[10] = {6};
uint8_t txBuffer[1000] = {0};
uint8_t rxBuffer[1000] = {0};
uint8_t dataForTransmitt[1000] = {0};


