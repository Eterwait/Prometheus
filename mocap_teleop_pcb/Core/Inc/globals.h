#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_
#include <stdint.h>
/*
* GLOBAL VARIABLES
*/

extern uint16_t rxBufferCounter;
extern uint32_t calculated_CRC_32;
extern uint32_t recieved_CRC_32;
extern uint16_t pwm;
extern uint8_t flagUart;
extern uint16_t gripper_position;
extern uint8_t force_sensor_adress;
extern uint8_t command_to_read_force;
extern uint8_t stop_force_control_flag;

extern float pwm_calculations;

// Current PWM BORDERS
extern uint16_t PWM_MIN;
extern uint16_t PWM_MAX;

/*
* ARRAYS
*/
extern uint32_t CRC_uint32_calc_buffer[4];
extern uint16_t force_sensor_4_value[4];
extern uint8_t USB_Tx_buffer[100];
extern uint8_t test_usb_buffer[10];
extern uint8_t txBuffer[1000];
extern uint8_t rxBuffer[1000];
extern uint8_t dataForTransmitt[1000];

#endif /* INC_GLOBALS_H_ */
