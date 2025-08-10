#ifndef INC_MOCAP_TELEOP_LIB_H_
#define INC_MOCAP_TELEOP_LIB_H_

#include "mocap_teleop_lib.h"
#include "usart.h"

void transmitData(USART_TypeDef *USARTx, uint8_t *data, int length);
uint8_t readForseSensor(void);
void clearBufferAndCounter(void);
void calibration(void);
void set_new_force(void);
void stop_motor(void);
#endif /* INC_MOCAP_TELEOP_LIB_H_ */
