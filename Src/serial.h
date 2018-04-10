
#ifndef uart_incl
#define uart_incl

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart4;

extern void serialInit(void);
extern void serialMain(void);
extern void processISR(void);
extern void sendByte(unsigned char byte);
extern void sendString(unsigned char *pnt, unsigned char numberBytes);
extern void sendInt(unsigned short);

#endif
