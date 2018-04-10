
#ifndef statehandlincl
#define statehandlincl

#include "stm32f4xx_hal.h"

enum {
	Init=0,
	DeInit,
	TransmitSingle,
	InitPeriodicFrame,
	PeriodicFrame,
	ReceiveLimitedTime,
	SetBaud,
	PosRespSecAcc,
	SendSecAcc,
	SendWriteBytes,
	DiagReq,
	Fuzzing,
	Idle
};

typedef struct {
	unsigned short ID;
	unsigned char DLC;
	unsigned char Data[8];
	unsigned char Dir;
	unsigned int Timestamp;
}canFrameStruct;

extern CAN_HandleTypeDef hcan2;
extern CanTxMsgTypeDef TxMsg;
extern CanRxMsgTypeDef RxMsg;
extern volatile unsigned char startLoop;
extern unsigned char STATE;
extern unsigned char PERIODICITY;
extern unsigned char BAUDRATE;
extern unsigned int CLOCK_VAR;
extern unsigned char buffIndex;
extern canFrameStruct RxFrameBuffer[200];
extern void stateMachineMain(void);

#endif
