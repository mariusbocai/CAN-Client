#include "state_handler.h"
#include "serial.h"
#include "commands.h"

static unsigned char sentOk[8] = {0x53, 0x65, 0x6E, 0x64, 0x20, 0x4F, 0x4B, 0xA};
static unsigned char sentErr[6] = {0x45, 0x72, 0x72, 0x6F, 0x72, 0xA};
static unsigned char sentInitErr[11] = {0x49, 0x6E, 0x69, 0x74, 0x20, 0x45, 0x72, 0x72, 0x6F, 0x72, 0xA};
static unsigned char sentTO[8] = {0x54, 0x69, 0x6D, 0x65, 0x6F, 0x75, 0x74, 0xA};
static unsigned char sentBusy[5] = {0x42, 0x75, 0x73, 0x79, 0xA};
static unsigned char sentInitOk[8] = {0x49, 0x6E, 0x69, 0x74, 0x20, 0x4F, 0x4B, 0xA};
static unsigned char sentHello[6] = {0x48, 0x65, 0x6C, 0x6C, 0x6F, 0xA};
static unsigned char sentVersion[13] = {0x43, 0x41, 0x4E, 0x74, 0x65, 0x73, 0x74, 0x20, 0x76, 0x31, 0x2E, 0x34, 0xA};
static unsigned char sentPeriodic[14] = {0x53, 0x65, 0x6E, 0x64, 0x20, 0x70, 0x65, 0x72, 0x69, 0x6F, 0x64, 0x69, 0x63, 0xA};
static unsigned char sentBaud[5] = {0x42, 0x61, 0x75, 0x64, 0x20};
static unsigned char sent125[4] = {0x31, 0x32, 0x35, 0xA};
static unsigned char sent250[4] = {0x32, 0x35, 0x30, 0xA};
static unsigned char sent500[4] = {0x35, 0x30, 0x30, 0xA};
static unsigned char sent1000[5] = {0x31, 0x30, 0x30, 0x30,0xA};
static unsigned char Error_too_many_bytes[] = "Error too many bytes";
static unsigned char Command_complete[] = "Command complete";
unsigned char PERIODICITY;
static unsigned short RECEIVE_CNT;
unsigned char PERIOD_COUNT;
unsigned char BAUDRATE;
unsigned char RxArray[15];

unsigned char prevSTATE;
unsigned char commandSubState;
unsigned char changeToPrev;
unsigned char lockState;

extern unsigned short parameter[4];

/*Build an array of Rx frames so we can store the incoming data*/
canFrameStruct RxFrameBuffer[200];
unsigned char buffIndex;

/*SELECT CAN ID and MASK HERE
=============================
*/
#define canMASK 0x0
#define canID 0x0
#define txID 0x5
#define txDLC 2
#define txPayload 0xAA

/*SELECT A SECURITY ACCESS FRAME ID*/
const unsigned char secAccId = 0x23;

CAN_FilterConfTypeDef Filter1Config;

unsigned char STATE;

void stateMachineMain()
{
	unsigned char ret, value,index;
	switch(STATE)
	{
		case (Init):
			buffIndex = 0;
			lockState = 0;
			commandSubState = 0;
			serialInit();
			sendString(sentHello, 6);
			sendString(sentVersion, 13);
			value = HAL_CAN_Init(&hcan2);
			if(value == HAL_OK)
			{
				sendString(sentInitOk, 8); /*send "Init ok" to signal a succesfull init*/
			}
			else
			{
				sendString(sentInitErr, 11);
			}
			STATE = Idle;
			/*Build a test frame*/
			TxMsg.StdId = txID;
			TxMsg.ExtId = txID;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = txDLC;
			for(value = 0; value < txDLC; value++)
			{
				TxMsg.Data[value] = txPayload;
			}
			hcan2.pTxMsg = &TxMsg;
		
			/*Configure filters*/
			Filter1Config.FilterMode = CAN_FILTERMODE_IDMASK;
			Filter1Config.FilterScale = CAN_FILTERSCALE_16BIT;
			Filter1Config.FilterIdHigh = canID; /*Accept this ID on FIFO0*/
			Filter1Config.FilterIdLow = canMASK; /*Accept this MASK on FIFO0*/
			Filter1Config.FilterFIFOAssignment = 0;
			Filter1Config.FilterNumber = 20;
			Filter1Config.FilterActivation = ENABLE;
			Filter1Config.BankNumber = 0;
			
			hcan2.pRxMsg = &RxMsg;
			HAL_CAN_ConfigFilter(&hcan2, &Filter1Config);
			
			break;
		case (DeInit):
			break;
		case (TransmitSingle):
			/*Send frame*/
			ret = HAL_CAN_Transmit(&hcan2, 10);
			if(ret == HAL_OK)
			{
				sendString(sentOk, 8); /*send "t" to signal a tx frame*/
			}
			else if(ret == HAL_ERROR)
			{
				sendString(sentErr, 6);
			}
			else if(ret == HAL_TIMEOUT)
			{
				sendString(sentTO, 8);
			}
			else if(ret == HAL_BUSY)
			{
				sendString(sentBusy, 5);
			}
			if(changeToPrev == 1)
			{
				changeToPrev = 0;
				STATE = prevSTATE;
				prevSTATE = Idle;
			}
			else
			{
				STATE = Idle;
			}
			break;
		case (InitPeriodicFrame):
			sendString(sentPeriodic, 14);
			STATE = PeriodicFrame;
			break;
		case (PeriodicFrame):
			if(PERIOD_COUNT == 0)
			{
				PERIOD_COUNT = PERIODICITY;
				ret = HAL_CAN_Transmit(&hcan2, 100);
			}
			PERIOD_COUNT--;
			break;
		case (ReceiveLimitedTime):
			do
			{
				unsigned char i;
				if((HAL_CAN_Receive(&hcan2, 0, 0) == HAL_OK)&&(buffIndex<200))
				{
					/*Store frame in RAM and increment counter*/
					RxFrameBuffer[++buffIndex].ID = (unsigned short)RxMsg.StdId;
					RxFrameBuffer[buffIndex].DLC = (unsigned char)RxMsg.DLC;
					for(i=0; i< RxFrameBuffer[buffIndex].DLC; i++)
					{
						RxFrameBuffer[buffIndex].Data[i] = RxMsg.Data[i];
					}
					RxFrameBuffer[buffIndex].Timestamp = CLOCK_VAR;
				
					/*========================================*/
					/*Check if a Security Access was requested*/
					/*========================================*/
					if((RxFrameBuffer[buffIndex].Data[0] == 0x27) || (lockState == 1))
					{
						lockState = 1;
						if(securityAccess() == 1)
						{
							lockState = 0; /*Release the lock on the state because the command is finished*/
						}
					}
					/*==================================*/
					/*End of the Security Access handler*/
					/*==================================*/
					
					/*=============================================*/
					/*Check if a WriteMemoryByAddress was requested*/
					/*=============================================*/
					if((RxFrameBuffer[buffIndex].Data[0] == 0x3D) || (lockState == 2))
					{
						lockState = 2;
						if(writeMemoryByAddress() == 1)
						{
							lockState = 0; /*Release the lock on the state because the command is finished*/
						}
					}
					/*=======================================*/
					/*End of the WriteMemoryByAddress handler*/
					/*=======================================*/
				}
			}while (startLoop == 0);
			if((RECEIVE_CNT--) == 0) STATE = Idle;
			break;
		case (SetBaud):
			if(BAUDRATE == 12)
			{
				sendString(sentBaud, 5);
				sendString(sent125, 4);
				hcan2.Init.Prescaler = 16;
				hcan2.Init.SJW = CAN_SJW_1TQ;
				hcan2.Init.BS1 = CAN_BS1_12TQ;
				hcan2.Init.BS2 = CAN_BS2_8TQ;
			}
			else if(BAUDRATE == 25)
			{
				sendString(sentBaud, 5);
				sendString(sent250, 4);
				hcan2.Init.Prescaler = 8;
				hcan2.Init.SJW = CAN_SJW_1TQ;
				hcan2.Init.BS1 = CAN_BS1_12TQ;
				hcan2.Init.BS2 = CAN_BS2_8TQ;
			}
			else if(BAUDRATE == 50)
			{
				sendString(sentBaud, 5);
				sendString(sent500, 4);
				hcan2.Init.Prescaler = 4;
				hcan2.Init.SJW = CAN_SJW_1TQ;
				hcan2.Init.BS1 = CAN_BS1_12TQ;
				hcan2.Init.BS2 = CAN_BS2_8TQ;
			}
			else if(BAUDRATE == 100)
			{
				sendString(sentBaud, 5);
				sendString(sent1000, 5);
				hcan2.Init.Prescaler = 2;
				hcan2.Init.SJW = CAN_SJW_1TQ;
				hcan2.Init.BS1 = CAN_BS1_12TQ;
				hcan2.Init.BS2 = CAN_BS2_8TQ;
			}
			STATE = Init;
			break;
		/*
			Send a Security Access request
		*/
		case (SendSecAcc):
			(void) securityAccess_client();
			break;
		case (SendWriteBytes):
			if(SendWriteDataByAddress(parameter[2], (unsigned char)parameter[1], (unsigned char)parameter[0]) == 1)
			{
				sendString(Command_complete, sizeof(Command_complete));
				sendByte(0xA);
			}
			else
			{
				sendString(Error_too_many_bytes, sizeof(Error_too_many_bytes));
				sendByte(0xA);
			}
			for(index = 0; index < 4; index++)
			{
				parameter[index] = 0;
			}
			STATE = Idle;
			break;
		case (DiagReq):
			DiagReq_func();
			STATE = Idle;
			break;
		case (Fuzzing):
		{
			unsigned int temp;
			temp = SendFuzzingMessages(parameter[3], (unsigned char)parameter[2], (unsigned char)parameter[1], &parameter[0]);
			if(temp != 0)
			{
				sendString(Command_complete, sizeof(Command_complete));
				sendByte(0xA);
				sendInt((unsigned short)temp);
				sendByte(0xA);
				for(index = 0; index < 4; index++)
				{
					parameter[index] = 0;
				}
				STATE = Idle;
			}
		}
			break;
		case (Idle):
			RECEIVE_CNT = 5000;
			if(buffIndex != 0)
			{
				unsigned char i;
				/*A frame has been received, now send it via serial*/
				if(RxFrameBuffer[buffIndex].Dir == 1)
				{
					sendByte(0x2B);
					RxFrameBuffer[buffIndex].Dir = 0;
				}
				sendInt(RxFrameBuffer[buffIndex].ID);
				sendByte(0x20);
				for(i=0; i<RxFrameBuffer[buffIndex].DLC; i++)
				{
					sendInt(RxFrameBuffer[buffIndex].Data[i]);
					sendByte(0x20); //space between payload bytes
				}
				sendByte(0xA); //newline
				buffIndex--;
			}
			break;
	}
}
