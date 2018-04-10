#include<stdlib.h>
#include "stm32f4xx_hal.h"
#include "state_handler.h"
#include "commands.h"

/*Define memory areas that are readable*/


enum {
	idle = 0,
	reqReceived,
	receiveAllBytes,
	executeCommand
};

unsigned char writeMemoryByAddress()
{
	static unsigned char addressSize, lengthSize;
	static unsigned short bytesToReceive;
	unsigned char stayInLoop = 0, retVal = 0, indeX;
	do
	{
		if(commandSubState == idle)
		{
			addressSize = RxFrameBuffer[buffIndex].Data[1] & 0xF;
			lengthSize = (RxFrameBuffer[buffIndex].Data[1] & 0xF0) >> 4;
			for(indeX = 0; indeX < lengthSize; indeX++)
			{
				bytesToReceive += (unsigned short)RxFrameBuffer[buffIndex].Data[2+addressSize+indeX] <<((lengthSize-1)-indeX);
			}
			if ((6-(addressSize+lengthSize)) >= bytesToReceive)
			{
				/*Whole command fits in one CAN frame*/
				commandSubState = executeCommand;
				stayInLoop = 1;
			}
			else
			{
				commandSubState = receiveAllBytes;
				bytesToReceive -= (6-(addressSize+lengthSize)) ;
			}
			/*Store now the positive response*/
			TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = RxFrameBuffer[buffIndex].DLC;
			TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x40;
			for (indeX = 1; indeX < TxMsg.DLC; indeX++)
			{
				TxMsg.Data[indeX] = RxFrameBuffer[buffIndex].Data[indeX];
			}
		}
		else if(commandSubState == receiveAllBytes)
		{
			if(bytesToReceive > 0)
			{
				if(RxFrameBuffer[buffIndex].DLC < bytesToReceive)
				{
					bytesToReceive -= RxFrameBuffer[buffIndex].DLC;
				}
				else
				{
					bytesToReceive = 0;
					commandSubState = executeCommand;
					stayInLoop = 1;
				}
			}
		}
		else if (commandSubState == executeCommand)
		{
			/* add the response (TxMsg) to the array of frames*/
			buffIndex++;
			RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
			RxFrameBuffer[buffIndex].DLC = 1;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Dir = 1;
			/*Send positive response*/
			(void)HAL_CAN_Transmit(&hcan2, 1);
			stayInLoop = 0;
			retVal = 1; /*Command is complete, exit from state*/
			commandSubState = 0;
		}
	}while (stayInLoop == 1);
	return retVal;
}

unsigned char securityAccess()
{
	unsigned char retVal = 0, indeX;
	if((RxFrameBuffer[buffIndex].Data[1] % 2) == 1)
	{
		/*Build a test frame*/
		TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
		TxMsg.IDE = CAN_ID_EXT;
		TxMsg.RTR = CAN_RTR_DATA;
		TxMsg.DLC = 4;
		TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x40;
		TxMsg.Data[1] = RxFrameBuffer[buffIndex].Data[1];
		for(indeX = 2; indeX < 4; indeX++)
		{
			TxMsg.Data[indeX] = 0xAA;
		}
		(void)HAL_CAN_Transmit(&hcan2, 1);
		commandSubState = reqReceived;
		/* add the response (TxMsg) to the array of frames*/
		buffIndex++;
		RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
		RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
		RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
		RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
		RxFrameBuffer[buffIndex].Data[2] = TxMsg.Data[2];
		RxFrameBuffer[buffIndex].Data[3] = TxMsg.Data[3];
		RxFrameBuffer[buffIndex].Dir = 1;
	}
	else
	{
		if(commandSubState == reqReceived)
		{
			/*Security access seed was requested before, now send OK to the key (THE KEY IS NOT CHECKED)*/
			TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = 2;
			TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x40;
			TxMsg.Data[1] = RxFrameBuffer[buffIndex].Data[1];
			/* add the response (TxMsg) to the array of frames*/
			buffIndex++;
			RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
			RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
			RxFrameBuffer[buffIndex].Dir = 1;
		}
		else
		{
			/*No Security access seed was requested, so return NOK frame*/
			TxMsg.StdId = TxMsg.ExtId = RxFrameBuffer[buffIndex].ID+1;
			TxMsg.IDE = CAN_ID_EXT;
			TxMsg.RTR = CAN_RTR_DATA;
			TxMsg.DLC = 3;
			TxMsg.Data[0] = RxFrameBuffer[buffIndex].Data[0] | 0x20;
			TxMsg.Data[1] = RxFrameBuffer[buffIndex].Data[1];
			TxMsg.Data[2] = 0x22; /*Return error code CNC - ConditionsNotCorrect*/
			/* add the response (TxMsg) to the array of frames*/
			buffIndex++;
			RxFrameBuffer[buffIndex].ID = TxMsg.StdId;
			RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
			RxFrameBuffer[buffIndex].Data[2] = TxMsg.Data[2];
			RxFrameBuffer[buffIndex].Dir = 1;
		}
		(void)HAL_CAN_Transmit(&hcan2, 1);
		commandSubState = idle;
		retVal = 1;
	}
	return retVal;
}

unsigned char securityAccess_client()
{
	static unsigned char response;
	unsigned char i,value;
	/*Build a security access request tx frame*/
	TxMsg.StdId = secAccId;
	TxMsg.ExtId = secAccId;
	TxMsg.IDE = CAN_ID_EXT;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 2;
	TxMsg.Data[0] = 0x27;
	TxMsg.Data[1] = 0x1;
	/*Add frame to queue, so user can read it later*/
	RxFrameBuffer[++buffIndex].ID = TxMsg.ExtId;
	RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
	RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
	RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
	RxFrameBuffer[buffIndex].Dir = 1;
	(void)HAL_CAN_Transmit(&hcan2, 10); /*Send Sec access request*/
	/*Now wait for the response*/
	response = 1;
	if(HAL_CAN_Receive(&hcan2, 0, 10) == HAL_OK) /*10ms timeout set for the response*/
	{
		/*Add frame to queue, so user can read it later*/
		RxFrameBuffer[++buffIndex].ID = (unsigned short)RxMsg.ExtId;
		RxFrameBuffer[buffIndex].DLC = (unsigned char)RxMsg.DLC;
		for(i=0; i< RxFrameBuffer[buffIndex].DLC; i++)
		{
			RxFrameBuffer[buffIndex].Data[i] = RxMsg.Data[i];
		}
		RxFrameBuffer[buffIndex].Timestamp = CLOCK_VAR;
		RxFrameBuffer[buffIndex].Dir = 0;
		/*Now verify if the response is positive*/
		if(RxMsg.Data[0] == 0x67)
		{
			/*Send the Key*/
			TxMsg.DLC = RxMsg.DLC;
			TxMsg.Data[0] = 0x27;
			TxMsg.Data[1] = 0x2;
			for(value = 2; value < RxMsg.DLC; value++)
			{
				TxMsg.Data[value] = RxMsg.Data[value];
			}
			/*Store the TX frame again*/
			RxFrameBuffer[++buffIndex].ID = TxMsg.ExtId;
			RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
			RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
			RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
			RxFrameBuffer[buffIndex].Dir = 1;
			(void)HAL_CAN_Transmit(&hcan2, 10); /*Send the key*/
			if(HAL_CAN_Receive(&hcan2, 0, 10) == HAL_OK) /*10ms timeout set for the key response*/
			{
				/*Add frame to queue, so user can read it later*/
				RxFrameBuffer[++buffIndex].ID = (unsigned short)RxMsg.ExtId;
				RxFrameBuffer[buffIndex].DLC = (unsigned char)RxMsg.DLC;
				for(i=0; i< RxFrameBuffer[buffIndex].DLC; i++)
				{
					RxFrameBuffer[buffIndex].Data[i] = RxMsg.Data[i];
				}
				RxFrameBuffer[buffIndex].Timestamp = CLOCK_VAR;
				RxFrameBuffer[buffIndex].Dir = 0;
			}
		}
	}
	STATE = Idle;
	return response;
}

void testSeedEntropy(unsigned char no_of_tries)
{
	unsigned char i;
	TxMsg.StdId = secAccId;
	TxMsg.ExtId = secAccId;
	TxMsg.IDE = CAN_ID_EXT;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 2;
	TxMsg.Data[0] = 0x27;
	TxMsg.Data[1] = 0x1;
	for (i = 0; i<no_of_tries; i++)
	{
		/*Add frame to queue, so user can read it later*/
		RxFrameBuffer[++buffIndex].ID = TxMsg.ExtId;
		RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
		RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
		RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
		RxFrameBuffer[buffIndex].Dir = 1;
		(void)HAL_CAN_Transmit(&hcan2, 10); /*Send Sec access request*/
		if(HAL_CAN_Receive(&hcan2, 0, 10) == HAL_OK)
		{
			/*Add frame to queue, so user can read it later*/
			RxFrameBuffer[++buffIndex].ID = (unsigned short)RxMsg.ExtId;
			RxFrameBuffer[buffIndex].DLC = (unsigned char)RxMsg.DLC;
			for(i=0; i< RxFrameBuffer[buffIndex].DLC; i++)
			{
				RxFrameBuffer[buffIndex].Data[i] = RxMsg.Data[i];
			}
			RxFrameBuffer[buffIndex].Timestamp = CLOCK_VAR;
			RxFrameBuffer[buffIndex].Dir = 0;
		}
		else
		{
			break;
		}
	}
}
void DiagReq_func()
{
	unsigned char i;
	TxMsg.StdId = secAccId;
	TxMsg.ExtId = secAccId;
	TxMsg.IDE = CAN_ID_EXT;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.DLC = 2;
	TxMsg.Data[0] = 0x10;
	TxMsg.Data[1] = 0x01;
	/*Add frame to queue, so user can read it later*/
	RxFrameBuffer[++buffIndex].ID = TxMsg.ExtId;
	RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
	RxFrameBuffer[buffIndex].Data[0] = TxMsg.Data[0];
	RxFrameBuffer[buffIndex].Data[1] = TxMsg.Data[1];
	RxFrameBuffer[buffIndex].Dir = 1;
	(void)HAL_CAN_Transmit(&hcan2, 10);
	if(HAL_CAN_Receive(&hcan2, 0, 10) == HAL_OK)
	{
		/*Add frame to queue, so user can read it later*/
		RxFrameBuffer[++buffIndex].ID = (unsigned short)RxMsg.ExtId;
		RxFrameBuffer[buffIndex].DLC = (unsigned char)RxMsg.DLC;
		for(i=0; i< RxFrameBuffer[buffIndex].DLC; i++)
		{
			RxFrameBuffer[buffIndex].Data[i] = RxMsg.Data[i];
		}
		RxFrameBuffer[buffIndex].Timestamp = CLOCK_VAR;
		RxFrameBuffer[buffIndex].Dir = 0;
	}
}

unsigned char SendWriteDataByAddress(unsigned char FId, unsigned char NoB, unsigned short Addr)
{
	unsigned char i, retVal;

	TxMsg.StdId = FId;
	TxMsg.ExtId = FId;
	TxMsg.Data[0] = 0x3D;
	TxMsg.Data[1] = 0x12;
	TxMsg.Data[2] = (Addr&0xFF00) >> 8; /*Address MSB*/
	TxMsg.Data[3] = Addr&0xFF; /*Address LSB*/
	TxMsg.Data[4] = NoB; /*number of bytes to write*/
	if(NoB <= 3)
	{
		TxMsg.DLC = 5+NoB;
		retVal = 1;
		for(i=5; i<(5+NoB); i++)
		{
			TxMsg.Data[i] = 0xAA; /*Fill the rest of the message with dummy bytes*/
		}
		/*Add frame to queue, so user can read it later*/
		RxFrameBuffer[++buffIndex].ID = TxMsg.ExtId;
		RxFrameBuffer[buffIndex].DLC = TxMsg.DLC;
		for(i=0; i<(5+NoB); i++)
		{
			RxFrameBuffer[buffIndex].Data[i] = TxMsg.Data[i];
		}
		RxFrameBuffer[buffIndex].Dir = 1;
		(void)HAL_CAN_Transmit(&hcan2, 10);
		if(HAL_CAN_Receive(&hcan2, 0, 10) == HAL_OK)
		{
			/*Add frame to queue, so user can read it later*/
			RxFrameBuffer[++buffIndex].ID = (unsigned short)RxMsg.ExtId;
			RxFrameBuffer[buffIndex].DLC = (unsigned char)RxMsg.DLC;
			for(i=0; i< RxFrameBuffer[buffIndex].DLC; i++)
			{
				RxFrameBuffer[buffIndex].Data[i] = RxMsg.Data[i];
			}
			RxFrameBuffer[buffIndex].Timestamp = CLOCK_VAR;
			RxFrameBuffer[buffIndex].Dir = 0;
		}
	}
	else
	{
		/*Error, too many bytes in one frame*/
		retVal = 2;
	}
	return retVal;
}

unsigned int SendFuzzingMessages(unsigned short FrameId, unsigned char dataLow, unsigned char dataHigh, unsigned short *nof)
{
	unsigned char i;
	static unsigned int number, retval; //nu e bine!!!!
	static unsigned char StopFuzzing;
	
	if(*nof > 0)
	{
		StopFuzzing = 0;
		*nof -= 1;
	}
	else
	{
		StopFuzzing = 1;
	}
	TxMsg.ExtId = FrameId;
	TxMsg.DLC = 8;
	for(i=0; i<8; i++)
	{
		TxMsg.Data[i] = rand()%((dataHigh + 1) - dataLow) + dataLow;
	}
	if(HAL_CAN_Transmit(&hcan2, 2) == HAL_OK)
	{
		number++;
	}
	if(StopFuzzing == 0)
	{
		retval = 0;
	}
	else
	{
		retval = number;
		number = 0;
	}
	return retval;
}
