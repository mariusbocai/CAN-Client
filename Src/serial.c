#include "state_handler.h"
#include "serial.h"
#include "commands.h"
#include <math.h>

unsigned char serialBuffer[5];
unsigned char commandBuffer[30];
unsigned short parameter[4];
unsigned char bufferIndex, commandInProcess, commandIndex;
static unsigned char newData;

/*
Commands implemented:
-> 1 = send single frame
-> 2 = send repetitive frame each 1 ms
-> 3 = send repetitive frame each 5 ms
-> 4 = send repetitive frame each 10 ms
-> 5 = stop repetitive frames
-> 6 = baudrate 125kbps
-> 7 = baudrate 250 kbps
-> 8 = baudrate 500kbps
-> 9 = baudrate 1Mbps
-> A = read bus for 5 seconds
-> S = send a true securityAccessRequest
-> D = Request DiagnosisSession
NOTE: please do not send a parameter larger than 0xFFFF for the next commands!!!
-> Seed Entropy frame: #E,numberOfSeedRequests# Maximum 80 at a time!
-> Write Data by Address: #W,frameID,noOfbytesToWrite,addressToWrite#
-> Fuzzing frame: #F,frameID,dataLowestValue,dataHighestValue,numberOfFuzzes#
WARN: Fuzzing will not send the actual data via serial (can't see why it would)
WARN2: Fuzzing will send a can frame each 1 ms
*/

void decodeComm(unsigned char *command, unsigned char Index)
{
	unsigned char indeX_local, param_index, local_array_index, local_array[4];
	
	param_index = 0;
	local_array_index = 0;
	for(indeX_local = (Index-1); indeX_local > 0; indeX_local--)
	{
		if(command[indeX_local] != '#')
		{
			if(command[indeX_local] != ',')
			{
				local_array[local_array_index++] = command[indeX_local] - 0x30;
			}
			else
			{
				/*A parameter array has been found, now turn it into a number*/
				unsigned char i;
				for (i = 0; i < local_array_index; i++)
				{
					parameter[param_index] += local_array[i]*(pow(10,i));
				}
				param_index++;
				local_array_index = 0;
			}
		}
		else
		{
			/*exit the loop*/
			break;
		}
	}
}

void processISR(void)
{
	unsigned char getVal, decodeCommand;
	
	decodeCommand = 0;
	getVal = huart4.Instance->DR;
	if((getVal == '#') || (commandInProcess == 1))
	{
		if((getVal == '#') && (commandInProcess == 0))
		{
			commandInProcess = 1;    /*start processing the command*/
		}
		else if((getVal == '#') && (commandInProcess == 1))
		{
			commandInProcess = 0;    /*stop processing the command*/
			decodeCommand = 1;       /*start decoding the command*/
		}
		/*Check if too many bytes have been received. Max limit is 30 per command*/
		if (++commandIndex > 30)
		{
			/*This is an error situation*/
			commandInProcess = 0;
		}
		/*Add command bytes to array*/
		commandBuffer[commandIndex] = getVal;
		if(decodeCommand == 1)
		{
			decodeComm(commandBuffer, commandIndex);
			serialBuffer[bufferIndex++] = commandBuffer[2];
			newData = 1;
			commandIndex = 0;
		}
	}
	else
	{
		serialBuffer[bufferIndex++] = getVal;
		if(bufferIndex == 5)
		{
			bufferIndex = 0;
		}
		newData = 1;
	}
}

void sendByte(unsigned char byte)
{
	huart4.Instance->DR = byte;
	do
	{
	} while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE) == 0);
}

void sendString(unsigned char *pnt, unsigned char numberBytes)
{
	unsigned char i;
	i = 0;
	for(i=0; i<numberBytes; i++)
	{
		sendByte(*(pnt+i));
	}
}

void sendInt(unsigned short data)
{
	unsigned char index, separateChars[10];
	index = 0;
	do
	{
		separateChars[index++] = data%10;
		data /= 10;
	}
	while(data > 0);
	for(;index>0;index--)
	{
		sendByte((unsigned char)(separateChars[index-1]+48));
	}
}

void serialInit()
{
	bufferIndex = 0;
	newData = 0;
	commandInProcess = 0;
	commandIndex = 0;
}

void serialMain()
{
	unsigned char index;
	if(newData == 1)    /*if new command is available, process it*/
	{
		switch (serialBuffer[bufferIndex-1])
		{
			case 49:
				if(STATE == Idle)
				{
					STATE = TransmitSingle;
				}
				break;
			case 50:
				if(STATE == Idle)
				{
					STATE = InitPeriodicFrame;
					PERIODICITY = 1;
				}
				break;
			case 51:
				if(STATE == Idle)
				{
					STATE = InitPeriodicFrame;
					PERIODICITY = 5;
				}
				break;
			case 52:
				if(STATE == Idle)
				{
					STATE = InitPeriodicFrame;
					PERIODICITY = 10;
				}
				break;
			case 53:
				STATE = Idle;
				break;
			case 54:
				BAUDRATE = 12;
				STATE = SetBaud;
				break;
			case 55:
				BAUDRATE = 25;
				STATE = SetBaud;
				break;
			case 56:
				BAUDRATE = 50;
				STATE = SetBaud;
				break;
			case 57:
				BAUDRATE = 100;
				STATE = SetBaud;
				break;
			case 65:
				STATE = ReceiveLimitedTime;
				break;
			case 83:
				STATE = SendSecAcc;
				break;
			case 'W':
				STATE = SendWriteBytes;
				break;
			case 'D':
				STATE = DiagReq;
				break;
			case 'E':
				if ((parameter[0] >0)&&(parameter[0]<=80))
				{
					testSeedEntropy(parameter[0]);
				}
				for(index = 0; index < 4; index++)
				{
					parameter[index] = 0;
				}
				STATE = Idle;
			case 'F':
				STATE = Fuzzing;
			default:
				break;
		}
		newData = 0;
	}
}
