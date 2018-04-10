
#ifndef commands_h_incl_
#define commands_h_incl_

extern unsigned char commandSubState;
extern unsigned char writeMemoryByAddress(void);
extern unsigned char securityAccess(void);
extern unsigned char securityAccess_client(void);
extern void testSeedEntropy(unsigned char x);
extern void DiagReq_func(void);
extern unsigned char SendWriteDataByAddress(unsigned char FId, unsigned char NoB, unsigned short Addr);
extern unsigned int SendFuzzingMessages(unsigned short FrameId, unsigned char dataLow, unsigned char dataHigh, unsigned short *nof);

extern const unsigned char secAccId;

#endif
