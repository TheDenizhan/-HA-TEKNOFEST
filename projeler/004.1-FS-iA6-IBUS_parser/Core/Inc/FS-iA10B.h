#ifndef __FSiA10B_H
#define __FSiA10B_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct _FSiA10B_iBus
{
	unsigned short RH;
	unsigned short RV;
	unsigned short LV;
	unsigned short LH;
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
}FSiA10B_iBus;

extern FSiA10B_iBus iBus;
unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char* data, FSiA10B_iBus* iBus);
void FSiA10B_UART5_Initialization(void);


#ifdef __cplusplus
}
#endif

#endif /* __FSIA10B_H */
