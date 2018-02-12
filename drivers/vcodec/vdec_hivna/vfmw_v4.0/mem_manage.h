#ifndef _VFMW_MEM_MANAGE_HEAD_
#define _VFMW_MEM_MANAGE_HEAD_

#include "vfmw.h"

#define MEM_MAN_ERR  -1
#define MEM_MAN_OK    0

typedef struct {
	unsigned int PhyAddr;
	unsigned int Length;
	int IsSecMem;
	unsigned char *VirAddr;
} MEM_RECORD_S;

void MEM_InitMemManager(void);

int MEM_AddMemRecord(unsigned int PhyAddr, void *VirAddr, unsigned int Length);

int MEM_DelMemRecord(unsigned int PhyAddr, void *VirAddr, unsigned int Length);

void *MEM_Phy2Vir(unsigned int PhyAddr);

unsigned int MEM_Vir2Phy(unsigned char *VirAddr);

void MEM_WritePhyWord(unsigned int PhyAddr, unsigned int Data32);

unsigned int MEM_ReadPhyWord(unsigned int PhyAddr);

int MEM_MapRegisterAddr(unsigned int RegStartPhyAddr, unsigned int RegByteLen, MEM_RECORD_S *pMemRecord);

void MEM_UnmapRegisterAddr(unsigned int PhyAddr, unsigned char *VirAddr, unsigned int Size);

#endif
