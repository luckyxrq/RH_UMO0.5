#include "bsp.h"



#define PPPPP   0xea,0x0a,0x3c,0x87,0x99,0x8c,0x43,0xaa

#define KKKKK   0xe2,0xe1,0x31,0x8a,0xf1,0x34,0x08,0xef,0x7e,0x73,0x70,0xd6,0x97,0x5b,0xe1,0x85 

#define ZZZZZ0  0xe3,0x27,0xc2,0x74,0xe1,0x48,0x4a,0xb0,0x13,0xc7,0x34,0xd0,0x4a,0x44,0x32,0xd7 

#define ZZZZZ1  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 

#define ZZZZZ2  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 

#define ZZZZZ3  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 


//------------------------------------------------------------------------------
//  Host CPU Random Number Generation
//------------------------------------------------------------------------------
void GetSoftRandom(unsigned char *random, unsigned short len)
{
  unsigned short i;
  // Strongly recommended the seed involved by system time !!!!!
  //srand((unsigned int)time(NULL) + srand_cnt++); 
  srand(srand_cnt++); 
  for (i=0; i<len; i++) random[i] = rand() % 256;
}


//------------------------------------------------------------------------------
//  PIN and Host Authentication
//------------------------------------------------------------------------------
unsigned char AuthenticationTest(void)
{
   unsigned char rv;
   unsigned char random[32];
   unsigned char tmpBuf1[20] = { PPPPP };
   unsigned char tmpBuf2[20] = { KKKKK };   

   // Wakeup and Reset DX8
   rv = DX8_Reset();
   if (rv) return rv;

   // PIN Authentication
   GetSoftRandom(random,32); // Generate random for verify PIN
   rv = DX8_VerifyPin(random,tmpBuf1);
   if (rv) return rv;

   // Host Authentication
   memset(tmpBuf1,0x00,20);
   GetSoftRandom(random,32);
   rv = DX8_HostAuth(random,32,tmpBuf1);
   if (rv) return rv;
   Lib_HostAuth(random,32,tmpBuf2,tmpBuf2);
   rv = memcmp(tmpBuf1,tmpBuf2,20);
   if (rv) return rv;

   // DX8 Sleep to save power
   rv = DX8_Sleep(); 
   if (rv) return rv;

   return 0;
}
