#include "bsp.h"


#define PPPPP   0x0b,0x50,0xb7,0xa7,0x3d,0x1f,0x62,0xe5 

#define KKKKK   0x03,0x32,0xd0,0x27,0xc3,0x0c,0x1e,0x7b,0x3e,0x22,0x75,0xeb,0x7c,0x1d,0x48,0x9b 

#define ZZZZZ0  0x0d,0x7f,0x7e,0xaa,0x68,0x59,0x3c,0xee,0x3c,0xa2,0x0f,0x65,0xc0,0x27,0x58,0xf9 

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
