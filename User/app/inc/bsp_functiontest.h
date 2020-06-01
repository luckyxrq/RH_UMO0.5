#ifndef __BSP_FUNCTIONTEST_H
#define __BSP_FUNCTIONTEST_H

#define FTS_ReadyGo               0
#define FTS_RightEdgewiseRun      1
#define FTS_ErLangStartTurnAround 2
#define FTS_ErlangGostraight      3
#define FTS_ErLangStopTurnAround  4
#define FTS_CliffGosraight        5
#define FTS_CliffBackward         6
#define FTS_CliffStartTurnAround  7
#define FTS_LeftEdgewiseRun       8
#define FTS_SearchChargePile      9
#define FTS_AllComplete           10


typedef struct
{
	/*×´Ì¬»ú*/
	volatile uint8_t action ;
	volatile bool isRunning ;
	volatile uint32_t delay ;
}FunctionTest;

void bsp_StartFunctionTest(void);
void bsp_StopFunctionTest(void);
void bsp_FunctionTestUpdate(void);

#endif
