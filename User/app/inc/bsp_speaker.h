#ifndef __BSP_SPEAKER_H
#define __BSP_SPEAKER_H

#include <stdbool.h>

typedef enum
{
	Song1 = 0,
	Song2,
	Song3,
	Song4,
	Song5,
	Song6,
	Song7,
	Song8,
	Song9,
	Song10,
	Song11,
	Song12,
	Song13,
	Song14,
	Song15,
	Song16,
	Song17,
	Song18,
	Song19,
	Song20,
	Song21,
	Song22,
	Song23,
	Song24,
	Song25,
	Song26,
	Song27,
	Song28,
	Song29,
	Song30,
}SongSN;

void bsp_InitSpeaker(void);
void bsp_SperkerPlay(SongSN sn);
bool bsp_SpeakerIsBusy(void);

#endif

