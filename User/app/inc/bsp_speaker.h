#ifndef __BSP_SPEAKER_H
#define __BSP_SPEAKER_H

#include <stdbool.h>

typedef enum
{
	Song1 = 4, /*前面1 2 3没有声音*/
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

//  Song1 开机
//   关机
//   开始清扫
//   暂停清扫
//   返回充电
//   电池电量低请回充
//   电池异常
//   陀螺仪异常
//   尘盒取出
//   尘盒装回
//   请擦拭跳崖传感器
//   右轮机异常
//   左轮机异常
//   滚刷异常
//   边刷异常
//   主机悬空
//   主机被困
//   碰撞开关异常
//   吸尘电机异常
//   滤网阻塞
//   未找到充电桩，请将主机移动到充电桩附近
//   开始充电
//   充电完成
//   回充定位失败，帮助返回充电桩
//   定位失败，重新开始建图
//   重定位成功，继续开始清扫
//   网络已连接
//   网络未连接
//   开始连接网络





void bsp_InitSpeaker(void);
void bsp_SperkerPlay(SongSN sn);
bool bsp_SpeakerIsBusy(void);

#endif

