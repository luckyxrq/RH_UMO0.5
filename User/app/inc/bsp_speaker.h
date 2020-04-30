#ifndef __BSP_SPEAKER_H
#define __BSP_SPEAKER_H

#include <stdbool.h>

typedef enum
{
	Song1 = 4, /*开机   前面1 2 3没有声音*/
	Song2,     /*关机*/
	Song3,     /*开始清扫*/
	Song4,     /*暂停清扫*/
	Song5,     /*返回充电*/
	Song6,     /*电池电量低，请回充*/
	Song7,     /*电池异常*/
	Song8,     /*陀螺仪异常*/
	Song9,     /*尘盒取出*/
	Song10,    /*尘盒装回*/
	Song11,    /*请擦拭跳崖传感器*/
	Song12,    /*右轮机异常*/
	Song13,    /*左轮机异常*/
	Song14,    /*滚刷异常*/
	Song15,    /*边刷异常*/
	Song16,    /*主机悬空*/
	Song17,    /*主机被困*/
	Song18,    /*碰撞开关异常*/
	Song19,    /*吸尘电机异常*/
	Song20,    /*滤网阻塞*/
	Song21,    /*未找到充电桩，请将主机移动到充电桩附近*/
	Song22,    /*开始充电*/
	Song23,    /*充电完成*/
	Song24,    /*回充点位失败，帮助返回充电桩*/
	Song25,    /*定位失败，重先开始建图*/
	Song26,    /*重定位成功，继续开始清扫*/
	Song27,    /*网络已连接*/
	Song28,    /*网络未连接*/
	Song29,    /*开始连接网络*/
	
	/*新增加的*/
	Song30,    /*暂停回充*/
	Song31,    /*进入休眠模式*/
	Song32,    /*退出休眠模式*/
	Song33,    /*定点清扫*/
	Song34,    /*沿边清扫*/
	Song35,    /*弓字清扫*/
	
	/*还需要新增的（待新增）
	陀螺仪异常
	*/
	
	
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

