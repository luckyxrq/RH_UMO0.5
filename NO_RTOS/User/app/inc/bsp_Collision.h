#ifndef __BSP_COLLISION_H
#define __BSP_COLLISION_H

typedef enum
{
	CollisionLeft = 0 ,
	CollisionRight,
	CollisionAll,
	CollisionNone
}Collision;

void bsp_InitCollision(void);      /*初始化碰撞检测引脚*/
Collision bsp_CollisionScan(void); /*返回碰撞结果*/
void bsp_PrintCollision(void);

#endif
