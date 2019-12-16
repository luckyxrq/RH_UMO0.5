#ifndef __BSP_COLLISION_H
#define __BSP_COLLISION_H

typedef enum
{
	CollisionLeft = 0 ,
	CollisionRight,
	CollisionAll,
	CollisionNone
}Collision;

void bsp_InitCollision(void);      /*��ʼ����ײ�������*/
Collision bsp_CollisionScan(void); /*������ײ���*/
void bsp_PrintCollision(void);

#endif
