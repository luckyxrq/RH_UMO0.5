#ifndef __BSP_SEARCHCHARGEPILE_H
#define __BSP_SEARCHCHARGEPILE_H


#define _SEARCH_PILE_GO_BACK_PULSE                  (10/(3.14F*70)*1024)

/*
**********************************************************************************************************
                                            直走
**********************************************************************************************************
*/
#define RUN_STRAIGHT_0      ( IR_FL_L && IR_FL_R && IR_FR_L && IR_FR_R    )  /*前面两个都能收到2个窄角信号*/
#define RUN_STRAIGHT_1      ( IR_FL_L && !IR_FL_R && !IR_FR_L && IR_FR_R  )  /*前面两个都能收到与之正对的窄角信号*/


/*
**********************************************************************************************************
                                     前面没有收到任何窄角信号
**********************************************************************************************************
*/
#define F_NO_NARROW_SIGNAL  ( !IR_FL_L && !IR_FL_R && !IR_FR_L && !IR_FR_R  ) /*前面两个都收不到任何窄角信号*/


/*
**********************************************************************************************************
                                    边上收到了广角  需要小一点的大转弯
**********************************************************************************************************
*/
#define ROTATE_CW_LITTLE    ( !(IR_SR_L && IR_SR_R) && IR_SR_M && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW_LITTLE   ( !(IR_SL_L && IR_SL_R) && IR_SL_M && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                    边上收到了2个窄角  需要大转弯
**********************************************************************************************************
*/
#define ROTATE_CW           ( (IR_SR_L && IR_SR_R) && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW          ( (IR_SL_L && IR_SL_R) && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                     已经几乎到了正前面，准备微调
**********************************************************************************************************
*/
#define INCLINATION_GO_L_0    ( IR_FL_L && IR_FL_R && !IR_FR_L && IR_FR_R    )  /*左能收左右，右能收右不能收左*/
#define INCLINATION_GO_L_1    ( IR_FL_R && !IR_FR_L && !IR_FR_R    )            /*左能收右，并且右不能收左也不能收右*/
#define INCLINATION_GO_L_2    ( IR_FL_L && IR_FL_R && !IR_FR_L && !IR_FR_R    ) /*左能同时左右 右既不能左也不能右*/

#define INCLINATION_GO_R_0    ( IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R    )  /*左能左  左不能右 并且 右能左也能右*/
#define INCLINATION_GO_R_1    ( !IR_FL_L && !IR_FL_R && IR_FR_L              )  /*左既不能左也不能右  右能左*/
#define INCLINATION_GO_R_2    ( !IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R   )  /*左既不能左也不能右，右能同时左右*/

/*
**********************************************************************************************************
                  头部收到了广角但是没有收到窄角，两边没有任何信号(科沃斯原地左转，再向右画弧线)
**********************************************************************************************************
*/
#define SL_NO_SIGNAL          (!IR_SL_L && !IR_SL_R && !IR_SL_M)  /*边上的左边 没有任何信号*/
#define SR_NO_SIGNAL          (!IR_SR_L && !IR_SR_R && !IR_SR_M)  /*边上的右边 没有任何信号*/

#define FL_NO_SIGNAL          (!IR_FL_L && !IR_FL_R && !IR_FL_M)  /*前面左边没有任何信号*/
#define FR_NO_SIGNAL          (!IR_FR_L && !IR_FR_R && !IR_FR_M)  /*前面右边没有任何信号*/

#define ALL_NO_SIGNAL         (SL_NO_SIGNAL && SR_NO_SIGNAL && FL_NO_SIGNAL && FR_NO_SIGNAL)

#define ONLY_F_RX_WIDE        ( (IR_FL_M ||  IR_FR_M) && F_NO_NARROW_SIGNAL && SL_NO_SIGNAL && SR_NO_SIGNAL)  


bool bsp_IsCharging(void);
bool bsp_IsChargeDone(void);

void bsp_StartSearchChargePile(void);
void bsp_StopSearchChargePile(void);
void bsp_SearchChargePile(void);
bool bsp_IsTouchChargePile(void);
void bsp_DetectIsTouchChargePile(void);
void bsp_InitChargeIO(void);
#endif

