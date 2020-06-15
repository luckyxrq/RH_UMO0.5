#ifndef __BSP_SEARCHCHARGEPILE_H
#define __BSP_SEARCHCHARGEPILE_H


#define _SEARCH_PILE_GO_BACK_PULSE                  (10/(3.14F*70)*1024)

/*
**********************************************************************************************************
                                            ֱ��
**********************************************************************************************************
*/
#define RUN_STRAIGHT_0      ( IR_FL_L && IR_FL_R && IR_FR_L && IR_FR_R    )  /*ǰ�����������յ�2��խ���ź�*/
#define RUN_STRAIGHT_1      ( IR_FL_L && !IR_FL_R && !IR_FR_L && IR_FR_R  )  /*ǰ�����������յ���֮���Ե�խ���ź�*/


/*
**********************************************************************************************************
                                     ǰ��û���յ��κ�խ���ź�
**********************************************************************************************************
*/
#define F_NO_NARROW_SIGNAL  ( !IR_FL_L && !IR_FL_R && !IR_FR_L && !IR_FR_R  ) /*ǰ���������ղ����κ�խ���ź�*/


/*
**********************************************************************************************************
                                    �����յ��˹��  ��ҪСһ��Ĵ�ת��
**********************************************************************************************************
*/
#define ROTATE_CW_LITTLE    ( !(IR_SR_L && IR_SR_R) && IR_SR_M && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW_LITTLE   ( !(IR_SL_L && IR_SL_R) && IR_SL_M && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                    �����յ���2��խ��  ��Ҫ��ת��
**********************************************************************************************************
*/
#define ROTATE_CW           ( (IR_SR_L && IR_SR_R) && F_NO_NARROW_SIGNAL)
#define ROTATE_CCW          ( (IR_SL_L && IR_SL_R) && F_NO_NARROW_SIGNAL)

/*
**********************************************************************************************************
                                     �Ѿ�����������ǰ�棬׼��΢��
**********************************************************************************************************
*/
#define INCLINATION_GO_L_0    ( IR_FL_L && IR_FL_R && !IR_FR_L && IR_FR_R    )  /*���������ң��������Ҳ�������*/
#define INCLINATION_GO_L_1    ( IR_FL_R && !IR_FR_L && !IR_FR_R    )            /*�������ң������Ҳ�������Ҳ��������*/
#define INCLINATION_GO_L_2    ( IR_FL_L && IR_FL_R && !IR_FR_L && !IR_FR_R    ) /*����ͬʱ���� �ҼȲ�����Ҳ������*/

#define INCLINATION_GO_R_0    ( IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R    )  /*������  ������ ���� ������Ҳ����*/
#define INCLINATION_GO_R_1    ( !IR_FL_L && !IR_FL_R && IR_FR_L              )  /*��Ȳ�����Ҳ������  ������*/
#define INCLINATION_GO_R_2    ( !IR_FL_L && !IR_FL_R && IR_FR_L && IR_FR_R   )  /*��Ȳ�����Ҳ�����ң�����ͬʱ����*/

/*
**********************************************************************************************************
                  ͷ���յ��˹�ǵ���û���յ�խ�ǣ�����û���κ��ź�(����˹ԭ����ת�������һ�����)
**********************************************************************************************************
*/
#define SL_NO_SIGNAL          (!IR_SL_L && !IR_SL_R && !IR_SL_M)  /*���ϵ���� û���κ��ź�*/
#define SR_NO_SIGNAL          (!IR_SR_L && !IR_SR_R && !IR_SR_M)  /*���ϵ��ұ� û���κ��ź�*/

#define FL_NO_SIGNAL          (!IR_FL_L && !IR_FL_R && !IR_FL_M)  /*ǰ�����û���κ��ź�*/
#define FR_NO_SIGNAL          (!IR_FR_L && !IR_FR_R && !IR_FR_M)  /*ǰ���ұ�û���κ��ź�*/

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

