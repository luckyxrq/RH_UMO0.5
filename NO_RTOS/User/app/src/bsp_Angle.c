#include "bsp.h"

/*���ݺ궨�����ѡ��ͬ��ͷ�ļ�*/

#if ANGLE_TYPE == ANGLE_TYPE_YUAN_ZI

#include "bsp_yuan_zi_Angle.c"

#elif ANGLE_TYPE == ANGLE_TYPE_GAN_RUI

#include "bsp_gan_rui_Angle.c"

#endif
