#ifndef __BSP_ANGLE_H
#define __BSP_ANGLE_H


#define ANGLE_TYPE_YUAN_ZI      0
#define ANGLE_TYPE_GAN_RUI      1

#define ANGLE_TYPE      ANGLE_TYPE_GAN_RUI




#if ANGLE_TYPE == ANGLE_TYPE_YUAN_ZI

#include "bsp_yuan_zi_Angle.h"

#elif ANGLE_TYPE == ANGLE_TYPE_GAN_RUI

#include "bsp_gan_rui_Angle.h"

#endif



#endif

