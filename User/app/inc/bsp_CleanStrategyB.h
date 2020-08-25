#ifndef __BSP_CLEANSTRATEGYB_H
#define __BSP_CLEANSTRATEGYB_H

#include <stdbool.h>

#if 0

#define RIGHTRUNNING_WORK_SETP        0x01
#define LEFTRUNNING_WORK_SETP         0x02
#define RIGHTRETURN_ORIGIN_WORK_SETP  0x03
#define LEFTRETURN_ORIGIN_WORK_SETP   0x04
#define ALL_FINSHED_WORK_SETP         0x05


//**********return origin define*************
#define GOSTR_RETURN_ORIGIN_STEP                                                               0x001
#define DIR_Y_MORE_POSITIVE_200                                                                0x002
#define DIR_Y_LESS_NEGATIVE_200                                                                0x003
#define DIR_X_MORE_POSITIVE_200                                                                0x004
#define DIR_X_LESS_NEGATIVE_200                                                                0x005

#define TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN                                        0x006
#define TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN                                        0x007
#define TURN_CLOCK_TARGET_YAW_180_RETURN_ORIGIN                                                0x008
#define TURN_CLOCK_TARGET_YAW_5_RETURN_ORIGIN                                                  0x009

#define COLLISION_Y_POSITIVE_90_RETURN_ORIGIN                                                  0x00A
#define COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN                                                  0x00B
#define COLLISION_X_POSITIVE_180_RETURN_ORIGIN                                                 0x00C
#define COLLISION_X_NEGATIVE_5_RETURN_ORIGIN                                                   0x00D

#define TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN                                                 0x00E
#define TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN                                                 0x00F
#define TURN_CLOCK_X_POSITIVE_180_RETURN_ORIGIN                                                0x010
#define TURN_CLOCK_X_NEGATIVE_5_RETURN_ORIGIN                                                  0x011

#define TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN                                       0x012
//#define TURN_CLOCK_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN                                       0x013
#define TURN_CLOCK_COLLISION_X_POSITIVE_180_RETURN_ORIGIN                                      0x014
#define TURN_CLOCK_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN                                        0x015

#define BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN                                           0x016
#define BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN                                           0x017
#define BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN                                          0x018
#define BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN                                            0x019

#define LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN                                      0x01A
#define LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN                                      0x01B
#define LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN                                     0x01C
#define LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN                                       0x01D

#define COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN                            0x01E
#define COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN                            0x01F
#define COLLISION_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN                           0x020
#define COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN                             0x021

#define Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN                               0x022
#define Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN                               0x023
#define X_MORE_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN                              0x024
#define X_MORE_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN                                0x025

#define TURN_CLOCK_LOOP_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN                                  0x026
#define TURN_CLOCK_LOOP_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN                                  0x027
#define TURN_CLOCK_LOOP_COLLISION_X_POSITIVE_180_RETURN_ORIGIN                                 0x028
#define TURN_CLOCK_LOOP_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN                                   0x029

#define TURN_CLOCK_DIR_Y_MORE_POSITIVE_200                                                     0x02A
#define TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200                                                     0x02B
#define TURN_CLOCK_DIR_X_MORE_POSITIVE_200                                                     0x02C
#define TURN_CLOCK_DIR_X_LESS_NEGATIVE_200                                                     0x02D

//*************stuck*************
#define STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP                                                   0x030
#define STUCK_FORWARD_BOUNDARY_STATUS                                                          0x031
#define TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS                                   0x032
#define GO_STUCK_FORWARD_BOUNDARY_STATUS                                                       0x033
#define COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS                         0x034
#define COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS                                             0x035
#define GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS                                              0x036
#define TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS                                   0x037
#define COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS                         0x038
#define ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                                              0x039
#define COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS                                    0x03A
#define COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                                    0x03B
#define COLLISION_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS                          0x03C
#define COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                                      0x03D
#define TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                               0x03E
#define COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                     0x03F

//*************A* return origin*************
#define ASTAR_MOTION_GOSTR_RETURN                                                              0x040
#define PLAN_ASTAR_MOTION_GOSTR_RETURN                                                         0x041
#define START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                                   0x042
#define COLLISION_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                         0x043
#define TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                        0x044
#define DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                            0x045
#define FINISH_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                            0x046
#define COLLISION_TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                              0x047
#define A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x048
#define B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x049
#define C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x04A
#define D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x04B
#define E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x04C
#define F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x04D
#define G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x04E
#define H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                          0x04F
#define LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x050
#define MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x051
#define GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x052
#define COLLISION_LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x053
#define COLLISION_MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x054

#define LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x055
#define MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x056
#define GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x057
#define COLLISION_LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x058
#define COLLISION_MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x059

#define LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x05A
#define MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x05B
#define GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x05C
#define COLLISION_LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x05D
#define COLLISION_MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x05E

#define LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x05F
#define MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x060
#define GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x061
#define COLLISION_LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x062
#define COLLISION_MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x063

#define LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x064
#define MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x065
#define GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x066
#define COLLISION_LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x067
#define COLLISION_MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x068

#define LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x069
#define MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x06A
#define GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x06B
#define COLLISION_LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x06C
#define COLLISION_MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x06D

#define LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x06E
#define MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x06F
#define GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x070
#define COLLISION_LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x071
#define COLLISION_MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x072

#define LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x073
#define MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                  0x074
#define GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                                       0x075
#define COLLISION_LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x076
#define COLLISION_MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN                        0x077

#define A_STAR_COMPLETED                                                                       0x078

#define A_STAR_COLLISION_COMPLETED                                                             0x081
#define A_STAR_COLLISION                                                                       0x082
#define START_A_STAR_COLLISION                                                                 0x083
#define RECALCULATE_A_STAR_COLLISION_COMPLETED                                                 0x084
#define BACK_START_A_STAR_COLLISION                                                            0x085
#define LEFT_OBSTACLE_START_A_STAR_COLLISION                                                   0x086
#define RIGHT_OBSTACLE_START_A_STAR_COLLISION                                                  0x087
#define FRONT_OBSTACLE_START_A_STAR_COLLISION                                                  0x088

#define GO_LEFT_OBSTACLE_START_A_STAR_COLLISION                                                0x089
#define COLLISION_LEFT_OBSTACLE_START_A_STAR_COLLISION                                         0x08A
#define GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION                                               0x08B
#define COLLISION_RIGHT_OBSTACLE_START_A_STAR_COLLISION                                        0x08C
#define GO_FRONT_OBSTACLE_START_A_STAR_COLLISION                                               0x08D
#define COLLISION_FRONT_OBSTACLE_START_A_STAR_COLLISION                                        0x08E

#define GOSTR_CLIFF_RUNNING_STEP                                                               0x0A0
#define CLIFF_COMPLETE                                                                         0x0A1
#define LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                                     0x0A2
#define GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                                                  0x0A3
#define LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                                     0x0A4
#define GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                                                  0x0A5
#define ORIGIN_FRONT_CLIFF_RUNNING_STEP                                                        0x0A6

#define LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                                     0x0A7
#define GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                                                  0x0A8
#define GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                                                   0x0A9
#define LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                                      0x0AA

#define TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                                0x0AC
#define GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                               0x0AD
#define COLLISION_TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                      0x0AE
#define COLLISION_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                     0x0AF
#define TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                          0x0B0
#define COLLISION_TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP                                0x0B1

#define TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                                             0x0B2
#define GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                                            0x0B3
#define COLLISION_TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                                   0x0B4
#define COLLISION_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                                  0x0B5
#define TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                                       0x0B6
#define COLLISION_TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP                             0x0B8

#define TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                                             0x0B9
#define GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                                            0x0BA
#define COLLISION_TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                                   0x0BB
#define COLLISION_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                                  0x0BC
#define TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                                       0x0BD
#define COLLISION_TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP                             0x0BE

#define TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                                0x0BF
#define GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                               0x0C0
#define COLLISION_TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                      0x0C1
#define COLLISION_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                     0x0C2
#define TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                          0x0C3
#define COLLISION_TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP                                0x0C4

#define TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP                                                   0x0C5
#define GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP                                                  0x0C6
#define COLLISION_TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP                                         0x0C7
#define COLLISION_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP                                        0x0C8
#define TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP                                             0x0C9
#define COLLISION_TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP                                   0x0CA

#define TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                                0x0CB
#define GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                               0x0CC
#define COLLISION_TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                      0x0CD
#define COLLISION_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                     0x0CE
#define TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                          0x0CF
#define COLLISION_TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP                                0x0D0

#define TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                                             0x0D1
#define GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                                            0x0D2
#define COLLISION_TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                                   0x0D3
#define COLLISION_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                                  0x0D4
#define TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                                       0x0D5
#define COLLISION_TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP                             0x0D6

#define TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                                              0x0D7
#define GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                                             0x0D8
#define COLLISION_TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                                    0x0D9
#define COLLISION_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                                   0x0DA
#define TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                                        0x0DB
#define COLLISION_TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP                              0x0DC

#define TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                                 0x0DD
#define GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                                0x0DE
#define COLLISION_TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                       0x0DF
#define COLLISION_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                      0x0E0
#define TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                           0x0E1
#define COLLISION_TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP                                 0x0E2
#define INTERRUPT_AND_STOP                                                                     0x0E3



#define GOSTR_RIGHTRUN_STEP                                                                     0x100 //go straight
#define GOSTR_RIGHT_DEV_RIGHTRUN_STEP                                                           0x101 //go straight left deviation
#define GOSTR_LEFT_DEV_RIGHTRUN_STEP                                                            0x102 //go srtaight right deviation
#define COLLISION_RIGHT_RIGHTRUN_STEP                                                           0x103
#define GOBACK_DISTANCE_CRRRS                                                                   0x103  //鍚庨€€涓€娈佃窛绂伙紝璺濈�诲�熻烦杞�
#define DIR_RIGHT_YAW_LESS_ABS90_CRRRS                                                          0x104  //鍒ゆ柇褰撳墠瑙掑害鏄�鍚﹀皬浜嶢BS90锛岃烦杞�

//#define TURN_CLOCK_TARGET_YAW_NEG27_CR_DRYL                                                     0x105 //寮€濮嬪悜鍙宠浆锛屽皬浜�-27鎴栫�版挒锛岃烦杞�
//#define TURN_CLOCK_TARGET_YAW_NEG27_COLLISION_CR_DRYL                                           0x106
//#define GOSTR_YAW_EQUAL_NEG27_CR_DRYL                                                           0x107

#define TURN_CLOCK_TARGET_YAW_NEG90_CR_DRYL                                                     0x105 //寮€濮嬪悜鍙宠浆锛屽皬浜�-27鎴栫�版挒锛岃烦杞�
#define TURN_CLOCK_TARGET_YAW_NEG90_COLLISION_CR_DRYL                                           0x106
#define GOSTR_YAW_EQUAL_NEG90_CR_DRYL                                                           0x107


#define TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL                                                     0x108
#define TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL                                           0x109
#define GOSTR_YAW_EQUAL_NEG57_CR_DRYL                                                           0x10A
#define TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL                                                     0x10B
#define TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL                                           0x10C
#define GOSTR_YAW_EQUAL_NEG82_CR_DRYL                                                           0x10D
#define RIGHT_WALK_EDGE_CR_DRYL                                                                 0x10E
#define TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL                                                    0x10F
#define TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL                                          0x110
#define COMPLETE_CR_DRYL                                                                        0x111
#define DIR_RIGHT_YAW_MORE_ABS90_CRRRS                                                          0x112
#define TURN_CCLCOK_TARGET_YAW_ABS153_CR_DRYM                                                   0x113       //寮€濮嬪悜宸﹁浆锛屽皬浜�153鎴栫�版挒锛岃烦杞�
#define TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM                                         0x114
#define GOSTR_YAW_EQUAL_ABS153_CR_DRYM                                                          0x115
#define GOSTR_BYPASS_CR_DRYM                                                                    0x116
#define RIGHT_COLLISION_BYPASS_CR_DRYM                                                          0x117
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM                                              0x118
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM                                              0x119
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM                                        0x11A
#define TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM                                                    0x11B
#define TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM                                          0x11C
#define TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM                                                        0x11D
#define TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM                                              0x11E
#define MORE_TRY_BREAK_BYPASS_CR_DRYM                                                           0x11F
#define COMPLETE_CR_DRYM                                                                        0x120
#define GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM                                                       0x121
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM                                             0x122
#define GOSTR_BYPASS_LOOP_CR_DRYM                                                               0x123
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM                                                  0x124

#define TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM                                    0x125
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM                                    0x126

#define COLLISION_LEFT_RIGHTRUN_STEP                                                            0x130
#define GOBACK_DISTANCE_CLRRS                                                                   0x131  //鍚庨€€涓€娈佃窛绂伙紝璺濈�诲�熻烦杞�
#define DIR_RIGHT_YAW_LESS_ABS90_CLRRS                                                          0x132
#define TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL                                                     0x133
#define TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL                                           0x134
#define GOSTR_YAW_MORE_ABS30_CL_DRYL                                                            0x135
#define GOSTR_BYPASS_CL_DRYL                                                                    0x136
#define LEFT_COLLISION_BYPASS_CL_DRYL                                                           0x137
#define TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL                                                0x138
#define TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL                                                0x139
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL                                        0x13A
#define TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL                                                0x13B
#define TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL                                      0x13C
#define TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL                                                       0x13D
#define TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL                                             0x13E

#define MORE_TRY_BREAK_BYPASS_CL_DRYL                                                           0x13F
#define COMPLETE_CL_DRYL                                                                        0x140
#define GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL                                                       0x141
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL                                             0x142
#define GOSTR_BYPASS_LOOP_CL_DRYL                                                               0x143
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL                                                  0x144
#define DIR_RIGHT_YAW_MORE_ABS90_CLRRS                                                          0x145

//#define TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM                                                   0x146
//#define TURN_CCLOCK_TAEGET_YAW_ABS153_COLLISION_CL_DRYM                                         0x147
//#define GOSTR_YAW_EQUAL_ABS153_CL_DRYM                                                          0x148

#define TURN_CCLOCK_TARGET_YAW_ABS90_CL_DRYM                                                   0x146
#define TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_CL_DRYM                                         0x147
#define GOSTR_YAW_EQUAL_ABS90_CL_DRYM                           								0x148

#define TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM                                                   0x149
#define TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM                                         0x14A
#define GOSTR_YAW_EQUAL_NEG123_CL_DRYM                                                          0x14B
#define TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM                                                    0x14C
#define TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM                                          0x14D
#define GOSTR_YAW_EQUAL_NEG98_CL_DRYM                                                           0x14E
#define RIGHT_REVERSE_WALK_EDGE_CL_DRYM                                                         0x14F
#define TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM                                                     0x150
#define TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM                                           0x151
#define COMPLETE_CL_DRYM                                                                        0x152

#define TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL                                      0x153
#define TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL                                      0x154

#define COLLISION_FRONT_RIGHTRUN_STEP                                                           0x160
#define GOBACK_DISTANCE_CFRRS                                                                   0x161
#define DIR_RIGHT_YAW_LESS_ABS90_CFRRS                                                          0x162
#define TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL                                                     0x163
#define TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL                                           0x164
#define GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL                                                        0x165
#define TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL                                                     0x166
#define TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL                                           0x167
#define GOSTR_YAW_EQUAL_NEG82_CF_DRYL                                                           0x168
#define RIGHT_WALK_EDGE_CF_DRYL                                                                 0x169
#define TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL                                                    0x16A
#define TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL                                          0x16B
#define COMPLETE_CF_DRYL                                                                        0x16C
#define DIR_RIGHT_YAW_MORE_ABS90_CFRRS                                                          0x16D
#define TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM                                                   0x16E
#define TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM                                         0x16F
#define GOSTR_YAW_ABS120_CF_DRYM                                                                0x170
#define TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM                                                    0x171
#define TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM                                          0x172
#define GOSTR_YAW_ABS93_CF_DRYM                                                                 0x173
#define RIGHT_REVERSE_WALK_EDGE_CF_DRYM                                                         0x174
#define TURN_CCLOCK_TARGET_YAW_ABS10_CF_DRYM                                                    0x175
#define TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM                                          0x176
#define COMPLETE_CF_DRYM                                                                        0x177

#define LEAKING_SWEEP_RIGHTRUN_STEP                                                             0x190
#define RIGHT_LEAKING_SWEEP_COLLISION                                                           0x191
#define RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90                                                      0x192
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90                                         0x193
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION                               0x194
#define RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE                                                     0x195
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3                                          0x196
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION                                0x197
#define RIGHT_LEAKING_SWEEP_YAW_OTHER                                                           0x198
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90                                        0x199
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION                              0x19A
#define RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER                                                    0x19B
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178                                       0x19C
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION                            0x19D
#define RIGHT_LEAKING_SWEEP_COMPLETE                                                            0x19E
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE                                                     0x19F
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP                                                0x1A0
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION                                      0x1A1
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN                                         0x1A2
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN                                 0x1A3
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION                       0x1A4

#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE                                               0x1A5
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP                                          0x1A6
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION                                0x1A7
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN                                   0x1A8
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN                           0x1A9
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION                 0x1AA



#define FORWARD_BOUNDARY_RIGHTRUN_STEP                                                          0x1B0
#define FORWARDBOUNDARY_YAW_LESS_ABS10                                                          0x1B1
#define FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178                                            0x1B2
#define FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION                                  0x1B3
#define FORWARDBOUNDARY_YAW_OTHER                                                               0x1B4
#define FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3                                             0x1B5
#define FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION                                   0x1B6
#define FORWARDBOUNDARY_GOSTRAIGHT                                                              0x1B7
#define FORWARDBOUNDARY_COMPLETE                                                                0x1B8

#define GOBACK_WALK_EDGE                                                                        0x1C0
#define TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE                                                    0x1C1
#define TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE                                          0x1C2
#define READY_GOSTR_BYPASS_WE                                                                   0x1C3
#define GOSTR_BYPASS_WE                                                                         0x1C4
#define COLLISION_BYPASS_WE                                                                     0x1C5
#define TURN_CLOCK_YAW_ADD_ABS15_WE                                                             0x1C6
#define TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE                                                   0x1C7
#define TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE                                                 0x1C8
#define DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE                                            0x1C9
#define TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE                                                      0x1CA
#define TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE                                            0x1CB
#define RIGHT_EDGE_DILEMMA_WE                                                                   0x1CC

//dilemma function
#define GOSTR_X_MORE_LATERALDIS_BYPASS_WE                                                       0x1CD
#define TURN_CCLOCK_TARGET_YAW_LESS_0_WE                                                        0x1CE
#define TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE                                              0x1CF
#define TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE                                    0x1F0
#define TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE                          0x1F1
#define BOW_CONTINUE_WE                                                                         0x1F2
#define RETURN_ORIGIN_WE                                                                        0x1F3
#define REBACK_GOSTR_BYPASS_CHECK_WE                                                            0x1F4

#define GOSTR_BYPASS_WE_X                                                                       0x1F5


//#define CLOSE_EDGE_MAP_RIGHT_WALK                                                               0x1F6
//#define TURN_CLOSE_EDGE_MAP_RIGHT_WALK                                                          0x1F7
//#define COLLISION_TURN_CLOSE_EDGE_MAP_RIGHT_WALK                                                0x1F8
//#define STRAIGHT_CLOSE_EDGE_MAP_RIGHT_WALK                                                      0x1F9

#define CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE                                   0x1F6
#define COLLISION_CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE                         0x1F7
#define FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_WE                                             0x1F8
#define FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE                                   0x1F9



#define GOBACK_REVERSE_WALK_EDGE                                                                0x200
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE                                                   0x201
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE                                         0x202
#define READY_GOSTR_BYPASS_RWE                                                                  0x203
#define GOSTR_BYPASS_RWE                                                                        0x204
#define COLLISION_BYPASS_RWE                                                                    0x205
#define TURN_CCLOCK_YAW_ADD_ABS15_RWE                                                           0x206
#define TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE                                                 0x207
#define TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE                                                     0x208
#define DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                                           0x209
#define TURN_CCLOCK_YAW_MORE_178ABS_RWE                                                         0x20A
#define TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE                                               0x20B
#define RIGHT_EDGE_DILEMMA_RWE                                                                  0x20C

//dilemma function
#define GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                                                      0x20D
#define TURN_CLOCK_TARGET_YAW_LESS_0_RWE                                                        0x20E
#define TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE                                              0x20F
#define TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE                                  0x210
#define TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE                        0x211
#define BOW_CONTINUE_RWE                                                                        0x212
#define RETURN_ORIGIN_RWE                                                                       0x213
#define REBACK_GOSTR_BYPASS_CHECK_RWE                                                           0x214

#define GOSTR_BYPASS_RWE_X                                                                      0x215
//#define CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK                                                       0x216
//#define STRAIGHT_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK                                              0x217
//#define COLLISION_TURN_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK                                        0x219
#define CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE                         0x216
#define COLLISION_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE               0x217
#define FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_RWE                                           0x218
#define FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE                                 0x219


//dilemma function
#define DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA                                       0x221
#define LOOP_TEN_NUM_DILEMMA                                                                   0x222
#define YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA                   0x223
#define GOSTR_DILEMMA                                                                          0x224
#define GOSTR_COLLISION_DILEMMA                                                                0x225

#define COLLISION_YAW_MORE_ABS90_DILEMMA                                                       0x226
#define CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA                                                    0x227
#define CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA                                          0x228
#define GOSTR_CYM_DILEMMA                                                                      0x229
#define GOSTR_COLLISION_CYM_DILEMMA                                                            0x22A
#define GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA                                             0x22B
#define CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA                                                     0x22C
#define CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA                                           0x22D

#define COLLISION_YAW_LESS_ABS90_DILEMMA                                                       0x22E
#define CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA                                                   0x22F
#define CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA                                         0x230
#define GOSTR_CYL_DILEMMA                                                                      0x231
#define GOSTR_COLLISION_CYL_DILEMMA                                                            0x232
#define GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA                                       0x233
#define CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA                                                  0x234
#define CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA                                        0x235

#define COMPLETE_EL_DRYM                                                                       0x236
#define RIGHTWALKEDGE                                                                          0x237
#define RIGHTREVERSEWALKEDGE                                                                   0x238
#define RIGHTEDGEDILEMMA                                                                       0x239
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA                                    0x23A
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_COLLISION                          0x23B
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA                                     0x23C
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA_COLLISION                           0x23D



#define GOSTR_LEFTRUN_STEP                                                                     0x250 //go straight
#define GOSTR_RIGHT_DEV_LEFTRUN_STEP                                                           0x251 //go straight left deviation
#define GOSTR_LEFT_DEV_LEFTRUN_STEP                                                            0x252 //go srtaight right deviation
#define COLLISION_RIGHT_LEFTRUN_STEP                                                           0x253
#define GOBACK_DISTANCE_CRLRS                                                                  0x254
#define DIR_LEFT_YAW_MORE_ABS90_CRLRS                                                          0x255
//#define TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM                                              0x256
//#define TURN_CLOCK_TARGET_YAW_ABS153_COLLISION_LRUN_CR_DLYM                                    0x257
//#define GOSTR_YAW_EQUAL_ABS153_LRUN_CR_DLYM                                                    0x258

#define TURN_CLOCK_TARGET_YAW_ABS90_LRUN_CR_DLYM                                              0x256
#define TURN_CLOCK_TARGET_YAW_ABS90_COLLISION_LRUN_CR_DLYM                                    0x257
#define GOSTR_YAW_EQUAL_ABS90_LRUN_CR_DLYM                                                    0x258

#define TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM                                              0x259
#define TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM                                    0x25A
#define GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM                                                    0x25B
#define TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM                                               0x25C
#define TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM                                     0x25D
#define GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM                                                     0x25E
#define LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM                                                    0x25F
#define TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM                                                0x260
#define TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM                                      0x261
#define COMPLETE_LRUN_CR_DLYM                                                                  0x262
#define DIR_LEFT_YAW_LESS_ABS90_CRLRS                                                          0x263
#define TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL                                              0x264
#define TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL                                    0x265
#define GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL                                                     0x266
#define GOSTR_BYPASS_LRUN_CR_DLYL                                                              0x267
#define RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL                                                    0x268
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL                                         0x269
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL                                         0x26A
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL                                  0x26B
#define TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL                                               0x26C
#define TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL                                     0x26D
#define TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL                                                  0x26E
#define TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL                                        0x26F
#define TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL                                         0x270
#define TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL                               0x271
#define MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL                                                     0x272
#define COMPLETE_LRUN_CR_DLYL                                                                  0x273
#define GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL                                                 0x274
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL                                       0x275
#define GOSTR_BYPASS_LOOP_LRUN_CR_DLYL                                                         0x276
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL                                            0x277
#define GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL                                 0x278
#define GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL                       0x279

#define TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL                               0x27A
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL                               0x27B
#define MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL                                           0x27C

#define COLLISION_LEFT_LEFTRUN_STEP                                                            0x290
#define GOBACK_DISTANCE_CLLRS                                                                  0x291
#define DIR_LEFT_YAW_MORE_ABS90_CLLRS                                                          0x292
#define TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM                                              0x293
#define TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM                                    0x294
#define GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM                                                    0x295
#define GOSTR_BYPASS_LRUN_CL_DLYM                                                              0x296
#define LEFT_COLLISION_BYPASS_LRUN_CL_DLYM                                                     0x297
#define TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM                                         0x298
#define TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM                                         0x299
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM                                  0x29A
#define TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM                                                 0x29B
#define TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM                                       0x29C
#define TURN_CLOCK_TARGET_YAW_LESS_AB3_LRUN_CL_DLYM                                            0x29D
#define TURN_CLOCK_TARGET_YAW_LESS_AB3_COLLISION_LRUN_CL_DLYM                                  0x29E
#define MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM                                                     0x29F
#define COMPLETE_LRUN_CL_DLYM                                                                  0x2A0
#define GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM                                                 0x2A1
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM                                       0x2A2
#define GOSTR_BYPASS_LOOP_LRUN_CL_DLYM                                                         0x2A3
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM                                            0x2A4
#define GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM                                             0x2A5
#define GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM                                   0x2A6
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM                                          0x2A7
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM                                0x2A8
#define DIR_LEFT_YAW_LESS_ABS90_CLLRS                                                          0x2A9

//#define TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL                                              0x2AA
//#define TURN_CCLOCK_TAEGET_YAW_ABS27_COLLISION_LRUN_CL_DLYL                                    0x2AB
//#define GOSTR_YAW_EQUAL_ABS27_LRUN_CL_DLYL                                                     0x2AC

#define TURN_CCLOCK_TARGET_YAW_ABS90_LRUN_CL_DLYL                                              0x2AA
#define TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_LRUN_CL_DLYL                                    0x2AB
#define GOSTR_YAW_EQUAL_ABS90_LRUN_CL_DLYL                                                     0x2AC


#define TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL                                              0x2AD
#define TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL                                    0x2AE
#define GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL                                                     0x2AF
#define TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL                                              0x2B0
#define TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL                                    0x2B1
#define GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL                                                     0x2B2
#define LEFT_WALK_EDGE_LRUN_CL_DLYL                                                            0x2B3
#define TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL                                             0x2B4
#define TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL                                   0x2B5
#define COMPLETE_LRUN_CL_DLYL                                                                  0x2B6

#define TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM                               0x2B7
#define TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM                               0x2B8
#define MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM                                           0x2B9

#define COLLISION_FRONT_LEFTRUN_STEP                                                           0x2D0
#define GOBACK_DISTANCE_CFLRS                                                                  0x2D1
#define DIR_LEFT_YAW_LESS_ABS90_CFRLS                                                          0x2D2
#define TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL                                              0x2D3
#define TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL                                    0x2D4
#define GOSTR_YAW_ABS60_LRUN_CF_DLYL                                                           0x2D5
#define TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL                                              0x2D6
#define TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL                                    0x2D7
#define GOSTR_YAW_ABS82_LRUN_CF_DLYL                                                           0x2D8
#define LEFT_WALK_EDGE_LRUN_CF_DLYL                                                            0x2D9
#define TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL                                             0x2DA
#define TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL                                   0x2DB
#define COMPLETE_LRUN_CF_DLYL                                                                  0x2DC
#define DIR_LEFT_YAW_MORE_ABS90_CFRLS                                                          0x2DD
#define TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM                                              0x2DE
#define TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM                                    0x2DF
#define GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM                                                 0x2E0
#define TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM                                               0x2E1
#define TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM                                     0x2E2
#define GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM                                                     0x2E3
#define LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM                                                    0x2E4
#define TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM                                                0x2E5
#define TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM                                      0x2E6
#define COMPLETE_LRUN_CF_DLYM                                                                  0x2E7

#define LEAKING_SWEEP_LEFTRUN_STEP                                                             0x2F0
#define LEFT_LEAKING_SWEEP_COLLISION                                                           0x2F1
#define LEFT_LEAKING_SWEEP_YAW_MORE_ABS90                                                      0x2F2
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90                                        0x2F3
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION                              0x2F4
#define LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE                                                     0x2F5
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8                                         0x2F6
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION                               0x2F7

#define LEFT_LEAKING_SWEEP_YAW_OTHER                                                           0x2F8
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90                                         0x2F9
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION                               0x2FA
#define LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER                                                    0x2FB
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173                                        0x2FC
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION                             0x2FD
#define LEFT_LEAKING_SWEEP_COMPLETE                                                            0x2FE
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE                                                      0x2FF
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP                                                 0x300
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION                                       0x301
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN                                          0x302
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN                                  0x303
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION                        0x304

#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE                                                0x305
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP                                           0x306
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION                                 0x307
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN                                    0x308
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN                            0x309
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION                  0x30A



#define FORWARD_BOUNDARY_LEFTRUN_STEP                                                          0x310
#define LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10                                                    0x311
#define LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178                                     0x312
#define LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION                           0x313
#define LEFT_FORWARDBOUNDARY_YAW_OTHER                                                         0x314
#define LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3                                        0x315
#define LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION                              0x316
#define LEFT_FORWARDBOUNDARY_GOSTRAIGHT                                                        0x317
#define LEFT_FORWARDBOUNDARY_COMPLETE                                                          0x318

#define LEFT_GOBACK_WALK_EDGE                                                                  0x320
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE                                        0x321
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE                              0x322
#define LEFT_EDGE_READY_GOSTR_BYPASS_WE                                                        0x323
#define LEFT_EDGE_GOSTR_BYPASS_WE                                                              0x324
#define LEFT_EDGE_COLLISION_BYPASS_WE                                                          0x325
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE                                                 0x326
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE                                       0x327
#define LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE                                      0x328
#define LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE                                 0x329
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE                                          0x32A
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE                                0x32B
#define LEFT_EDGE_LEFT_EDGE_DILEMMA_WE                                                         0x32C
//left dilemma function
#define LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE                                            0x32D
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE                                              0x32E
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE                                    0x32F
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE                          0x330
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE                0x331
#define LEFT_EDGE_BOW_CONTINUE_WE                                                              0x332
#define LEFT_EDGE_RETURN_ORIGIN_WE                                                             0x333
#define LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE                                                 0x334

#define LEFT_EDGE_GOSTR_BYPASS_WE_X                                                            0x335
#define CLOSE_EDGE_MAP_LEFT_WALK                                                               0x336
#define STRAIGHT_CLOSE_EDGE_MAP_LEFT_WALK                                                      0x337
#define COLLISION_TURN_CLOSE_EDGE_MAP_LEFT_WALK                                                0x338
#define CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE                                  0x339
#define COLLISION_CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE                        0x33A
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION                                 0x33B
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION_COLLISION                       0x33C

#define LEFT_GOBACK_REVERSE_WALK_EDGE                                                          0x340
#define LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE                                 0x341
#define LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE                       0x342
#define LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE                                               0x343
#define LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE                                                     0x344
#define LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE                                                 0x345
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE                                         0x346
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE                               0x347
#define LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE                                  0x348
#define LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                        0x349
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE                                        0x34A
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE                              0x34B
#define LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE                                                0x34C

//dilemma function
#define LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                                   0x34D
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE                                    0x34E
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE                          0x34F
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE              0x350
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE    0x351
#define LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE                                                     0x352
#define LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE                                                    0x353
#define LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE                                        0x354

#define LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X                                                   0x355
#define CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE                          0x356
#define COLLISION_CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE                0x357
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION                         0x358
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION_COLLISION               0x359

//left dilemma function
#define LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA                          0x361
#define LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA                                                      0x362
#define LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA      0x363
#define LEFT_DILEMMA_GOSTR_DILEMMA                                                             0x364
#define LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA                                                   0x365

#define LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA                                          0x366
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA                                      0x367
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA                            0x368
#define LEFT_DILEMMA_GOSTR_CYM_DILEMMA                                                         0x369
#define LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA                                               0x36A
#define LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA                                0x36B
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA                                       0x36C
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA                             0x36D

#define LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA                                          0x36E
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA                                       0x36F
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA                             0x370
#define LEFT_DILEMMA_GOSTR_CYL_DILEMMA                                                         0x371
#define LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA                                               0x372
#define LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA                          0x373
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA                                      0x374
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA                            0x375
#define COMPLETE_LEFT_DILEMMA                                                                  0x376
#define LEFTWALKEDGE                                                                           0x377
#define LEFTREVERSEWALKEDGE                                                                    0x378
#define LEFTEDGEDILEMMA                                                                        0x379
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION                      0x37A
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION_COLLISION            0x37B
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION                      0x37C
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION_COLLISION            0x37D


#define LEFT_STUCK_FORWARD_BOUNDARY_LEFT_RUNSTEP                                                0x390
#define LEFT_STUCK_FORWARD_BOUNDARY_STATUS                                                      0x391
#define LEFT_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS                               0x392
#define LEFT_GO_STUCK_FORWARD_BOUNDARY_STATUS                                                   0x393
#define LEFT_COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS                     0x394
#define LEFT_COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS                                         0x395
#define LEFT_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS                                          0x396
#define LEFT_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS                               0x397
#define LEFT_COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS                     0x398
#define LEFT_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                                          0x399
#define LEFT_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS                                0x39A
#define LEFT_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                                0x39B
#define LEFT_COLLISION_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS                      0x39C
#define LEFT_COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                                  0x39D
#define LEFT_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                           0x39E
#define LEFT_COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS                 0x39F



#define START_TURN_CLOCK_TARGET_CLOSE_EDGE_MAP                                                  0x3D0
#define START_LOOP_CLOSE_EDGE_MAP                                                               0x3D1
#define LESS_ABS176__CLOSE_EDGE_MAP                                                             0x3D2
#define COLLISION_LESS_ABS176_CLOSE_EDGE_MAP                                                    0x3D3
#define LOOP_CLOSE_EDGE_MAP                                                                     0x3D4
#define COLLISION_LOOP_CLOSE_EDGE_MAP                                                           0x3D5
#define MORE_LOOP_CLOSE_EDGE_MAP                                                                0x3D6
#define COMPLETE_LOOP_CLOSE_EDGE_MAP                                                            0x3D7
#define LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP                                                      0x3D8
#define RIGHT_COLLISION_LOOP_CLOSE_EDGE_MAP                                                     0x3D9
#define COMPLETE_CLOSE_EDGE_MAP                                                                 0x3DA
#define FRONT_COLLISION_LOOP_CLOSE_EDGE_MAP                                                     0x3DB
#define COLLISION_LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP                                            0x3DC
#define X_MORE_LOOP_CLOSE_EDGE_MAP                                                              0x3DD
#define X_LESS_LOOP_CLOSE_EDGE_MAP                                                              0x3DE
#define Y_MORE_LOOP_CLOSE_EDGE_MAP                                                              0x3DF
#define GO_X_MORE_LOOP_CLOSE_EDGE_MAP                                                           0x3E0
#define COLLISION_X_MORE_LOOP_CLOSE_EDGE_MAP                                                    0x3E1
#define COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP                                                 0x3E2
#define TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP                                            0x3E3
#define COLLISION_TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP                                  0x3E4
#define GO_X_LESS_LOOP_CLOSE_EDGE_MAP                                                           0x3E5
#define COLLISION_X_LESS_LOOP_CLOSE_EDGE_MAP                                                    0x3E6
#define COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP                                                 0x3E7
#define TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP                                            0x3E8
#define COLLISION_TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP                                  0x3E9
#define GO_Y_MORE_LOOP_CLOSE_EDGE_MAP                                                           0x3EA
#define COLLISION_Y_MORE_LOOP_CLOSE_EDGE_MAP                                                    0x3EB
#define COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP                                                 0x3EC
#define TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP                                            0x3ED
#define COLLISION_TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP                                  0x3EE
#define GO_Y_LESS_LOOP_CLOSE_EDGE_MAP                                                           0x3EF
#define COLLISION_Y_LESS_LOOP_CLOSE_EDGE_MAP                                                    0x3F0
#define COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP                                                 0x3F1
#define TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP                                            0x3F2
#define COLLISION_TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP                                  0x3F3
#define Y_LESS_LOOP_CLOSE_EDGE_MAP                                                              0x3F4
#define CALCULATION_DELIMMANUMBER_CLOSE_EDGE_MAP                                                0x3F5


#define START_OVERALL_CLEANING_STRATEGY                                                        0x600
#define RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY                                        0x601
#define RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY                                        0x602
#define LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY                                         0x603
#define A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY                             0x604
#define A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY                                 0x605
#define A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY                          0x606
#define A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY                       0x607
#define EDGEWISERUN_CLEANING_STRATEGY                                                          0x608 
#define CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY                                              0x609

#endif

///

///
#if 1
#define RIGHTRUNNING_WORK_SETP 1
#define LEFTRUNNING_WORK_SETP 2
#define RIGHTRETURN_ORIGIN_WORK_SETP 3
#define LEFTRETURN_ORIGIN_WORK_SETP 4
#define ALL_FINSHED_WORK_SETP 5
#define GOSTR_RETURN_ORIGIN_STEP 6
#define DIR_Y_MORE_POSITIVE_200 7
#define DIR_Y_LESS_NEGATIVE_200 8
#define DIR_X_MORE_POSITIVE_200 9
#define DIR_X_LESS_NEGATIVE_200 10
#define TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN 11
#define TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN 12
#define TURN_CLOCK_TARGET_YAW_180_RETURN_ORIGIN 13
#define TURN_CLOCK_TARGET_YAW_5_RETURN_ORIGIN 14
#define COLLISION_Y_POSITIVE_90_RETURN_ORIGIN 15
#define COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN 16
#define COLLISION_X_POSITIVE_180_RETURN_ORIGIN 17
#define COLLISION_X_NEGATIVE_5_RETURN_ORIGIN 18
#define TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN 19
#define TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN 20
#define TURN_CLOCK_X_POSITIVE_180_RETURN_ORIGIN 21
#define TURN_CLOCK_X_NEGATIVE_5_RETURN_ORIGIN 22
#define TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN 23
#define TURN_CLOCK_COLLISION_X_POSITIVE_180_RETURN_ORIGIN 24
#define TURN_CLOCK_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN 25
#define BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 26
#define BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 27
#define BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 28
#define BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 29
#define LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 30
#define LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 31
#define LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 32
#define LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 33
#define COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 34
#define COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 35
#define COLLISION_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 36
#define COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 37
#define Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 38
#define Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 39
#define X_MORE_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 40
#define X_MORE_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 41
#define TURN_CLOCK_LOOP_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN 42
#define TURN_CLOCK_LOOP_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN 43
#define TURN_CLOCK_LOOP_COLLISION_X_POSITIVE_180_RETURN_ORIGIN 44
#define TURN_CLOCK_LOOP_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN 45
#define TURN_CLOCK_DIR_Y_MORE_POSITIVE_200 46
#define TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200 47
#define TURN_CLOCK_DIR_X_MORE_POSITIVE_200 48
#define TURN_CLOCK_DIR_X_LESS_NEGATIVE_200 49
#define STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP 50
#define STUCK_FORWARD_BOUNDARY_STATUS 51
#define TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS 52
#define GO_STUCK_FORWARD_BOUNDARY_STATUS 53
#define COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS 54
#define COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS 55
#define GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 56
#define TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS 57
#define COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS 58
#define ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 59
#define COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 60
#define COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 61
#define COLLISION_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 62
#define COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 63
#define TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 64
#define COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 65
#define ASTAR_MOTION_GOSTR_RETURN 66
#define PLAN_ASTAR_MOTION_GOSTR_RETURN 67
#define START_PLAN_ASTAR_MOTION_GOSTR_RETURN 68
#define COLLISION_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 69
#define TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 70
#define DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 71
#define FINISH_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 72
#define COLLISION_TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 73
#define A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 74
#define B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 75
#define C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 76
#define D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 77
#define E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 78
#define F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 79
#define G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 80
#define H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 81
#define LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 82
#define MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 83
#define GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 84
#define COLLISION_LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 85
#define COLLISION_MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 86
#define LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 87
#define MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 88
#define GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 89
#define COLLISION_LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 90
#define COLLISION_MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 91
#define LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 92
#define MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 93
#define GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 94
#define COLLISION_LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 95
#define COLLISION_MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 96
#define LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 97
#define MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 98
#define GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 99
#define COLLISION_LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 100
#define COLLISION_MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 101
#define LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 102
#define MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 103
#define GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 104
#define COLLISION_LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 105
#define COLLISION_MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 106
#define LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 107
#define MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 108
#define GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 109
#define COLLISION_LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 110
#define COLLISION_MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 111
#define LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 112
#define MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 113
#define GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 114
#define COLLISION_LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 115
#define COLLISION_MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 116
#define LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 117
#define MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 118
#define GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 119
#define COLLISION_LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 120
#define COLLISION_MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 121
#define A_STAR_COMPLETED 122
#define A_STAR_COLLISION_COMPLETED 123
#define A_STAR_COLLISION 124
#define START_A_STAR_COLLISION 125
#define RECALCULATE_A_STAR_COLLISION_COMPLETED 126
#define BACK_START_A_STAR_COLLISION 127
#define LEFT_OBSTACLE_START_A_STAR_COLLISION 128
#define RIGHT_OBSTACLE_START_A_STAR_COLLISION 129
#define FRONT_OBSTACLE_START_A_STAR_COLLISION 130
#define GO_LEFT_OBSTACLE_START_A_STAR_COLLISION 131
#define COLLISION_LEFT_OBSTACLE_START_A_STAR_COLLISION 132
#define GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION 133
#define COLLISION_RIGHT_OBSTACLE_START_A_STAR_COLLISION 134
#define GO_FRONT_OBSTACLE_START_A_STAR_COLLISION 135
#define COLLISION_FRONT_OBSTACLE_START_A_STAR_COLLISION 136
#define GOSTR_CLIFF_RUNNING_STEP 137
#define CLIFF_COMPLETE 138
#define LESS_90_R_FRONT_CLIFF_RUNNING_STEP 139
#define GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 140
#define LESS_90_L_FRONT_CLIFF_RUNNING_STEP 141
#define GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 142
#define ORIGIN_FRONT_CLIFF_RUNNING_STEP 143
#define LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 144
#define GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 145
#define GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 146
#define LESS_90_L_LEFT_CLIFF_RUNNING_STEP 147
#define TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP 148
#define GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP 149
#define COLLISION_TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP 150
#define COLLISION_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP 151
#define TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP 152
#define COLLISION_TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP 153
#define TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 154
#define GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 155
#define COLLISION_TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 156
#define COLLISION_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 157
#define TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 158
#define COLLISION_TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP 159
#define TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 160
#define GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 161
#define COLLISION_TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 162
#define COLLISION_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 163
#define TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 164
#define COLLISION_TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP 165
#define TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP 166
#define GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP 167
#define COLLISION_TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP 168
#define COLLISION_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP 169
#define TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP 170
#define COLLISION_TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP 171
#define TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP 172
#define GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP 173
#define COLLISION_TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP 174
#define COLLISION_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP 175
#define TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP 176
#define COLLISION_TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP 177
#define TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 178
#define GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 179
#define COLLISION_TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 180
#define COLLISION_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 181
#define TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 182
#define COLLISION_TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP 183
#define TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 184
#define GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 185
#define COLLISION_TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 186
#define COLLISION_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 187
#define TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 188
#define COLLISION_TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP 189
#define TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 190
#define GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 191
#define COLLISION_TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 192
#define COLLISION_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 193
#define TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 194
#define COLLISION_TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP 195
#define TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP 196
#define GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP 197
#define COLLISION_TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP 198
#define COLLISION_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP 199
#define TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP 200
#define COLLISION_TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP 201
#define INTERRUPT_AND_STOP 202
#define GOSTR_RIGHTRUN_STEP 203
#define GOSTR_RIGHT_DEV_RIGHTRUN_STEP 204
#define GOSTR_LEFT_DEV_RIGHTRUN_STEP 205
#define COLLISION_RIGHT_RIGHTRUN_STEP 206
#define GOBACK_DISTANCE_CRRRS 207
#define DIR_RIGHT_YAW_LESS_ABS90_CRRRS 208
#define TURN_CLOCK_TARGET_YAW_NEG90_CR_DRYL 209
#define TURN_CLOCK_TARGET_YAW_NEG90_COLLISION_CR_DRYL 210
#define GOSTR_YAW_EQUAL_NEG90_CR_DRYL 211
#define TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL 212
#define TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL 213
#define GOSTR_YAW_EQUAL_NEG57_CR_DRYL 214
#define TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL 215
#define TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL 216
#define GOSTR_YAW_EQUAL_NEG82_CR_DRYL 217
#define RIGHT_WALK_EDGE_CR_DRYL 218
#define TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL 219
#define TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL 220
#define COMPLETE_CR_DRYL 221
#define DIR_RIGHT_YAW_MORE_ABS90_CRRRS 222
#define TURN_CCLCOK_TARGET_YAW_ABS153_CR_DRYM 223
#define TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM 224
#define GOSTR_YAW_EQUAL_ABS153_CR_DRYM 225
#define GOSTR_BYPASS_CR_DRYM 226
#define RIGHT_COLLISION_BYPASS_CR_DRYM 227
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM 228
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM 229
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM 230
#define TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM 231
#define TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM 232
#define TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM 233
#define TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM 234
#define MORE_TRY_BREAK_BYPASS_CR_DRYM 235
#define COMPLETE_CR_DRYM 236
#define GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM 237
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM 238
#define GOSTR_BYPASS_LOOP_CR_DRYM 239
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM 240
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM 241
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM 242
#define COLLISION_LEFT_RIGHTRUN_STEP 243
#define GOBACK_DISTANCE_CLRRS 244
#define DIR_RIGHT_YAW_LESS_ABS90_CLRRS 245
#define TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL 246
#define TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL 247
#define GOSTR_YAW_MORE_ABS30_CL_DRYL 248
#define GOSTR_BYPASS_CL_DRYL 249
#define LEFT_COLLISION_BYPASS_CL_DRYL 250
#define TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL 251
#define TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL 252
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL 253
#define TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL 254
#define TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL 255
#define TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL 256
#define TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL 257
#define MORE_TRY_BREAK_BYPASS_CL_DRYL 258
#define COMPLETE_CL_DRYL 259
#define GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL 260
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL 261
#define GOSTR_BYPASS_LOOP_CL_DRYL 262
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL 263
#define DIR_RIGHT_YAW_MORE_ABS90_CLRRS 264
#define TURN_CCLOCK_TARGET_YAW_ABS90_CL_DRYM 265
#define TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_CL_DRYM 266
#define GOSTR_YAW_EQUAL_ABS90_CL_DRYM 267
#define TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM 268
#define TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM 269
#define GOSTR_YAW_EQUAL_NEG123_CL_DRYM 270
#define TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM 271
#define TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM 272
#define GOSTR_YAW_EQUAL_NEG98_CL_DRYM 273
#define RIGHT_REVERSE_WALK_EDGE_CL_DRYM 274
#define TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM 275
#define TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM 276
#define COMPLETE_CL_DRYM 277
#define TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL 278
#define TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL 279
#define COLLISION_FRONT_RIGHTRUN_STEP 280
#define GOBACK_DISTANCE_CFRRS 281
#define DIR_RIGHT_YAW_LESS_ABS90_CFRRS 282
#define TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL 283
#define TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL 284
#define GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL 285
#define TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL 286
#define TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL 287
#define GOSTR_YAW_EQUAL_NEG82_CF_DRYL 288
#define RIGHT_WALK_EDGE_CF_DRYL 289
#define TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL 290
#define TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL 291
#define COMPLETE_CF_DRYL 292
#define DIR_RIGHT_YAW_MORE_ABS90_CFRRS 293
#define TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM 294
#define TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM 295
#define GOSTR_YAW_ABS120_CF_DRYM 296
#define TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM 297
#define TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM 298
#define GOSTR_YAW_ABS93_CF_DRYM 299
#define RIGHT_REVERSE_WALK_EDGE_CF_DRYM 300
#define TURN_CCLOCK_TARGET_YAW_ABS10_CF_DRYM 301
#define TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM 302
#define COMPLETE_CF_DRYM 303
#define LEAKING_SWEEP_RIGHTRUN_STEP 304
#define RIGHT_LEAKING_SWEEP_COLLISION 305
#define RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90 306
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90 307
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION 308
#define RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE 309
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3 310
#define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION 311
#define RIGHT_LEAKING_SWEEP_YAW_OTHER 312
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90 313
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION 314
#define RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER 315
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178 316
#define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION 317
#define RIGHT_LEAKING_SWEEP_COMPLETE 318
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE 319
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP 320
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION 321
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN 322
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN 323
#define RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION 324
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE 325
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP 326
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION 327
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN 328
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN 329
#define RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION 330
#define FORWARD_BOUNDARY_RIGHTRUN_STEP 331
#define FORWARDBOUNDARY_YAW_LESS_ABS10 332
#define FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178 333
#define FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION 334
#define FORWARDBOUNDARY_YAW_OTHER 335
#define FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3 336
#define FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION 337
#define FORWARDBOUNDARY_GOSTRAIGHT 338
#define FORWARDBOUNDARY_COMPLETE 339
#define GOBACK_WALK_EDGE 340
#define TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE 341
#define TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE 342
#define READY_GOSTR_BYPASS_WE 343
#define GOSTR_BYPASS_WE 344
#define COLLISION_BYPASS_WE 345
#define TURN_CLOCK_YAW_ADD_ABS15_WE 346
#define TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE 347
#define TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE 348
#define DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE 349
#define TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE 350
#define TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE 351
#define RIGHT_EDGE_DILEMMA_WE 352
#define GOSTR_X_MORE_LATERALDIS_BYPASS_WE 353
#define TURN_CCLOCK_TARGET_YAW_LESS_0_WE 354
#define TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE 355
#define TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE 356
#define TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE 357
#define BOW_CONTINUE_WE 358
#define RETURN_ORIGIN_WE 359
#define REBACK_GOSTR_BYPASS_CHECK_WE 360
#define GOSTR_BYPASS_WE_X 361
#define CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE 362
#define COLLISION_CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE 363
#define FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_WE 364
#define FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE 365
#define GOBACK_REVERSE_WALK_EDGE 366
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE 367
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE 368
#define READY_GOSTR_BYPASS_RWE 369
#define GOSTR_BYPASS_RWE 370
#define COLLISION_BYPASS_RWE 371
#define TURN_CCLOCK_YAW_ADD_ABS15_RWE 372
#define TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE 373
#define TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE 374
#define DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE 375
#define TURN_CCLOCK_YAW_MORE_178ABS_RWE 376
#define TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE 377
#define RIGHT_EDGE_DILEMMA_RWE 378
#define GOSTR_X_MORE_LATERALDIS_BYPASS_RWE 379
#define TURN_CLOCK_TARGET_YAW_LESS_0_RWE 380
#define TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE 381
#define TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE 382
#define TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE 383
#define BOW_CONTINUE_RWE 384
#define RETURN_ORIGIN_RWE 385
#define REBACK_GOSTR_BYPASS_CHECK_RWE 386
#define GOSTR_BYPASS_RWE_X 387
#define CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE 388
#define COLLISION_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE 389
#define FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_RWE 390
#define FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE 391
#define DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA 392
#define LOOP_TEN_NUM_DILEMMA 393
#define YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA 394
#define GOSTR_DILEMMA 395
#define GOSTR_COLLISION_DILEMMA 396
#define COLLISION_YAW_MORE_ABS90_DILEMMA 397
#define CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA 398
#define CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA 399
#define GOSTR_CYM_DILEMMA 400
#define GOSTR_COLLISION_CYM_DILEMMA 401
#define GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA 402
#define CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA 403
#define CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA 404
#define COLLISION_YAW_LESS_ABS90_DILEMMA 405
#define CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA 406
#define CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA 407
#define GOSTR_CYL_DILEMMA 408
#define GOSTR_COLLISION_CYL_DILEMMA 409
#define GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA 410
#define CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA 411
#define CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA 412
#define COMPLETE_EL_DRYM 413
#define RIGHTWALKEDGE 414
#define RIGHTREVERSEWALKEDGE 415
#define RIGHTEDGEDILEMMA 416
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA 417
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_COLLISION 418
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA 419
#define FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA_COLLISION 420
#define GOSTR_LEFTRUN_STEP 421
#define GOSTR_RIGHT_DEV_LEFTRUN_STEP 422
#define GOSTR_LEFT_DEV_LEFTRUN_STEP 423
#define COLLISION_RIGHT_LEFTRUN_STEP 424
#define GOBACK_DISTANCE_CRLRS 425
#define DIR_LEFT_YAW_MORE_ABS90_CRLRS 426
#define TURN_CLOCK_TARGET_YAW_ABS90_LRUN_CR_DLYM 427
#define TURN_CLOCK_TARGET_YAW_ABS90_COLLISION_LRUN_CR_DLYM 428
#define GOSTR_YAW_EQUAL_ABS90_LRUN_CR_DLYM 429
#define TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM 430
#define TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM 431
#define GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM 432
#define TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM 433
#define TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM 434
#define GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM 435
#define LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM 436
#define TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM 437
#define TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM 438
#define COMPLETE_LRUN_CR_DLYM 439
#define DIR_LEFT_YAW_LESS_ABS90_CRLRS 440
#define TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL 441
#define TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL 442
#define GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL 443
#define GOSTR_BYPASS_LRUN_CR_DLYL 444
#define RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL 445
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL 446
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL 447
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL 448
#define TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL 449
#define TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL 450
#define TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL 451
#define TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL 452
#define TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL 453
#define TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL 454
#define MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL 455
#define COMPLETE_LRUN_CR_DLYL 456
#define GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL 457
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL 458
#define GOSTR_BYPASS_LOOP_LRUN_CR_DLYL 459
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL 460
#define GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL 461
#define GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL 462
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL 463
#define TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL 464
#define MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL 465
#define COLLISION_LEFT_LEFTRUN_STEP 466
#define GOBACK_DISTANCE_CLLRS 467
#define DIR_LEFT_YAW_MORE_ABS90_CLLRS 468
#define TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM 469
#define TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM 470
#define GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM 471
#define GOSTR_BYPASS_LRUN_CL_DLYM 472
#define LEFT_COLLISION_BYPASS_LRUN_CL_DLYM 473
#define TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM 474
#define TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM 475
#define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM 476
#define TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM 477
#define TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM 478
#define TURN_CLOCK_TARGET_YAW_LESS_AB3_LRUN_CL_DLYM 479
#define TURN_CLOCK_TARGET_YAW_LESS_AB3_COLLISION_LRUN_CL_DLYM 480
#define MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM 481
#define COMPLETE_LRUN_CL_DLYM 482
#define GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM 483
#define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM 484
#define GOSTR_BYPASS_LOOP_LRUN_CL_DLYM 485
#define GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM 486
#define GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM 487
#define GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM 488
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM 489
#define TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM 490
#define DIR_LEFT_YAW_LESS_ABS90_CLLRS 491
#define TURN_CCLOCK_TARGET_YAW_ABS90_LRUN_CL_DLYL 492
#define TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_LRUN_CL_DLYL 493
#define GOSTR_YAW_EQUAL_ABS90_LRUN_CL_DLYL 494
#define TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL 495
#define TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL 496
#define GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL 497
#define TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL 498
#define TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL 499
#define GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL 500
#define LEFT_WALK_EDGE_LRUN_CL_DLYL 501
#define TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL 502
#define TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL 503
#define COMPLETE_LRUN_CL_DLYL 504
#define TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM 505
#define TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM 506
#define MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM 507
#define COLLISION_FRONT_LEFTRUN_STEP 508
#define GOBACK_DISTANCE_CFLRS 509
#define DIR_LEFT_YAW_LESS_ABS90_CFRLS 510
#define TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL 511
#define TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL 512
#define GOSTR_YAW_ABS60_LRUN_CF_DLYL 513
#define TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL 514
#define TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL 515
#define GOSTR_YAW_ABS82_LRUN_CF_DLYL 516
#define LEFT_WALK_EDGE_LRUN_CF_DLYL 517
#define TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL 518
#define TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL 519
#define COMPLETE_LRUN_CF_DLYL 520
#define DIR_LEFT_YAW_MORE_ABS90_CFRLS 521
#define TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM 522
#define TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM 523
#define GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM 524
#define TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM 525
#define TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM 526
#define GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM 527
#define LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM 528
#define TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM 529
#define TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM 530
#define COMPLETE_LRUN_CF_DLYM 531
#define LEAKING_SWEEP_LEFTRUN_STEP 532
#define LEFT_LEAKING_SWEEP_COLLISION 533
#define LEFT_LEAKING_SWEEP_YAW_MORE_ABS90 534
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90 535
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION 536
#define LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE 537
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8 538
#define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION 539
#define LEFT_LEAKING_SWEEP_YAW_OTHER 540
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90 541
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION 542
#define LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER 543
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173 544
#define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION 545
#define LEFT_LEAKING_SWEEP_COMPLETE 546
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE 547
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP 548
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION 549
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN 550
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN 551
#define LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION 552
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE 553
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP 554
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION 555
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN 556
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN 557
#define LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION 558
#define FORWARD_BOUNDARY_LEFTRUN_STEP 559
#define LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10 560
#define LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178 561
#define LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION 562
#define LEFT_FORWARDBOUNDARY_YAW_OTHER 563
#define LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3 564
#define LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION 565
#define LEFT_FORWARDBOUNDARY_GOSTRAIGHT 566
#define LEFT_FORWARDBOUNDARY_COMPLETE 567
#define LEFT_GOBACK_WALK_EDGE 568
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE 569
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE 570
#define LEFT_EDGE_READY_GOSTR_BYPASS_WE 571
#define LEFT_EDGE_GOSTR_BYPASS_WE 572
#define LEFT_EDGE_COLLISION_BYPASS_WE 573
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE 574
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE 575
#define LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE 576
#define LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE 577
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE 578
#define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE 579
#define LEFT_EDGE_LEFT_EDGE_DILEMMA_WE 580
#define LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE 581
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE 582
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE 583
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE 584
#define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE 585
#define LEFT_EDGE_BOW_CONTINUE_WE 586
#define LEFT_EDGE_RETURN_ORIGIN_WE 587
#define LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE 588
#define LEFT_EDGE_GOSTR_BYPASS_WE_X 589
#define CLOSE_EDGE_MAP_LEFT_WALK 590
#define STRAIGHT_CLOSE_EDGE_MAP_LEFT_WALK 591
#define COLLISION_TURN_CLOSE_EDGE_MAP_LEFT_WALK 592
#define CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE 593
#define COLLISION_CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE 594
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION 595
#define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION_COLLISION 596
#define LEFT_GOBACK_REVERSE_WALK_EDGE 597
#define LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE 598
#define LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE 599
#define LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE 600
#define LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE 601
#define LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE 602
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE 603
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE 604
#define LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE 605
#define LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE 606
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE 607
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE 608
#define LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE 609
#define LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE 610
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE 611
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE 612
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE 613
#define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE 614
#define LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE 615
#define LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE 616
#define LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE 617
#define LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X 618
#define CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE 619
#define COLLISION_CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE 620
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION 621
#define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION_COLLISION 622
#define LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA 623
#define LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA 624
#define LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA 625
#define LEFT_DILEMMA_GOSTR_DILEMMA 626
#define LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA 627
#define LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA 628
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA 629
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA 630
#define LEFT_DILEMMA_GOSTR_CYM_DILEMMA 631
#define LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA 632
#define LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA 633
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA 634
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA 635
#define LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA 636
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA 637
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA 638
#define LEFT_DILEMMA_GOSTR_CYL_DILEMMA 639
#define LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA 640
#define LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA 641
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA 642
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA 643
#define COMPLETE_LEFT_DILEMMA 644
#define LEFTWALKEDGE 645
#define LEFTREVERSEWALKEDGE 646
#define LEFTEDGEDILEMMA 647
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION 648
#define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION_COLLISION 649
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION 650
#define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION_COLLISION 651
#define LEFT_STUCK_FORWARD_BOUNDARY_LEFT_RUNSTEP 652
#define LEFT_STUCK_FORWARD_BOUNDARY_STATUS 653
#define LEFT_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS 654
#define LEFT_GO_STUCK_FORWARD_BOUNDARY_STATUS 655
#define LEFT_COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS 656
#define LEFT_COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS 657
#define LEFT_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 658
#define LEFT_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS 659
#define LEFT_COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS 660
#define LEFT_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 661
#define LEFT_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 662
#define LEFT_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 663
#define LEFT_COLLISION_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 664
#define LEFT_COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 665
#define LEFT_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 666
#define LEFT_COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 667
#define START_TURN_CLOCK_TARGET_CLOSE_EDGE_MAP 668
#define START_LOOP_CLOSE_EDGE_MAP 669
#define LESS_ABS176__CLOSE_EDGE_MAP 670
#define COLLISION_LESS_ABS176_CLOSE_EDGE_MAP 671
#define LOOP_CLOSE_EDGE_MAP 672
#define COLLISION_LOOP_CLOSE_EDGE_MAP 673
#define MORE_LOOP_CLOSE_EDGE_MAP 674
#define COMPLETE_LOOP_CLOSE_EDGE_MAP 675
#define LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP 676
#define RIGHT_COLLISION_LOOP_CLOSE_EDGE_MAP 677
#define COMPLETE_CLOSE_EDGE_MAP 678
#define FRONT_COLLISION_LOOP_CLOSE_EDGE_MAP 679
#define COLLISION_LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP 680
#define X_MORE_LOOP_CLOSE_EDGE_MAP 681
#define X_LESS_LOOP_CLOSE_EDGE_MAP 682
#define Y_MORE_LOOP_CLOSE_EDGE_MAP 683
#define GO_X_MORE_LOOP_CLOSE_EDGE_MAP 684
#define COLLISION_X_MORE_LOOP_CLOSE_EDGE_MAP 685
#define COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP 686
#define TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP 687
#define COLLISION_TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP 688
#define GO_X_LESS_LOOP_CLOSE_EDGE_MAP 689
#define COLLISION_X_LESS_LOOP_CLOSE_EDGE_MAP 690
#define COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP 691
#define TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP 692
#define COLLISION_TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP 693
#define GO_Y_MORE_LOOP_CLOSE_EDGE_MAP 694
#define COLLISION_Y_MORE_LOOP_CLOSE_EDGE_MAP 695
#define COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP 696
#define TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP 697
#define COLLISION_TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP 698
#define GO_Y_LESS_LOOP_CLOSE_EDGE_MAP 699
#define COLLISION_Y_LESS_LOOP_CLOSE_EDGE_MAP 700
#define COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP 701
#define TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP 702
#define COLLISION_TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP 703
#define Y_LESS_LOOP_CLOSE_EDGE_MAP 704
#define CALCULATION_DELIMMANUMBER_CLOSE_EDGE_MAP 705
#define START_OVERALL_CLEANING_STRATEGY 706
#define RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY 707
#define RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 708
#define LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY 709
#define A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 710
#define A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 711
#define A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 712
#define A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 713
#define EDGEWISERUN_CLEANING_STRATEGY 714
#define CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY 715
#endif



//all define left
#define PI 3.14159265
#define front_backward_distance 10
#define side_backward_distance 20//15 //25
#define star_collision_backward 50
#define star_collision_go 400
#define collision_backward_distance 10//3
#define lateral_move_distance 160
#define walk_edge_distance 200
#define close_map_move_distance 5
#define turn_backward_distance 10//3
#define second_turn_backward_distance 10//4
#define close_edge 160
#define dilemma_close_edge 80
#define return_origin_distance 200

#define left_cliff 1
#define front_cliff 1
#define right_cliff 1
#define none_cliff 0

#define W 5000
#define real_gostaright_vel 300
#define long_stra_vel 250
#define turn_vel 50
#define correction_turn_vel 20//40 //10
#define correction_straight_vel 10//200 //200
#define correction_big_turn_vel 20
#define correction_big_straight_vel 10//100 

#define  CleanAreaThreshold  500000


//#define Deg2Rad(deg) (PI * deg / 180.0F)
//#define Rad2Deg(rad) (180.0F * rad / PI)

#define DegToRad Deg2Rad
#define RadToDeg Rad2Deg

#define front_obstacle FRONT_OBSTACLE_SIGNAL 
#define left_obstacle LEFT_OBSTACLE_SIGNAL 
#define right_obstacle RIGHT_OBSTACLE_SIGNAL 
#define none_obstacle NONE_OBSTACLE_SIGNAL


//*******************************************
#define AStar_MaxLength  51
#define AStar_Height     25
#define AStar_Width      25
#define compression_map_x      25
#define compression_map_y      25
#define AStar_Reachable   0
#define AStar_Sequential  0
#define AStar_NoSolution  1
#define AStar_Infinity    0xffff

#define AStar_East       (1 << 0)
#define AStar_South_East (1 << 1)
#define AStar_South      (1 << 2)
#define AStar_South_West (1 << 3)
#define AStar_West       (1 << 4)
#define AStar_North_West (1 << 5)
#define AStar_North      (1 << 6)
#define AStar_North_East (1 << 7)

extern unsigned int RealWorkTime ;

typedef struct AStarPoint
{
    signed char x, y;
} AStarPoint;

typedef struct AStar_MapNode
{
    unsigned char x, y;
    unsigned char sur;
    bool reachable;
} AStar_MapNode;

typedef struct AStar_Close
{
    AStar_MapNode *cur;
    char vis;
    struct AStar_Close *from;
    short AStar_F, AStar_G;
    short AStar_H;
} AStar_Close;

typedef struct AStar_Open
{
    short length;
    AStar_Close* Array[AStar_MaxLength];
} AStar_Open;


#define uint8_t unsigned char


typedef struct POSE{
    int x;
    int y;
    int orientation;}POSE;

typedef struct
{
//    int work_step_status;
//    uint8_t right_running_complete;
//    uint8_t right_return_origin_complete;
//    uint8_t left_running_complete;
//    uint8_t left_return_origin_complete;
    
    volatile unsigned char action ;
    volatile bool isRunning ;
    volatile unsigned int delay ;
}CleanStrategyB;

double my_abs(double x);
bool isCleanRunning(void);
void bsp_StartUpdateCleanStrategyB(void);
void bsp_ResetCleanStrategyBStatus(void);
void bsp_StopUpdateCleanStrategyB(void);
void bsp_UpdateCleanStrategyB(int robotX,int robotY,double robotTheta,unsigned char obstacleSignal, int current_wheel_pulse_l, int current_wheel_pulse_r, unsigned char IRSensorData[],CLIFFADCVALUE * cliff_value);

uint8_t clean_strategyB(POSE *current_pose,unsigned char obstacleSignal);

unsigned char RightRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char ForwardBoundaryRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionRightRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionLeftRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionFrontRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal);


unsigned char LeftRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char ForwardBoundaryLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionRightLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionLeftLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionFrontLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal);
unsigned char StuckLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);


void StuckRunStep(POSE *current_pose);
unsigned char StuckRightRunStep(POSE *current_pose, unsigned char obstacleSignal);


/////////////////////////////////

//return origin

void RightMapExtreme(void);
void initOpen(AStar_Open *q);
void push(AStar_Open *q, AStar_Close cls[AStar_Height][AStar_Width], unsigned char x, unsigned char y, short g);
AStar_Close* shift(AStar_Open *q);
void initClose(AStar_Close cls[AStar_Height][AStar_Width], unsigned char sx, unsigned char sy,unsigned char dx,unsigned char dy);
void initGraph(bool maze[AStar_Height][AStar_Width],unsigned char sx,unsigned char sy,unsigned char dx,unsigned char dy);
bool astar(void);
AStar_Close* getShortest(void);
unsigned char printShortest(void);

unsigned char ForceReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);

unsigned char AStarReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarCollision(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarNotMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
/////////////////////////////////////

//unsigned char CliffRuningWorkStep(POSE *current_pose,CLIFFADCVALUE * cliff_value,unsigned char obstacleSignal);
unsigned char CloseEdgedMap(POSE *current_pose,unsigned char obstacleSignal);
unsigned char DetectionCloseEdge(void);
unsigned char CliffCloseEdge(void);
void MoreMap(POSE *current_pose);
void LessMap(void);
void StartUpdateGridMap(void);


uint8_t GetReturnChargeStationStatus(void);
void ResetReturnChargeStationStatus(void);
void ReturnExtreme_point_init(void);

int bsp_GetStrategyCurrentPosX(void);
int bsp_GetStrategyCurrentPosY(void);

void bsp_StartUpdateGridMap(void);
void bsp_StopUpdateGridMap(void);
unsigned char* bsp_GetIRSensorData(void);
short Edge_length(void);
short bsp_Right_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal);
short bsp_Left_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal);

const unsigned char*  bsp_Get_GridMap(int robotX,int robotY);


uint16_t bsp_GetStrategy_MajorIndex(void);
uint16_t bsp_GetStrategy_MinorIndex(void);

#endif


