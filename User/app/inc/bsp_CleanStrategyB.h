#ifndef __BSP_CLEANSTRATEGYB_H
#define __BSP_CLEANSTRATEGYB_H

#include <stdbool.h>



#define START_OVERALL_CLEANING_STRATEGY 0x1110
#define RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY 0x1111
#define RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 0x1112
#define LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY 0x1113
#define A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 0x1114
#define A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 0x1115
#define A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 0x1116
#define A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY 0x1117

#define RIGHTRUNNING_WORK_SETP        0x01
#define LEFTRUNNING_WORK_SETP         0x02
#define RIGHTRETURN_ORIGIN_WORK_SETP  0x03
#define LEFTRETURN_ORIGIN_WORK_SETP   0x04
#define ALL_FINSHED_WORK_SETP         0x05

#define RIGHTWALKEDGE                 0x0A
#define RIGHTREVERSEWALKEDGE          0x0B
#define RIGHTEDGEDILEMMA              0x0C


#define GOSTR_RIGHTRUN_STEP                               0x1 //go straight  
#define GOSTR_RIGHT_DEV_RIGHTRUN_STEP                     0x11 //go straight left deviation 
#define GOSTR_LEFT_DEV_RIGHTRUN_STEP                      0x12 //go srtaight right deviation
#define COLLISION_RIGHT_RIGHTRUN_STEP                     0x2
    #define GOBACK_DISTANCE_CRRRS                             0x21  //后退一段距离，距离够跳转
    #define DIR_RIGHT_YAW_LESS_ABS90_CRRRS                    0x22  //判断当前角度是否小于ABS90，跳转
        #define TURN_CLOCK_TARGET_YAW_NEG27_CR_DRYL               0x221 //开始向右转，小于-27或碰撞，跳转
        #define TURN_CLOCK_TARGET_YAW_NEG27_COLLISION_CR_DRYL      0x222
        #define GOSTR_YAW_EQUAL_NEG27_CR_DRYL                     0x223
        #define TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL               0x224
        #define TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL      0x225
        #define GOSTR_YAW_EQUAL_NEG57_CR_DRYL                     0x226
        #define TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL               0x227
        #define TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL      0x228
        #define GOSTR_YAW_EQUAL_NEG82_CR_DRYL                     0x229
        #define RIGHT_WALK_EDGE_CR_DRYL                           0x22A
        #define TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL              0x22B
        #define TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL     0x22C
        #define COMPLETE_CR_DRYL                                  0x22D 
    #define DIR_RIGHT_YAW_MORE_ABS90_CRRRS                    0x23
        #define TURN_CCLCOK_TARGET_YAW_ABS153_CR_DRYM             0x231          //开始向左转，小于153或碰撞，跳转
        #define TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM    0x232
        #define GOSTR_YAW_EQUAL_ABS153_CR_DRYM                    0x233        
        #define GOSTR_BYPASS_CR_DRYM                              0x234     
        #define RIGHT_COLLISION_BYPASS_CR_DRYM                    0x235     
        #define TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM        0x236
        #define TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM        0x237 
        #define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM  0x239        
        #define TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM              0x23A 
        #define TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM    0x23B     
        #define TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM                  0x23C  
        #define TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM        0x23D
        #define MORE_TRY_BREAK_BYPASS_CR_DRYM                     0x23E      
        #define COMPLETE_CR_DRYM                                  0x23F    
        #define GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM                 0x240
        #define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM       0x241
        #define GOSTR_BYPASS_LOOP_CR_DRYM                         0x242
        #define GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM            0x243

        #define TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM        0x244
        #define TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM        0x245


    
#define COLLISION_LEFT_RIGHTRUN_STEP                   0x3
    #define GOBACK_DISTANCE_CLRRS                                  0x31      //后退一段距离，距离够跳转
    #define DIR_RIGHT_YAW_LESS_ABS90_CLRRS                         0x32     
        #define TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL                0x321       
        #define TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL       0x322           
        #define GOSTR_YAW_MORE_ABS30_CL_DRYL                       0x323          
        #define GOSTR_BYPASS_CL_DRYL                               0x324       
        #define LEFT_COLLISION_BYPASS_CL_DRYL                      0x325        
        #define TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL           0x326
		#define TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL 0x33E
        #define TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL           0x327
		#define TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL 0x33F        
        #define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL   0x328           
        #define TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL           0x329 
        #define TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL 0x32A 
        #define TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL                  0x32B 
        #define TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL        0x32C
        #define MORE_TRY_BREAK_BYPASS_CL_DRYL                      0x23F              
        #define COMPLETE_CL_DRYL                                   0x240
        #define GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL                  0x241
        #define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL        0x242
        #define GOSTR_BYPASS_LOOP_CL_DRYL                          0x243
        #define GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL             0x244
    #define DIR_RIGHT_YAW_MORE_ABS90_CLRRS                  0x33           
        #define TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM             0x331         
        #define TURN_CCLOCK_TAEGET_YAW_ABS153_COLLISION_CL_DRYM    0x332            
        #define GOSTR_YAW_EQUAL_ABS153_CL_DRYM                    0x333           
        #define TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM             0x334        
        #define TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM    0x335              
        #define GOSTR_YAW_EQUAL_NEG123_CL_DRYM                    0x336     
        #define TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM              0x337            
        #define TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM     0x338           
        #define GOSTR_YAW_EQUAL_NEG98_CL_DRYM                     0x339        
        #define RIGHT_REVERSE_WALK_EDGE_CL_DRYM                   0x33A         
        #define TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM               0x33B          
        #define TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM      0x33C            
        #define COMPLETE_CL_DRYM                                  0x33D
		
#define COLLISION_FRONT_RIGHTRUN_STEP                  0x4
    #define GOBACK_DISTANCE_CFRRS                             0x41
    #define DIR_RIGHT_YAW_LESS_ABS90_CFRRS                    0x42
        #define TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL           0x421                 
        #define TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL  0x422               
        #define GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL              0x423              
        #define TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL           0x424             
        #define TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL  0x425             
        #define GOSTR_YAW_EQUAL_NEG82_CF_DRYL                 0x426           
        #define RIGHT_WALK_EDGE_CF_DRYL                       0x427           
        #define TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL          0x428            
        #define TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL 0x429                    
        #define COMPLETE_CF_DRYL                              0x42A           
    #define DIR_RIGHT_YAW_MORE_ABS90_CFRRS                     0x43
        #define TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM          0x431               
        #define TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM 0x432             
        #define GOSTR_YAW_ABS120_CF_DRYM                       0x433           
        #define TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM           0x434               
        #define TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM  0x435                 
        #define GOSTR_YAW_ABS93_CF_DRYM                        0x436         
        #define RIGHT_REVERSE_WALK_EDGE_CF_DRYM                0x437              
        #define TURN_CCLOCK_TARGET_YAW_ABS10_CF_DRYM           0x438          
        #define TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM  0x439       
        #define COMPLETE_CF_DRYM                               0x43A      

#define LEAKING_SWEEP_RIGHTRUN_STEP              0x05
    #define RIGHT_LEAKING_SWEEP_COLLISION                                0x0501                  
    #define RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90                           0x0502                       
    #define RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90              0x0503                                    
    #define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION    0x0504                                              
    #define RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE                          0x0505                        
    #define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3               0x0506                                   
    #define RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION     0x0507                                             
    #define RIGHT_LEAKING_SWEEP_YAW_OTHER                                0x0508                  
    #define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90             0x0509                                     
    #define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION   0x050A                                               
    #define RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER                         0x050B                         
    #define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178             0x050C                                     
    #define RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION  0x050D                                                
    #define RIGHT_LEAKING_SWEEP_COMPLETE                                 0x050E                

#define FORWARD_BOUNDARY_RIGHTRUN_STEP           0x06
    #define FORWARDBOUNDARY_YAW_LESS_ABS10                                    0x0601
    #define FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178                      0x0602
    #define FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION            0x0603
    #define FORWARDBOUNDARY_YAW_OTHER                                         0x0604
    #define FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3                       0x0605
    #define FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION             0x0606
    #define FORWARDBOUNDARY_GOSTRAIGHT                                        0x0607
    #define FORWARDBOUNDARY_COMPLETE                                          0x0608


#define GOBACK_WALK_EDGE                                                   0x60                                                                      
    #define TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE                           0x61                                                     
    #define TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE                  0x62                                                             
    #define READY_GOSTR_BYPASS_WE                                          0x63
    #define GOSTR_BYPASS_WE                                                0x64                                                                                               
    #define COLLISION_BYPASS_WE                                            0x65                                   
    #define TURN_CLOCK_YAW_ADD_ABS15_WE                                    0x66                                           
    #define TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE                           0x67                                                                                                                     
    #define TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE                        0x68                                                       
    #define DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE                   0x69                                                            
    #define TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE                             0x6A                                                  
    #define TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE                    0x6B                                                           
    #define RIGHT_EDGE_DILEMMA_WE                                          0x6C 
    //dilemma function
    #define GOSTR_X_MORE_LATERALDIS_BYPASS_WE                              0x6D                                                 
    #define TURN_CCLOCK_TARGET_YAW_LESS_0_WE                               0x6E                                                
    #define TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE                      0x6F                                                         
    #define TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE           0x70
    #define TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE  0x71                                                                    
    #define BOW_CONTINUE_WE                                                0x72                              
    #define RETURN_ORIGIN_WE                                               0x73 
    #define REBACK_GOSTR_BYPASS_CHECK_WE                                   0x74

    #define GOSTR_BYPASS_WE_X                                                0x75

#define GOBACK_REVERSE_WALK_EDGE                                             0x80                                                        
    #define TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE                            0x81                                              
    #define TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE                   0x82                                                       
    #define READY_GOSTR_BYPASS_RWE                                           0x83
    #define GOSTR_BYPASS_RWE                                                 0x84                         
    #define COLLISION_BYPASS_RWE                                              0x85                            
    #define TURN_CCLOCK_YAW_ADD_ABS15_RWE                                    0x86                                      
    #define TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE                           0x87                                               
    #define TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE                              0x88                                            
    #define DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                    0x89                                                      
    #define TURN_CCLOCK_YAW_MORE_178ABS_RWE                                  0x8A                                        
    #define TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE                         0x8B                                                 
    #define RIGHT_EDGE_DILEMMA_RWE                                           0x8C                                                         
    //dilemma function       
    #define GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                               0x8D                                           
    #define TURN_CLOCK_TARGET_YAW_LESS_0_RWE                                 0x8E                                                       
    #define TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE                        0x8F                                                                 
    #define TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE           0x90                                                               
    #define TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE  0x91
    #define BOW_CONTINUE_RWE                                                 0x92                        
    #define RETURN_ORIGIN_RWE                                                0x93
    #define REBACK_GOSTR_BYPASS_CHECK_RWE                                    0x94

    #define GOSTR_BYPASS_RWE_X                                                 0x95


//dilemma function
    #define DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA                             0x6B01                                                           
    #define LOOP_TEN_NUM_DILEMMA                                                         0x6B02                            
    #define YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA         0x6B03                                                                                                                                                          
    #define GOSTR_DILEMMA                                                                0x6B04                                               
    #define GOSTR_COLLISION_DILEMMA                                                       0x6B05 

    #define COLLISION_YAW_MORE_ABS90_DILEMMA                                              0x6B06                                       
    #define CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA                                          0x6B07                                           
    #define CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA                                0x6B08
    #define GOSTR_CYM_DILEMMA                                                            0x6B09      
    #define GOSTR_COLLISION_CYM_DILEMMA                                                   0x6B0A                                  
    #define GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA                                   0x6B0B                                                  
    #define CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA                                           0x6B0C
    #define CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA                                 0x6B0D 

    #define COLLISION_YAW_LESS_ABS90_DILEMMA                                              0x6B0E                                       
    #define CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA                                         0x6B0F                                           
    #define CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA                               0x6B10 
    #define GOSTR_CYL_DILEMMA                                                            0x6B11 
    #define GOSTR_COLLISION_CYL_DILEMMA                                                   0x6B12                                  
    #define GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA                             0x6B13                                                        
    #define CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA                                        0x6B14                                            
    #define CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA                              0x6B15
	#define COMPLETE_DEILEMMA                 											 0x6B16

//all define  right

#define GOSTR_LEFTRUN_STEP                               0x1 //go straight
#define GOSTR_RIGHT_DEV_LEFTRUN_STEP                     0x11 //go straight left deviation
#define GOSTR_LEFT_DEV_LEFTRUN_STEP                      0x12 //go srtaight right deviation
#define COLLISION_RIGHT_LEFTRUN_STEP                     0x2
    #define GOBACK_DISTANCE_CRLRS                             0x21
    #define DIR_LEFT_YAW_MORE_ABS90_CRLRS                    0x22
        #define TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM               0x221
        #define TURN_CLOCK_TARGET_YAW_ABS153_COLLISION_LRUN_CR_DLYM      0x222
        #define GOSTR_YAW_EQUAL_ABS153_LRUN_CR_DLYM                     0x223
        #define TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM               0x224
        #define TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM      0x225
        #define GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM                     0x226
        #define TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM               0x227
        #define TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM      0x228
        #define GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM                     0x229
        #define LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM                    0x22A
        #define TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM              0x22B
        #define TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM     0x22C
        #define COMPLETE_LRUN_CR_DLYM                                  0x22D
    #define DIR_LEFT_YAW_LESS_ABS90_CRLRS                    0x23
        #define TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL             0x231
        #define TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL    0x232
        #define GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL                    0x233
        #define GOSTR_BYPASS_LRUN_CR_DLYL                              0x234
        #define RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL                    0x235
        #define TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL        0x236
        #define TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL        0x237
		#define TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL        0x250
        #define TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL        0x251
        #define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL  0x239
        #define TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL              0x23A
        #define TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL    0x23B
        #define TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL                  0x23C
        #define TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL        0x23D
        #define TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL           0x246
        #define TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL 0x247
        #define MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL                               0x23E
        #define COMPLETE_LRUN_CR_DLYL                                            0x23F
        #define GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL                           0x240
        #define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL                 0x241
        #define GOSTR_BYPASS_LOOP_LRUN_CR_DLYL                                   0x242
        #define GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL                      0x243
        #define GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL            0x244
        #define GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL  0x245

        #define MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL                 0x252




#define COLLISION_LEFT_LEFTRUN_STEP                   0x3
    #define GOBACK_DISTANCE_CLLRS                                  0x31
    #define DIR_LEFT_YAW_MORE_ABS90_CLLRS                         0x32
        #define TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM                0x321
        #define TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM       0x322
        #define GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM                       0x323
        #define GOSTR_BYPASS_LRUN_CL_DLYM                               0x324
        #define LEFT_COLLISION_BYPASS_LRUN_CL_DLYM                      0x325
        #define TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM           0x326
        #define TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM           0x327
        #define GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM   0x328
        #define TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM                  0x32B
        #define TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM        0x32C
        #define TURN_CLOCK_TARGET_YAW_LESS_AB3_LRUN_CL_DLYM           0x329
        #define TURN_CLOCK_TARGET_YAW_LESS_AB3_COLLISION_LRUN_CL_DLYM 0x32A
        #define MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM                      0x23F
        #define COMPLETE_LRUN_CL_DLYM                                   0x240
        #define GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM                  0x241
        #define GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM        0x242
        #define GOSTR_BYPASS_LOOP_LRUN_CL_DLYM                          0x243
        #define GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM             0x244
        #define GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM              0x245
        #define GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM    0x246
        #define TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM           0x247
        #define TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM 0x248
    #define DIR_LEFT_YAW_LESS_ABS90_CLLRS                  0x33
        #define TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL             0x331
        #define TURN_CCLOCK_TAEGET_YAW_ABS27_COLLISION_LRUN_CL_DLYL    0x332
        #define GOSTR_YAW_EQUAL_ABS27_LRUN_CL_DLYL                    0x333
        #define TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL             0x334
        #define TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL    0x335
        #define GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL                    0x336
        #define TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL              0x337
        #define TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL     0x338
        #define GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL                     0x339
        #define LEFT_WALK_EDGE_LRUN_CL_DLYL                            0x33A
        #define TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL               0x33B
        #define TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL      0x33C
        #define COMPLETE_LRUN_CL_DLYL                                  0x33D


        #define TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM           0x340
        #define TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM           0x341
        #define MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM                       0x342

#define COLLISION_FRONT_LEFTRUN_STEP                  0x4
    #define GOBACK_DISTANCE_CFLRS                             0x41
    #define DIR_LEFT_YAW_LESS_ABS90_CFRLS                     0x43
        #define TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL          0x431
        #define TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL 0x432
        #define GOSTR_YAW_ABS60_LRUN_CF_DLYL                       0x433
        #define TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL           0x434
        #define TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL  0x435
        #define GOSTR_YAW_ABS82_LRUN_CF_DLYL                        0x436
        #define LEFT_WALK_EDGE_LRUN_CF_DLYL                0x437
        #define TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL           0x438
        #define TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL  0x439
        #define COMPLETE_LRUN_CF_DLYL                               0x43A
    #define DIR_LEFT_YAW_MORE_ABS90_CFRLS                    0x42
        #define TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM           0x421
        #define TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM  0x422
        #define GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM              0x423
        #define TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM           0x424
        #define TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM  0x425
        #define GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM                 0x426
        #define LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM                       0x427
        #define TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM          0x428
        #define TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM 0x429
        #define COMPLETE_LRUN_CF_DLYM                              0x42A



#define LEAKING_SWEEP_LEFTRUN_STEP              0x05
    #define LEFT_LEAKING_SWEEP_COLLISION                                0x0501
    #define LEFT_LEAKING_SWEEP_YAW_MORE_ABS90                           0x0502
    #define LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90              0x0503
    #define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION    0x0504
    #define LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE                          0x0505
    #define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8               0x0506
    #define LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION     0x0507

    #define LEFT_LEAKING_SWEEP_YAW_OTHER                                0x0508
    #define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90             0x0509
    #define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION   0x050A
    #define LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER                         0x050B
    #define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173             0x050C
    #define LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION  0x050D
    #define LEFT_LEAKING_SWEEP_COMPLETE                                 0x050E

#define FORWARD_BOUNDARY_LEFTRUN_STEP           0x06
    #define LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10                                    0x0601
    #define LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178                      0x0602
    #define LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION            0x0603
    #define LEFT_FORWARDBOUNDARY_YAW_OTHER                                         0x0604
    #define LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3                       0x0605
    #define LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION             0x0606
    #define LEFT_FORWARDBOUNDARY_GOSTRAIGHT                                        0x0607
    #define LEFT_FORWARDBOUNDARY_COMPLETE                                          0x0608


#define LEFT_GOBACK_WALK_EDGE                                                   0x60
    #define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE                           0x61
    #define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE                  0x62
    #define LEFT_EDGE_READY_GOSTR_BYPASS_WE                                          0x63
    #define LEFT_EDGE_GOSTR_BYPASS_WE                                                0x64
    #define LEFT_EDGE_COLLISION_BYPASS_WE                                            0x65
    #define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE                                    0x66
    #define LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE                           0x67
    #define LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE                        0x68
    #define LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE                   0x69
    #define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE                             0x6A
    #define LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE                    0x6B
    #define LEFT_EDGE_LEFT_EDGE_DILEMMA_WE                                          0x6C
    //left dilemma function
    #define LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE                              0x6D
    #define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE                               0x6E
    #define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE                      0x6F
    #define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE           0x70
    #define LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE  0x71
    #define LEFT_EDGE_BOW_CONTINUE_WE                                                0x72
    #define LEFT_EDGE_RETURN_ORIGIN_WE                                               0x73
    #define LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE                                   0x74
    #define LEFT_EDGE_GOSTR_BYPASS_WE_X                                                0x75


#define LEFT_GOBACK_REVERSE_WALK_EDGE                                             0x80
    #define LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE                            0x81
    #define LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE                   0x82
    #define LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE                                           0x83
    #define LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE                                                 0x84
    #define LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE                                              0x85
    #define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE                                    0x86
    #define LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE                           0x87
    #define LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE                              0x88
    #define LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                    0x89
    #define LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE                                  0x8A
    #define LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE                         0x8B
    #define LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE                                           0x8C
    //dilemma function
    #define LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                               0x8D
    #define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE                                 0x8E
    #define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE                        0x8F
    #define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE           0x90
    #define LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE  0x91
    #define LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE                                                 0x92
    #define LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE                                                0x93
    #define LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE                                    0x94


    #define LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X                                               0x95



//left dilemma function
    #define LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA                             0x6B01
    #define LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA                                                         0x6B02
    #define LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA         0x6B03
    #define LEFT_DILEMMA_GOSTR_DILEMMA                                                                0x6B04
    #define LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA                                                       0x6B05

    #define LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA                                              0x6B06
    #define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA                                          0x6B07
    #define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA                                0x6B08
    #define LEFT_DILEMMA_GOSTR_CYM_DILEMMA                                                            0x6B09
    #define LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA                                                   0x6B0A
    #define LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA                                   0x6B0B
    #define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA                                           0x6B0C
    #define LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA                                 0x6B0D

    #define LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA                                              0x6B0E
    #define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA                                         0x6B0F
    #define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA                               0x6B10
    #define LEFT_DILEMMA_GOSTR_CYL_DILEMMA                                                            0x6B11
    #define LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA                                                   0x6B12
    #define LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA                             0x6B13
    #define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA                                        0x6B14
    #define LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA                              0x6B15

#define COMPLETE_LEFT_DILEMMA                              0x6C01
#define LEFTWALKEDGE                                       0x6C02
#define LEFTREVERSEWALKEDGE                                0x6C03
#define LEFTEDGEDILEMMA                                    0x6C04

//all define left

#define PI 3.14159265
#define front_backward_distance 10
#define side_backward_distance 25
#define collision_backward_distance 3
#define lateral_move_distance 160
#define turn_backward_distance 3
#define second_turn_backward_distance 4
#define close_edge 160
#define return_origin_distance 400

#define W 5000
#define long_stra_vel 250
#define turn_vel 50
//#define Deg2Rad(deg) (PI * deg / 180.0F)
//#define Rad2Deg(rad) (180.0F * rad / PI)

#define DegToRad Deg2Rad
#define RadToDeg Rad2Deg

//#define FRONT_OBSTACLE_SIGNAL 2
//#define LEFT_OBSTACLE_SIGNAL 0
//#define RIGHT_OBSTACLE_SIGNAL 1

#define front_obstacle FRONT_OBSTACLE_SIGNAL 
#define left_obstacle LEFT_OBSTACLE_SIGNAL 
#define right_obstacle RIGHT_OBSTACLE_SIGNAL 
#define none_obstacle NONE_OBSTACLE_SIGNAL

//**********return origin define*************
#define GOSTR_RETURN_ORIGIN_STEP 0x001
#define DIR_Y_MORE_POSITIVE_200 0x002
#define DIR_Y_LESS_NEGATIVE_200 0x003
#define DIR_X_MORE_POSITIVE_200 0x004
#define DIR_X_LESS_NEGATIVE_200 0x005

#define TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN 0x006
#define TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN 0x007
#define TURN_CLOCK_TARGET_YAW_180_RETURN_ORIGIN 0x008
#define TURN_CLOCK_TARGET_YAW_5_RETURN_ORIGIN 0x009

#define COLLISION_Y_POSITIVE_90_RETURN_ORIGIN 0x00A
#define COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN 0x00B
#define COLLISION_X_POSITIVE_180_RETURN_ORIGIN 0x00C
#define COLLISION_X_NEGATIVE_5_RETURN_ORIGIN 0x00D

#define TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN 0x00E
#define TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN 0x00F
#define TURN_CLOCK_X_POSITIVE_180_RETURN_ORIGIN 0x010
#define TURN_CLOCK_X_NEGATIVE_5_RETURN_ORIGIN 0x011

#define TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN 0x012
#define TURN_CLOCK_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN 0x013
#define TURN_CLOCK_COLLISION_X_POSITIVE_180_RETURN_ORIGIN 0x014
#define TURN_CLOCK_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN 0x015

#define BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 0x016
#define BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 0x017
#define BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 0x018
#define BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 0x019

#define LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 0x01A
#define LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 0x01B
#define LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 0x01C
#define LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 0x01D

#define COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 0x01E
#define COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 0x01F
#define COLLISION_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 0x020
#define COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 0x021

#define Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN 0x022
#define Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN 0x023
#define X_MORE_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN 0x024
#define X_MORE_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN 0x025

#define TURN_CLOCK_LOOP_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN 0x026
#define TURN_CLOCK_LOOP_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN 0x027
#define TURN_CLOCK_LOOP_COLLISION_X_POSITIVE_180_RETURN_ORIGIN 0x028
#define TURN_CLOCK_LOOP_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN 0x029

#define TURN_CLOCK_DIR_Y_MORE_POSITIVE_200 0x02A
#define TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200 0x02B
#define TURN_CLOCK_DIR_X_MORE_POSITIVE_200 0x02C
#define TURN_CLOCK_DIR_X_LESS_NEGATIVE_200 0x02D

//*************stuck*************
#define STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP 0x030
#define STUCK_FORWARD_BOUNDARY_STATUS 0x031
#define TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS 0x032
#define GO_STUCK_FORWARD_BOUNDARY_STATUS 0x033
#define COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS 0x034
#define COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS 0x035
#define GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 0x036
#define TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS 0x037
#define COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS 0x038
#define ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 0x039
#define COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 0x03A
#define COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 0x03B
#define COLLISION_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS 0x03C
#define COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 0x03D
#define TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 0x03E
#define COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS 0x03F

//*************A* return origin*************
#define ASTAR_MOTION_GOSTR_RETURN 0x040
#define PLAN_ASTAR_MOTION_GOSTR_RETURN 0x041
#define START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x042
#define COLLISION_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x043
#define TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x044
#define DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x045
#define FINISH_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x046
#define COLLISION_TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x047
#define A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x048
#define B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x049
#define C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x04A
#define D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x04B
#define E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x04C
#define F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x04D
#define G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x04E
#define H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x04F
#define LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x050
#define MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x051
#define GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x052
#define COLLISION_LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x053
#define COLLISION_MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x054

#define LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x055
#define MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x056
#define GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x057
#define COLLISION_LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x058
#define COLLISION_MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x059

#define LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x05A
#define MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x05B
#define GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x05C
#define COLLISION_LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x05D
#define COLLISION_MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x05E

#define LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x05F
#define MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x060
#define GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x061
#define COLLISION_LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x062
#define COLLISION_MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x063

#define LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x064
#define MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x065
#define GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x066
#define COLLISION_LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x067
#define COLLISION_MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x068

#define LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x069
#define MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x06A
#define GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x06B
#define COLLISION_LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x06C
#define COLLISION_MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x06D

#define LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x06E
#define MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x06F
#define GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x070
#define COLLISION_LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x071
#define COLLISION_MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x072

#define LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x073
#define MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x074
#define GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x075
#define COLLISION_LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x076
#define COLLISION_MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN 0x077

#define A_STAR_COMPLETED 0x078

#define A_STAR_COLLISION_COMPLETED 0x081
#define A_STAR_COLLISION 0x082
#define START_A_STAR_COLLISION 0x083
#define RECALCULATE_A_STAR_COLLISION_COMPLETED 0x084
#define BACK_START_A_STAR_COLLISION 0x085
#define LEFT_OBSTACLE_START_A_STAR_COLLISION 0x086
#define RIGHT_OBSTACLE_START_A_STAR_COLLISION 0x087
#define FRONT_OBSTACLE_START_A_STAR_COLLISION 0x088

#define GO_LEFT_OBSTACLE_START_A_STAR_COLLISION 0x089
#define COLLISION_LEFT_OBSTACLE_START_A_STAR_COLLISION 0x08A
#define GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION 0x08B
#define COLLISION_RIGHT_OBSTACLE_START_A_STAR_COLLISION 0x08C
#define GO_FRONT_OBSTACLE_START_A_STAR_COLLISION 0x08D
#define COLLISION_FRONT_OBSTACLE_START_A_STAR_COLLISION 0x08E

//*******************************************
#define AStar_MaxLength 100
#define AStar_Height 25
#define AStar_Width 25
#define AStar_Reachable 0
#define AStar_Bar 1
#define AStar_Pass 2
#define AStar_Source 3
#define AStar_Destination 4
#define AStar_Sequential 0
#define AStar_NoSolution 2
#define AStar_Infinity 0xfffffff

#define AStar_East (1 << 0)
#define AStar_South_East (1 << 1)
#define AStar_South (1 << 2)
#define AStar_South_West (1 << 3)
#define AStar_West (1 << 4)
#define AStar_North_West (1 << 5)
#define AStar_North (1 << 6)
#define AStar_North_East (1 << 7)

typedef struct AStarPoint
{
    signed char x, y;
} AStarPoint;

typedef struct AStar_MapNode
{
    int x, y;
    unsigned char reachable, sur, value;
} AStar_MapNode;

typedef struct AStar_Close
{
    AStar_MapNode *cur;
    char vis;
    struct AStar_Close *from;
    float AStar_F, AStar_G;
    int AStar_H;
} AStar_Close;

typedef struct AStar_Open
{
    int length;
    AStar_Close *Array[AStar_MaxLength];
} AStar_Open;


#define uint8_t unsigned char

typedef struct POSE{
    int x;
    int y;
    int orientation;}POSE;

typedef struct
{
	int work_step_status;
	uint8_t right_running_complete;
	uint8_t right_return_origin_complete;
	uint8_t left_running_complete;
	uint8_t left_return_origin_complete;

	volatile unsigned char action ;
	volatile bool isRunning ;
	volatile unsigned int delay ;
}CleanStrategyB;

void bsp_StartUpdateCleanStrategyB(void);
void bsp_ResetCleanStrategyBStatus(void);
void bsp_StopUpdateCleanStrategyB(void);
void bsp_CleanStrategyUpdateB(int robotX,int robotY,double robotTheta,unsigned char obstacleSignal, int current_wheel_pulse_l, int current_wheel_pulse_r, unsigned char IRSensorData[]);
uint8_t clean_strategy(POSE *current_pose,unsigned char obstacleSignal);



unsigned char RightRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightReturnOriginWorkStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char ForwardBoundaryRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionRightRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionLeftRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionFrontRightRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char RightEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal);


unsigned char LeftRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftReturnOriginWorkStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char ForwardBoundaryLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionRightLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionLeftLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char CollisionFrontLeftRunStep(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal);
unsigned char LeftEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal);




//stuck 
unsigned char StuckRightRunStep(POSE *current_pose, unsigned char obstacleSignal);
/////////////////////////////////

//return origin
unsigned char within(int x, int y);
void initOpen(AStar_Open *q);
void push(AStar_Open *q, AStar_Close cls[AStar_Height][AStar_Width], int x, int y, float g);
AStar_Close *shift(AStar_Open *q);
void initClose(AStar_Close cls[AStar_Height][AStar_Width], int sx, int sy, int dx, int dy);
void initGraph(bool maze[AStar_Height][AStar_Width], int sx, int sy, int dx, int dy);
int astar(void);
AStar_Close *getShortest(void);
int printShortest(void);

unsigned char RightReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarCollision(POSE *current_pose, unsigned char obstacleSignal);
unsigned char AStarNotMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal);
/////////////////////////////////////




#endif


	
