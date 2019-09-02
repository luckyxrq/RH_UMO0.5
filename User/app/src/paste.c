/*
	这个文件没有功能，用于文件对比粘贴方便
*/


IR_TX_LEFT   = 0x27 ,
IR_TX_CENTER = 0x39 ,
IR_TX_RIGHT  = 0x16

IR_TX_SITE_LEFT   = 0 ,
IR_TX_SITE_CENTER ,
IR_TX_SITE_RIGHT  


TotalWitdh
g_tIR.TotalWitdh[ch]


g_tIR.isRev[ch][IR_TX_SITE_LEFT] = false;
g_tIR.isRev[ch][IR_TX_SITE_CENTER] = false;
g_tIR.isRev[ch][IR_TX_SITE_RIGHT] = false;


static void bsp_SearchRunStraightFast(void);
static void bsp_SearchRunStraightSlow(void);
static void bsp_SearchTurnRightFast(void)  ;
static void bsp_SearchTurnRightSlow(void)  ;
static void bsp_SearchTurnLeftFast(void)   ;
static void bsp_SearchTurnLeftSlow(void)   ;