/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-ActPack: User code running on Ex
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-09-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_ACTPACK

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************
 
#include "main.h"
#include "user-ex-ActPack.h"
#include "user-ex-rigid.h"
#include "flexsea_user_structs.h"
#include "flexsea_sys_def.h"
#include "flexsea_board.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************  

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Common setup done in user-ex-rigid/init_project()
//Add extra steps here
void initActPack(void)
{
	//...
}

//Finite State Machine.
//Call this function in one of the main while time slots.
void ActPack_fsm(void)
{
	ActPack_refresh_values();
}

//Common variables are refreshed in user-ex-rigid/refresh_values()
//Add extra variables here
void ActPack_refresh_values(void)
{	
	//...
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif //BOARD_TYPE_FLEXSEA_EXECUTE

#endif 	//INCLUDE_UPROJ_ACTPACK
