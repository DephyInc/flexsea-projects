/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex: User Projects & Functions, FlexSEA-Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-30 | jfduval | New release
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

/*Important: we reached a point where we couldn't support all configurations
  without changing the TopDesign (we ran out of ressources). You might have
  to select a different TopDesign file than the one included by default (check
  the folded, there is more than one included) */

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-ex.h"
#include "flexsea_user_structs.h"
#include "flexsea_sys_def.h"
#include "filters.h"
#include "control.h"
#include "motor.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//struct diffarr_s mot_ang_clks;
int32_t mot_ang = 0;
int32_t mot_ang_offset = 0; //initialized offset to match with calibration
int32_t mot_vel = 0;
int32_t mot_acc = 0;
	
//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void refresh_values(void);
static void init_project(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialization function - call once in main.c, before while()
void init_user(void)
{
	//Common setup:
	init_project();

	//MIT Ankle 2-DoF:
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
	init_ankle_2dof();
	#endif	//PROJECT_ANKLE_2DOF

	//RIC/NU Knee:
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
	init_ricnu_knee();
	#endif	//PROJECT_RICNU_KNEE

	//DpEb21
	#if(ACTIVE_PROJECT == PROJECT_DPEB21)
	initDpEb21();
	#endif	//PROJECT_MOTORTB
}

//Call this function in one of the main while time slots.
void user_fsm(void)
{
	//Refresh sensor values - common across projects:
	refresh_values();
	
	//DpEB2.1:
	#if(ACTIVE_PROJECT == PROJECT_DPEB21)
		DpEb21_refresh_values();
		#if(RUNTIME_FSM == ENABLED)
			DpEb21_fsm();
		#endif
	#endif	//PROJECT_DPEB21
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Common setup
static void init_project(void)
{
	setBoardID(SLAVE_ID);
	
	//Controller setup:
	ctrl[0].active_ctrl = CTRL_NONE;	//No controller at boot
	setMotorVoltage(0, 0);				//0% PWM
}

//Refresh sensor values - common across projects
static void refresh_values(void)
{
}

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
