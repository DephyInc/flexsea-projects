/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user_ankle_2dof: 2-DoF Ankle Functions
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_DLEG
#include "main.h"

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_MIT_DLEG
#define INC_MIT_DLEG

//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern int16_t glob_var_1;
extern int16_t glob_var_2;
extern int16_t glob_var_3;

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_MIT_DLeg(void);
void MIT_DLeg_fsm_1(void);
void MIT_DLeg_fsm_2(void);

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
int8_t safetyShutoff(void);
void clampCurrent(float* pcurrentDes);
float calcCurrent(float torqueDes);
int8_t findPoles(void);
void openSpeedFSM(void);
void twoPositionFSM(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//activate one of these for joint limits
#define IS_ANKLE_LEFT
//#define IS_ANKLE_RIGHT

//toDO check if counterclockwise convention is respected by joint encoders
#ifdef IS_ANKLE_LEFT
#define JOINT_MIN -10	//dorsi
#define JOINT_MAX  10	//plantar
#endif

#ifdef IS_ANKLE_RIGHT
#define JOINT_MIN -10
#define JOINT_MAX  10
#endif

//safety limits
#define FORCE_LIMIT	 	 50
#define CURRENT_LIMIT    10
#define CURRENT_SCALAR   1
#define MOTOR_TEMP_LIMIT 70
#define BOARD_TEMP_LIMIT 70


//Constants used by get_ankle_ang():
//Where is this equation???
#define A0 				(202.2+1140.0)
#define A1 				1302.0
#define A2				-39.06
#define B1 				14.76
#define B2 				-7.874
#define W				0.00223

#define SECONDS			1000

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
