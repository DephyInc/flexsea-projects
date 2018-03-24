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
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user_ankle_2dof: EMG FSM functions
****************************************************************************/

#include "main.h"
#include "state_variables.h"

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_EMG_STAND_H
#define INC_EMG_STAND_H

//****************************************************************************
// EASY ACCESS
//****************************************************************************

//HANDLE EMG FROM SEONG
#define GAIN_LG_STAND					1.0
#define GAIN_TA_STAND					1.0
#define EMG_IN_MAX_STAND				10000

//Constants for tuning the controller
#define PF_TORQUE_GAIN_STAND			50
#define DF_TORQUE_GAIN_STAND			50
#define PFDF_STIFF_GAIN_STAND			100
#define DP_ON_THRESH_STAND			0.1

#define COCON_THRESH_STAND			0.3 //co-contraction threshold


//VIRTUAL DYNAMIC JOINT PARAMS
#define VIRTUAL_K_STAND				1.0
#define VIRTUAL_B_STAND				0.1
#define VIRTUAL_J_STAND				0.0025


//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern int32_t stand_EMGavgs[2];
extern float stand_state[3];
extern float stand_equilibriumAngle;

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void updateStandJoint(GainParams* pgains);
void get_stand_EMG(void);
void interpret_stand_EMG (float k, float b, float J);
void RK4_SIMPLE_STAND(float dtheta, float domega, float* cur_state);

//****************************************************************************
// Definition(s):
//****************************************************************************


//****************************************************************************
// Structure(s)
//****************************************************************************


#endif	//INC_EMG_STAND_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
