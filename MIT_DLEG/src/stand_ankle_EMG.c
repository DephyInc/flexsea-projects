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
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-MIT_2DoF_Ankle_v1: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	* 2016-11-16 | jfduval | Cleaned code, improved formatting
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "stand_ankle_EMG.h"
#include "user-mn-MIT-EMG.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "state_variables.h"
#include "flexsea_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************
// joint limits
float stand_max_DFangle = -5 * -JOINT_ANGLE_DIR; // In terms of robot limits. Convention inside here is opposite robot.
float stand_max_PFangle = 40 * -JOINT_ANGLE_DIR;
float stand_equilibriumAngle = 0 * -JOINT_ANGLE_DIR; // 30 degrees plantarflexion

int32_t stand_EMGavgs[2] = {0, 0}; // Initialize the EMG signals to 0;
float stand_state[3] = {0, 0, 0};

float stand_stepsize = .001; //dt, if running at 1kHz

//HANDLE EMG FROM SEONG
float stand_gainLG = GAIN_LG_STAND;
float stand_gainTA = GAIN_TA_STAND;
float stand_emgInMax = EMG_IN_MAX_STAND;

//Constants for tuning the controller
float stand_pfTorqueGain = PF_TORQUE_GAIN_STAND;
float stand_dfTorqueGain = DF_TORQUE_GAIN_STAND;
float stand_pfdfStiffGain = PFDF_STIFF_GAIN_STAND;
float stand_dpOnThresh = DP_ON_THRESH_STAND;
float stand_cocontractThresh = COCON_THRESH_STAND;

int stand_k_lim = 250; //virtual spring constant opposing motion at virtual joint limit.
int stand_b_lim = 1; //virtual damping constant opposing motion at virtual joint limit.

//VIRTUAL DYNAMIC JOINT PARAMS
float stand_virtualK = VIRTUAL_K_STAND;
float stand_virtualB = VIRTUAL_B_STAND;
float stand_virtualJ = VIRTUAL_J_STAND;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************


//****************************************************************************
// Public Function(s)
//****************************************************************************

//stand_state[0] is position of virtual joint (i.e. desired position of robot)
//stand_state[1] is velocity of virtual joint (not used in current controller, but must be stored for numerical integration)
//stand_state[2] is desired stiffness, derived from average muscle activation.

void updateStandJoint(GainParams* pgains)
{
	get_stand_EMG();
	interpret_stand_EMG(stand_virtualK, stand_virtualB, stand_virtualJ);
	pgains->thetaDes = stand_state[0] * -JOINT_ANGLE_DIR; //flip the convention
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

void get_stand_EMG(void) //Read the EMG signal, rectify, and integrate. Output an integrated signal.
{
	//limit maximum emg_data in case something goes wrong
	for (uint8_t i = 0; i < (sizeof(emg_data)/sizeof(emg_data[0])); i++) {
		if (emg_data[i] > stand_emgInMax) {
			emg_data[i] = stand_emgInMax;
		}
	}

    // Read Seong's variables
	int16_t EMGin_LG = emg_data[0]; //SEONGS BOARD LG_VAR gastroc, 0-10000
	int16_t EMGin_TA = emg_data[1]; //SEONGS BOARD TA_VAR tibialis anterior, 0-10000

	//Apply gains
	stand_EMGavgs[0] = EMGin_LG * stand_gainLG;
	stand_EMGavgs[1] = EMGin_TA * stand_gainTA;
	
	stand_EMGavgs[0] = user_data_1.w[0];
	stand_EMGavgs[1] = user_data_1.w[1];

	//pack for Plan
	rigid1.mn.genVar[0] = stand_EMGavgs[0];
	rigid1.mn.genVar[1] = stand_EMGavgs[1];
}

//updates stand_state[] based on EMG activation
void interpret_stand_EMG (float k, float b, float J)
{
	float LGact = 0;
	float TAact = 0;
	float TALG_diff = 0;
	float Torque_PFDF = 0;
	float activationThresh = stand_dpOnThresh * stand_emgInMax;

	// Calculate LG activation
	if (stand_EMGavgs[0] > activationThresh)
	{
		LGact = ((float)stand_EMGavgs[0] - activationThresh) / (stand_emgInMax - activationThresh); //scaled activation 0-1
	}

	// Calculate TA activation

	if (stand_EMGavgs[1] > activationThresh)
	{
		TAact = ((float)stand_EMGavgs[1] - activationThresh) / (stand_emgInMax - activationThresh);
	}

	// PF/DF Calc
	TALG_diff = TAact - LGact;
	if (TALG_diff < 0) {
		Torque_PFDF = TALG_diff * stand_pfTorqueGain;
	} else {
		Torque_PFDF = TALG_diff * stand_dfTorqueGain;
	}

	//pack for Plan
	rigid1.mn.genVar[2] = TALG_diff*1000;

	// JOINT MODEL
	//The following can break the integration!!!
	if ((TAact > stand_cocontractThresh) && (LGact > stand_cocontractThresh))
	{
		float stiffness = (TAact + LGact)/2.0;
		b = b * (erff(10.0*(stiffness-0.15-stand_cocontractThresh))+1)*20;
	}

	//pack for Plan
	rigid1.mn.genVar[3] = b*1000;

	if (stand_state[0] > stand_equilibriumAngle) {
		k = 0.5*k*(stand_state[0] - stand_equilibriumAngle)/(stand_max_DFangle - stand_equilibriumAngle) + 0.5*k;
	} else {
		k = 0.5*k*(stand_state[0] - stand_equilibriumAngle)/(stand_max_PFangle - stand_equilibriumAngle) + 0.5*k;
	}

	//pack for Plan
	rigid1.mn.genVar[4] = k*1000;

	// forward dynamics equation
	float dtheta = stand_state[1];
	float domega = -k/J * (stand_state[0] - stand_equilibriumAngle) - b/J * stand_state[1] + 1/J * Torque_PFDF; //-restoring stiffness - damping + torque

	// virtual joint physical constraints
	if (stand_state[0] > stand_max_DFangle) //DF is negative
	{
		domega = domega - stand_k_lim/J * (stand_state[0] - stand_max_DFangle) - stand_b_lim/J * stand_state[1]; // Hittin a wall.
	}
	else if (stand_state[0] < stand_max_PFangle)
	{
		domega = domega - stand_k_lim/J * (stand_state[0] - stand_max_PFangle) - stand_b_lim/J * stand_state[1]; // Hittin the other wall.
	}

	//pack for Plan
	rigid1.mn.genVar[5] = dtheta*1000;
	rigid1.mn.genVar[6] = domega*1000;

	// Integration station.
	RK4_SIMPLE_STAND(dtheta, domega, stand_state);
	stand_state[2] = (TAact + LGact)/2.0 * stand_pfdfStiffGain; // stiffness comes from the common signal

	//pack for Plan
	rigid1.mn.genVar[7] = stand_state[0]*1000;
	rigid1.mn.genVar[8] = stand_state[1]*1000;
}

void RK4_SIMPLE_STAND(float d1_dt,float d2_dt, float* cur_state)
{
	float next_state[2];

	float F1 = d1_dt;
	float F2 = d1_dt + .5 * stand_stepsize * F1;
	float F3 = d1_dt + .5 * stand_stepsize * F2;
	float F4 = d1_dt + stand_stepsize * F3;
	next_state[0] = cur_state[0] + (stand_stepsize/6) * (F1 + 2*F2 + 2*F3 + F4);

	F1 = d2_dt;
	F2 = d2_dt + .5 * stand_stepsize * F1;
	F3 = d2_dt + .5 * stand_stepsize * F2;
	F4 = d2_dt + stand_stepsize * F3;
	next_state[1] = cur_state[1] + (stand_stepsize/6) * (F1 + 2*F2 + 2*F3 + F4);

	cur_state[0] = next_state[0];
	cur_state[1] = next_state[1];
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
