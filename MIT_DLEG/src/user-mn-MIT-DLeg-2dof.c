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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
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

#ifdef INCLUDE_UPROJ_MIT_DLEG
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include <user-mn-MIT-DLeg-2dof.h>
#include "state_machine.h"
#include "state_variables.h"
#include "user-mn-ActPack.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t mitDlegInfo[2] = {PORT_RS485_2, PORT_RS485_2};

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MIT_DLeg(void)
{

}

//MIT DLeg Finite State Machine.
//Call this function in one of the main while time slots.
void MIT_DLeg_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t time = 0, state = 0;

    //Increment time (1 tick = 1ms)
    time++;

	//begin shutoff safety check
    if (safetyShutoff()) {
    	//bypasses the switch statement if return true
    	return;
    }

    //begin main FSM
	switch(state)
	{
		case 0:
			//Same power-on delay as FSM2:
			if(time >= AP_FSM2_POWER_ON_DELAY)
			{
				stateMachine.current_state = STATE_IDLE;
				state = 1;
				time = 0;
			}

			break;

		//find poles during startup. Requires 60 s
		case 1:
			if(findPoles()) {
				stateMachine.current_state = STATE_INIT;
				//toDo global updatable vars for current gains
				setControlMode(CTRL_CURRENT);
				setMotorCurrent(0);
				state = 2;
				time = 0;
			}

			break;

		//main walking FSM
		case 2:
			//need to declare new scope here(brackets) to initialize vars
			{
				static float* ptorqueDes;
				static float* pcurrentDes;

				//return torqueDes from FSM
				runFlatGroundFSM(ptorqueDes);

				//convert torque to current and clamp if necessary
				*pcurrentDes = calcCurrent(*ptorqueDes);

				//send current command
				setMotorCurrent((int32_t) (*pcurrentDes * 1000));

				break;
			}

        default:
			//Handle exceptions here
			break;

	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}


//Second state machine for the DLeg project
void MIT_DLeg_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

		//Currently unused - we use ActPack's FSM2 for comm

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

/** Safety Shutoff
    Sets motor current to 0 if something is really wrong.
    Checked before entering FSM.
    Return boolean indicates whether safetyShutoff() tripped.

    @return active or passed
*/
int8_t safetyShutoff(void) {
	//toDo variableNames_ to be replaced by real measured values later
	float force_ = 0;

	//over temperature shutoff
	if (rigid1.mn.mot_temp > MOTOR_TEMP_LIMIT || rigid1.re.temp > BOARD_TEMP_LIMIT) {
		setControlMode(CTRL_CURRENT);
		setMotorCurrent(0);
		return 1;
	}

	//over force limit (something must be really wrong)
	if (abs(force_) > FORCE_LIMIT) {
		setControlMode(CTRL_CURRENT);
		setMotorCurrent(0);
		return 1;
	}

	return 0;
}

/** Current Clamp
	Scales desired output current to safe numbers.
    Checked after calculating current desired from torque desired.

    @param pcurrentDes
*/
void clampCurrent(float* pcurrentDes) {

	int8_t jointAngle_ = 0; //replace

	//check joint angles
	//toDo BE SURE CURRENT DIRECTIONS ARE CORRECT!!!
	if (jointAngle_ <= JOINT_MIN) {
		if (*pcurrentDes < 0) {
			*pcurrentDes = 0;
		}
	} else if (jointAngle_ >= JOINT_MAX) {
		if (*pcurrentDes > 0) {
			*pcurrentDes = 0;
		}
	}

	//check current limits
	if (abs(*pcurrentDes) > CURRENT_LIMIT) {
		*pcurrentDes = (*pcurrentDes < 0) ? -CURRENT_LIMIT : CURRENT_LIMIT;
	} //should there be thermal throttling also?



	//final current scaling for testing purposes
	//CURRENT_SCALAR == 1 at full power
	*pcurrentDes *= CURRENT_SCALAR;
}

/** Desired Current from Torque
	Walking FSM torque -> current
    Can incorporate transmission and spring functions.

    @param torqueDes
    @return outputCurrent
*/
float calcCurrent(float torqueDes) {

	float currentDes = 0;

	//toDo controls stuff here based on motor


	//clamp if necessary
	clampCurrent(&currentDes);

	return currentDes;
}

/** Find Poles
	Startup procedure for locating motor poles.
    Runs in about 60 s.

    @return success or failure
*/
int8_t findPoles(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 10)
			{
				polesState = 1;
				timer = 0;
			}
			return 0;

			break;

		case 1:
			//Send Find Poles command:

			tx_cmd_calibration_mode_rw(TX_N_DEFAULT, CALIBRATION_FIND_POLES);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, mitDlegInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;
			return 0;

			break;

		case 2:
			//Wait 60s... (conservative)

			if(timer >= 60*SECONDS)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				return 1;
			}
			return 0;

			break;

		default:

			return 0;

			break;
	}

	return 0;
}

void openSpeedFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static uint8_t fsm1State = 0;

	switch(fsm1State)
	{
		case 0:
			setControlMode(CTRL_OPEN);
			setMotorVoltage(0);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorVoltage(0);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorVoltage(1000);
			break;
	}
}

void twoPositionFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = *(rigid1.ex.enc_ang);
				fsm1State = 0;
			}
			break;
		case 0:
			setControlMode(CTRL_POSITION);
			setControlGains(20, 6, 0, 0);	//kp = 20, ki = 6
			setMotorPosition(initPos);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorPosition(initPos + 10000);
			break;
		case 2:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorPosition(initPos);
			break;
	}
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
