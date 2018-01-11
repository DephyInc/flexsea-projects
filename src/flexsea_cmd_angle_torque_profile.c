/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-system' System commands & functions
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY{} without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] flexsea_cmd_angle_torque_profile: commands specific to the angle-torque profile that exo tries to achieve
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-04-04 | dweisdorf | Initial GPL-3.0 release
	*
****************************************************************************/

#include "flexsea_cmd_angle_torque_profile.h"
#include "flexsea_cmd_user.h"
#include "flexsea_comm_def.h"
#include "flexsea_comm.h"
#include "flexsea_system.h"
#include "dynamic_user_structs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ANKLE_TORQUE_MODULE_NUM_POINTS 6

int16_t torqueArray[ANKLE_TORQUE_MODULE_NUM_POINTS] = {0};
int16_t angleArray[ANKLE_TORQUE_MODULE_NUM_POINTS] = {0};

int16_t *atProfile_torques = &torqueArray[0];
int16_t *atProfile_angles = &angleArray[0];

uint8_t atProfile_newProfileFlag = 0;
uint8_t atProfile_newDataFlag = 0;

#define BUFFER_SIZE 10
int16_t torqueBuffer[BUFFER_SIZE] = {0};
int16_t angleBuffer[BUFFER_SIZE] = {0};

int16_t* torqueBuf = &torqueBuffer[0];
int16_t* angleBuf = &angleBuffer[0];
int indexOfLastBuffered = -1;

const uint8_t profilePointFlag =	0x01;
const uint8_t dataFlag =			0x00;

void tx_profile(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, uint8_t includeFlag);

/* Initializes part of the array of function pointers which determines which
	function to call upon receiving a message
*/
void init_flexsea_payload_ptr_ankleTorqueProfile(void)
{
	flexsea_payload_ptr[CMD_ANGLE_TORQUE_PROFILE][RX_PTYPE_READ] = &rx_cmd_ankleTorqueProfile_r;
	flexsea_payload_ptr[CMD_ANGLE_TORQUE_PROFILE][RX_PTYPE_WRITE] = &rx_cmd_ankleTorqueProfile_rw;
	flexsea_payload_ptr[CMD_ANGLE_TORQUE_PROFILE][RX_PTYPE_REPLY] = &rx_cmd_ankleTorqueProfile_rr;
}

void tx_cmd_ankleTorqueProfile_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, uint8_t requestProfile)
{
	*cmd = CMD_ANGLE_TORQUE_PROFILE;
	*cmdType = CMD_READ;

	uint16_t index = 0;
	if(requestProfile)
		shBuf[index++] = profilePointFlag;
	else
		shBuf[index++] = dataFlag;

	*len = index;
}

/* Called by master to send a message to the slave, attempting to inititiate a
	calibration procedure specified by 'calibrationMode'. Slave will not respond.
	TODO: rename this to 'write'
*/
void tx_cmd_ankleTorqueProfile_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
						uint16_t *len)
{
	tx_profile(shBuf, cmd, cmdType, len, 0);
}

int unpackProfile(uint8_t* buf, int16_t* torques, int16_t* angles, int n)
{
	uint16_t index = 0;
	int i;
	for(i = 0; i < n; i++)
		torques[i] = (int16_t)(REBUILD_UINT16(buf, &index));

	for(i = 0; i < n; i++)
		angles[i] = (int16_t)(REBUILD_UINT16(buf, &index));

	return index;
}

void rx_cmd_ankleTorqueProfile_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = P_DATA1;
	uint8_t flag = buf[index++];
	if(flag == profilePointFlag)
	{
		unpackProfile(buf+index, atProfile_torques, atProfile_angles, ANKLE_TORQUE_MODULE_NUM_POINTS);
		atProfile_newProfileFlag = 1;
	}
	else if(flag == dataFlag)
	{
		indexOfLastBuffered = (indexOfLastBuffered + 1) % BUFFER_SIZE;

		angleBuffer[indexOfLastBuffered] = (int16_t)REBUILD_UINT16(buf, &index);
		torqueBuffer[indexOfLastBuffered] = (int16_t)REBUILD_UINT16(buf, &index);
		atProfile_newDataFlag = 1;
	}
	(void)info;
}

float quickSin(int y)
{
	y %= 360;
	float x = y > 180 ? -360+y : y;

	x = x * 314 / 18000;

	float x3 = x*x*x;
	float x5 = x3*x*x;
	float x7 = x5*x*x;
	float x9 = x7*x*x;

	return x - x3/6 + x5/120 - x7/5040 + x9/362880;
}

void tx_response_data(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len)
{
	uint16_t index = 0;
	*cmd = CMD_ANGLE_TORQUE_PROFILE;
	*cmdType = CMD_WRITE;

	shBuf[index++] = dataFlag;

	#if((defined BOARD_TYPE_FLEXSEA_EXECUTE))
		//SPLIT_16(dynamicUserData.ank_ang_control, shBuf, &index);
		//SPLIT_16(dynamicUserData.torq_ang_output, shBuf, &index);
		//ToDo fix this - what are they supposed to be? The variables above do not exist...
		SPLIT_16(0, shBuf, &index);
		SPLIT_16(0, shBuf, &index);
		#warning "Torque Profile won't work!"
	#else
		SPLIT_16(0, shBuf, &index);
		SPLIT_16(0, shBuf, &index);
	#endif

	*len = index;
}

void tx_profile(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, uint16_t *len, uint8_t includeFlag)
{
	*cmd = CMD_ANGLE_TORQUE_PROFILE;
	*cmdType = CMD_WRITE;

	uint16_t index = 0;

	if(includeFlag)
		shBuf[index++] = profilePointFlag;

	int i;
	for(i = 0; i < ANKLE_TORQUE_MODULE_NUM_POINTS; i++)
		SPLIT_16(torqueArray[i], shBuf, &index);

	for(i = 0; i < ANKLE_TORQUE_MODULE_NUM_POINTS; i++)
		SPLIT_16(angleArray[i], shBuf, &index);

	*len = index;
}
void rx_cmd_ankleTorqueProfile_rw(uint8_t *buf, uint8_t *info)
{
	int index = P_DATA1;
	unpackProfile(&buf[index], atProfile_torques, atProfile_angles, ANKLE_TORQUE_MODULE_NUM_POINTS);

	//Do any min - max limiting here

	//respond with the current state
	tx_profile(TX_N_DEFAULT, 1);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);

	//(void)info;
}

// We simply respond with current ankle torque relationship
void rx_cmd_ankleTorqueProfile_r(uint8_t *buf, uint8_t *info)
{
	//(void)info;

	if(buf[P_DATA1] == profilePointFlag)
		tx_profile(TX_N_DEFAULT, 1);
	else
		tx_response_data(TX_N_DEFAULT);

	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}
#ifdef __cplusplus
}
#endif
