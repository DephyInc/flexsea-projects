/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] cmd-ActPack: Custom commands for the Actuator package
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-09-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include "flexsea.h"
#include "flexsea_comm.h"
#include "flexsea_system.h"
#include "flexsea_cmd_user.h"
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#include "cmd-ActPack.h"

#include "stddef.h"

//Execute boards only:
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
#include "main.h"
#include "motor.h"
#include "control.h"
#include "safety.h"
#include "strain.h"
#include "analog.h"
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

//Manage boards only:
#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn-ActPack.h"
#include "mn-MotorControl.h"
#endif	//BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t newActPackRRpacketAvailableFlag = 0;
uint8_t ActPackSys = 0;

//****************************************************************************
// DLL Function(s)
//****************************************************************************

//Poll to see if new data is available:
uint8_t newActPackRRpacketAvailable(void)
{
	uint8_t retVal = newActPackRRpacketAvailableFlag;
	newActPackRRpacketAvailableFlag = 0;
	return retVal;
}

//Get a copy of the latest Rigid values
//void getLastRigidData(struct rigid_s *r){(*r) = rigid1;}

//****************************************************************************
// Private Function Prototype(s)
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Command: CMD_ACTPACK. Type: R/W.
//setGains: KEEP/CHANGE
void tx_cmd_actpack_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
						uint16_t *len, uint8_t offset, uint8_t controller, \
						int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
						int16_t g2, int16_t g3, uint8_t system)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_ACTPACK;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = offset;

	shBuf[index++] = controller;
	SPLIT_32(setpoint, shBuf, &index);
	shBuf[index++] = setGains;
	SPLIT_16(g0, shBuf, &index);
	SPLIT_16(g1, shBuf, &index);
	SPLIT_16(g2, shBuf, &index);
	SPLIT_16(g3, shBuf, &index);
	shBuf[index++] = system;

	//Payload length:
	(*len) = index;
}

//Pack tx_cmd_actpack_rw()
void ptx_cmd_actpack_rw(uint8_t slaveId, uint16_t *numb, uint8_t *commStr, \
						uint8_t offset, uint8_t controller, \
						int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
						int16_t g2, int16_t g3, uint8_t system)
{
	tx_cmd_actpack_rw(TX_N_DEFAULT, offset, controller, setpoint, setGains, \
						g0, g1,	g2, g3, system);
	pack(P_AND_S_DEFAULT, slaveId, NULL, numb, commStr);
}

//Command: CMD_RICNU. Type: R.
//setGains: KEEP/CHANGE
void tx_cmd_actpack_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_ACTPACK;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = 100 + offset;
	//An offset >= 100 means a pure Read, with no writing (not a RW)
	//Fill the other fields with zeros to avoid problems:
	shBuf[index++] = 0;
	SPLIT_32(0, shBuf, &index);
	shBuf[index++] = 0;
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	shBuf[index++] = 0;

	//Payload length:
	(*len) = index;
}

//Only used for replies
void tx_cmd_actpack_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_ACTPACK;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = offset;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		struct rigid_s *ri = &rigid1;

		SPLIT_32((uint32_t)*(ri->ex.enc_ang), shBuf, &index);
		SPLIT_32((uint32_t)(*ri->ex.enc_ang_vel), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ex.joint_ang), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ex.joint_ang_vel), shBuf, &index);
		SPLIT_32((uint32_t)ri->ex.mot_current, shBuf, &index);
		SPLIT_32((uint32_t)ri->ex.mot_acc, shBuf, &index);
		SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
		SPLIT_16((uint16_t)(ctrl[0].current.setpoint_val >> 3), shBuf, &index);
		SPLIT_16((uint16_t)ri->ex.strain, shBuf, &index);
		SPLIT_16((uint16_t)ri->ex.status, shBuf, &index);
		//(28 bytes)

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct rigid_s *ri = &rigid1;
		int16_t *encoder = ri->ex.joint_ang;
		int16_t *encoderVel = ri->ex.joint_ang_vel;

		//Arguments:
		if(offset == 0)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex.enc_ang), shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);
			SPLIT_16((uint16_t)*(encoder), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_current >> 3), shBuf, &index);
			shBuf[index++] = rigid1.re.temp;
			//(27 bytes)
		}
		else if(offset == 1)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex.enc_ang_vel), shBuf, &index);
			SPLIT_16((uint16_t)*(encoderVel), shBuf, &index);
			SPLIT_16(rigid1.re.vb, shBuf, &index);
			SPLIT_16((uint16_t)rigid1.re.current, shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[0]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[1]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[2]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[3]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[4]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[5]), shBuf, &index);
			//(26 bytes)
		}
		else if(offset == 2)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)ri->ex.mot_acc, shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.ctrl.current.setpoint_val >> 3), shBuf, &index);
			SPLIT_16(rigid1.re.status, shBuf, &index);
			SPLIT_16((uint16_t)ri->ex.strain, shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[6]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[7]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[8]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[9]), shBuf, &index);
			//(22 bytes)
		}
		else if(offset == 3)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_16(rigid1.re.vg, shBuf, &index);
			SPLIT_16(rigid1.re.v5, shBuf, &index);
			SPLIT_16((ri->mn.analog[0]), shBuf, &index);
			SPLIT_16((ri->mn.analog[1]), shBuf, &index);
			SPLIT_16((ri->mn.analog[2]), shBuf, &index);
			SPLIT_16((ri->mn.analog[3]), shBuf, &index);
			//(16 bytes)
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Payload length:
	(*len) = index;
}

void rx_cmd_actpack_rw(uint8_t *buf, uint8_t *info)
{
	MultiPacketInfo mInfo;
	fillMultiInfoFromBuf(&mInfo, buf, info);
	mInfo.portOut = info[1];
	rx_multi_cmd_actpack_rw( buf + P_DATA1, &mInfo, tmpPayload, &cmdLen );
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_multi_cmd_actpack_rw(uint8_t *msgBuf, MultiPacketInfo *mInfo, uint8_t *responseBuf, uint16_t* responseLen)
{
	uint16_t index = 0;
	(void)mInfo;

	//Temporary variables
	uint8_t offset = 0;
	uint8_t tmpController = 0, tmpSetGains = 0, tmpSystem = 0;
	int32_t tmpSetpoint = 0;
	int16_t tmpGain[4] = {0,0,0,0};

	//Decode data received:
	index = 0;
	offset = msgBuf[index++];
	tmpController = msgBuf[index++];
	tmpSetpoint = (int32_t)REBUILD_UINT32(msgBuf, &index);
	tmpSetGains = msgBuf[index++];
	tmpGain[0] = (int16_t)REBUILD_UINT16(msgBuf, &index);
	tmpGain[1] = (int16_t)REBUILD_UINT16(msgBuf, &index);
	tmpGain[2] = (int16_t)REBUILD_UINT16(msgBuf, &index);
	tmpGain[3] = (int16_t)REBUILD_UINT16(msgBuf, &index);
	tmpSystem = msgBuf[index++];

	//An offset >= 100 means a pure Read, with no writing (not a RW)
	if(offset < 100)
	{
		#if(defined BOARD_TYPE_FLEXSEA_EXECUTE || defined BOARD_TYPE_FLEXSEA_MANAGE)

			if(offset == 0 || offset == 1)
			{
				//Act on the decoded data:
				rx_cmd_actpack_Action1(tmpController, tmpSetpoint, tmpSetGains, tmpGain[0],
										tmpGain[1], tmpGain[2], tmpGain[3], tmpSystem, offset);
			}

		#else

			(void)tmpController;
			(void)tmpSetGains;
			(void)tmpSetpoint;
			(void)tmpGain;
			(void)tmpSystem;

		#endif //(defined BOARD_TYPE_FLEXSEA_EXECUTE || defined BOARD_TYPE_FLEXSEA_MANAGE)
	}
	else
	{
		offset -= 100;
	}

	//Reply:
	tx_cmd_actpack_w(responseBuf, &cmdCode, &cmdType, responseLen, offset);
}

void rx_cmd_actpack_rr(uint8_t *buf, uint8_t *info)
{
	MultiPacketInfo mInfo;
	fillMultiInfoFromBuf(&mInfo, buf, info);
	mInfo.portOut = info[1];
	rx_multi_cmd_actpack_rr( buf + P_DATA1, &mInfo, tmpPayload, &cmdLen );
}

void rx_multi_cmd_actpack_rr(uint8_t *msgBuf, MultiPacketInfo *mInfo, uint8_t *responseBuf, uint16_t* responseLen)
{
	uint16_t index = 0;
	uint8_t offset = 0;
	uint8_t processed = 0;
	(void)mInfo;	//Unused for now

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		struct rigid_s *ri = &rigid1;
		(void)index;
		(void)msgBuf;
		(void)offset;

	#endif

	#if (defined BOARD_TYPE_FLEXSEA_PLAN || defined BOARD_TYPE_FLEXSEA_MANAGE)
		struct rigid_s *ri = &rigid1;
	#endif

	//This is for communication between Ex and Mn
	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		if(mInfo->xid >= FLEXSEA_EXECUTE_BASE)
		{
			index = 0;
			offset = msgBuf[index++];

			if(offset == 0)
			{
				*(ri->ex.enc_ang) = (int32_t) REBUILD_UINT32(msgBuf, &index);
				*(ri->ex.enc_ang_vel) = (int32_t) REBUILD_UINT32(msgBuf, &index);
				*(ri->ex.joint_ang) = (int16_t) REBUILD_UINT16(msgBuf, &index);
				*(ri->ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->ex.mot_current = (int32_t) REBUILD_UINT32(msgBuf, &index);
				ri->ex.mot_acc = (int32_t) REBUILD_UINT32(msgBuf, &index);
				ri->ex.mot_volt = (int32_t) ((int16_t)REBUILD_UINT16(msgBuf, &index) << 3);
				ri->ex.ctrl.current.setpoint_val = (int32_t) ((int16_t)REBUILD_UINT16(msgBuf, &index) << 3);
				ri->ex.strain = REBUILD_UINT16(msgBuf, &index);
				ri->ex.status = REBUILD_UINT16(msgBuf, &index);
			}

			processed = 1;
		}

	#endif

	//This used to be only on Plan, but now that Manages can communicate we expanded:
	#if (defined BOARD_TYPE_FLEXSEA_PLAN || defined BOARD_TYPE_FLEXSEA_MANAGE)

		if(!processed)
		{
			#ifdef SC_PRJ_EN_RI2
			if(mInfo->xid == FLEXSEA_MANAGE_2)
			{
				ri = &rigid2;
			}
			#endif
			index = 0;
			offset = msgBuf[index++];

			if(offset == 0)
			{
				ri->ctrl.timestamp = REBUILD_UINT32(msgBuf, &index);
				*(ri->ex.enc_ang) = (int32_t) REBUILD_UINT32(msgBuf, &index);
				ri->mn.gyro.x = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->mn.gyro.y = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->mn.gyro.z = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->mn.accel.x = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->mn.accel.y = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->mn.accel.z = (int16_t) REBUILD_UINT16(msgBuf, &index);
				*(ri->ex.joint_ang) = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->ex.mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(msgBuf, &index)) << 3);
				ri->ex.mot_current = (int32_t)(((int16_t)REBUILD_UINT16(msgBuf, &index)) << 3);
				ri->re.temp = msgBuf[index++];
				//(27 bytes)
			}
			else if(offset == 1)
			{
				ri->ctrl.timestamp = REBUILD_UINT32(msgBuf, &index);
				*(ri->ex.enc_ang_vel) = (int32_t) REBUILD_UINT32(msgBuf, &index);
				*(ri->ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(msgBuf, &index);
				ri->re.vb = REBUILD_UINT16(msgBuf, &index);
				ri->re.current = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[0] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[1] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[2] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[3] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[4] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[5] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				//(26 bytes)

				//In some cases genVar contains Strain data:
				strain1.ch[0].strain_filtered = ri->mn.genVar[0];
				strain1.ch[1].strain_filtered = ri->mn.genVar[1];
				strain1.ch[2].strain_filtered = ri->mn.genVar[2];
				strain1.ch[3].strain_filtered = ri->mn.genVar[3];
				strain1.ch[4].strain_filtered = ri->mn.genVar[4];
				strain1.ch[5].strain_filtered = ri->mn.genVar[5];
				strain1.preDecoded = 1;
			}
			else if(offset == 2)
			{
				ri->ctrl.timestamp = REBUILD_UINT32(msgBuf, &index);
				ri->ex.mot_acc = (int32_t) REBUILD_UINT32(msgBuf, &index);
				ri->ex.ctrl.current.setpoint_val = (int32_t)(((int16_t)REBUILD_UINT16(msgBuf, &index)) << 3);
				ri->re.status = REBUILD_UINT16(msgBuf, &index);
				ri->ex.strain = REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[6] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[7] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[8] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				ri->mn.genVar[9] = (int16_t)REBUILD_UINT16(msgBuf, &index);
				//(22 bytes)
			}
			else if(offset == 3)
			{
				ri->ctrl.timestamp = REBUILD_UINT32(msgBuf, &index);
				ri->re.vg = REBUILD_UINT16(msgBuf, &index);
				ri->re.v5 = REBUILD_UINT16(msgBuf, &index);
				ri->mn.analog[0] = REBUILD_UINT16(msgBuf, &index);
				ri->mn.analog[1] = REBUILD_UINT16(msgBuf, &index);
				ri->mn.analog[2] = REBUILD_UINT16(msgBuf, &index);
				ri->mn.analog[3] = REBUILD_UINT16(msgBuf, &index);
				//(16 bytes)
			}
			else
			{
				//...
			}
		}

	#endif	//BOARD_TYPE_FLEXSEA_PLAN

	newActPackRRpacketAvailableFlag = 1;
	
	#if(defined BOARD_TYPE_FLEXSEA_PLAN || defined BOARD_TYPE_FLEXSEA_MANAGE || \
		defined BOARD_TYPE_FLEXSEA_EXECUTE)
	ri->lastOffsetDecoded = offset;
	#endif
}



//****************************************************************************
// Private Function(s)
//****************************************************************************

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
//Command = rx_cmd_actpack, section = READ
void rx_cmd_actpack_Action1(uint8_t controller, int32_t setpoint, uint8_t setGains,
						int16_t g0,	int16_t g1,	int16_t g2, int16_t g3, uint8_t system, uint8_t ch)
{
	(void) system;

	//Update controller (if needed):
	control_strategy(controller, ch);

	//Only change the setpoint if we are in current control mode:
	if(ctrl[ch].active_ctrl == CTRL_CURRENT)
	{
		ctrl[ch].current.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl[ch].current.gain.g0 = g0;
			ctrl[ch].current.gain.g1 = g1;
			ctrl[ch].current.error_sum = 0;
		}
	}
	else if(ctrl[ch].active_ctrl == CTRL_OPEN)
	{
		setMotorVoltage(setpoint, ch);
	}
	else if(ctrl[ch].active_ctrl == CTRL_POSITION)
	{
		ctrl[ch].position.setp = setpoint;
		if (setGains == CHANGE)
		{
			ctrl[ch].position.gain.g0 = g0;
			ctrl[ch].position.gain.g1 = g1;
			ctrl[ch].position.gain.g2 = g2;
			//ctrl[ch].position.error_sum = 0;
		}
		ctrl[ch].position.error_sum = 0;
	}
	else if (ctrl[ch].active_ctrl == CTRL_IMPEDANCE)
	{
		ctrl[ch].impedance.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl[ch].impedance.gain.g0 = g0;
			ctrl[ch].impedance.gain.g1 = g1;
			ctrl[ch].current.gain.g0 = g2;
			ctrl[ch].current.gain.g1 = g3;
			ctrl[ch].current.error_sum = 0;
			ctrl[ch].impedance.error_sum = 0;
		}
	}
}
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

#ifdef BOARD_TYPE_FLEXSEA_MANAGE
//Command = rx_cmd_actpack, section = READ
void rx_cmd_actpack_Action1(uint8_t controller, int32_t setpoint, uint8_t setGains,
						int16_t g0,	int16_t g1,	int16_t g2, int16_t g3, uint8_t system, uint8_t ch)
{
	//Update controller (if needed):
	setControlMode(controller, ch);

	//Only change the setpoint if we are in current control mode:
	if(ctrl[ch].active_ctrl == CTRL_CURRENT)
	{
		ctrl[ch].current.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl[ch].current.gain.g0 = g0;
			ctrl[ch].current.gain.g1 = g1;
			//Copy to writeEx:
			setControlGains(g0, g1, g2, g3, ch);
		}
		//Copy to writeEx:
		setMotorCurrent(setpoint, ch);
	}
	else if(ctrl[ch].active_ctrl == CTRL_OPEN)
	{
		setMotorVoltage(setpoint, ch);
	}
	else if(ctrl[ch].active_ctrl == CTRL_POSITION)
	{
		ctrl[ch].position.setp = setpoint;
		if (setGains == CHANGE)
		{
			ctrl[ch].position.gain.g0 = g0;
			ctrl[ch].position.gain.g1 = g1;
			ctrl[ch].position.gain.g2 = g2;
			//Copy to writeEx:
			setControlGains(g0, g1, g2, g3, ch);
		}
		//Copy to writeEx:
		setMotorPosition(setpoint, ch);
	}
	else if (ctrl[ch].active_ctrl == CTRL_IMPEDANCE)
	{
		ctrl[ch].impedance.setpoint_val = setpoint;

		if (setGains == CHANGE)
		{
			ctrl[ch].impedance.gain.g0 = g0;
			ctrl[ch].impedance.gain.g1 = g1;
			ctrl[ch].current.gain.g0 = g2;
			ctrl[ch].current.gain.g1 = g3;
			//Copy to writeEx:
			setControlGains(g0, g1, g2, g3, ch);
		}
		//Copy to writeEx:
		setMotorPosition(setpoint, ch);
	}

	ActPackSys = system;
}
#endif	//BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif
