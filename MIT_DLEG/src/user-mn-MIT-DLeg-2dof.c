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
	[Contributors] Tony Shu, tony shu at mit dot edu, Matthew Carney mcarney at mit dot edu
*****************************************************************************
	[This file] user-mn-MIT_DLeg_2dof: User code running on Manage
*****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "state_machine.h"
#include "state_variables.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"
//#include "filters.h"
#include "flexsea_user_structs.h"
#include <flexsea_comm.h>
#include <math.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

//Variables which aren't static may be updated by Plan in the future

uint8_t mitDlegInfo[2] = {PORT_RS485_2, PORT_RS485_2};

//SAFETY FLAGS - in addition to enum, so can be cleared but don't lose other flags that may exist.
static int8_t isSafetyFlag = 0;
static int8_t isAngleLimit = 0;
static int8_t isTorqueLimit = 0;
static int8_t isTempLimit = 0;

int8_t isEnabledUpdateSensors = 0;
int16_t fsm1State = -2;
float currentScalar = CURRENT_SCALAR_INIT;
int32_t currentOpLimit = CURRENT_LIMIT_INIT; 	//operational limit for current.

//torque gain values
float torqueKp = TORQ_KP_INIT;
float torqueKi = TORQ_KI_INIT;
float torqueKd = TORQ_KD_INIT;

//current gain values
int16_t currentKp = ACTRL_I_KP_INIT;
int16_t currentKi = ACTRL_I_KI_INIT;
int16_t currentKd = ACTRL_I_KD_INIT;

//const vars taken from defines (done to speed up computation time)
static const float angleUnit    = ANG_UNIT;
static const float jointHSMin   = JOINT_HS_MIN;
static const float jointHSMax   = JOINT_HS_MAX;
static const float jointZeroAbs = JOINT_ZERO_ABS;
static const float jointZero	= JOINT_ZERO;

static const float forceMaxTicks = FORCE_MAX_TICKS;
static const float forceMinTicks = FORCE_MIN_TICKS;
static const float forcePerTick  = FORCE_PER_TICK;

static const float nScrew = N_SCREW;

static const float jointMinSoft = JOINT_MIN_SOFT;
static const float jointMaxSoft = JOINT_MAX_SOFT;

//struct act_s act1;		//actuator sensor structure declared extern in flexsea_user_structs
				//defined in state_machine.c

struct diffarr_s jnt_ang_clks;		//maybe used for velocity and accel calcs.

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

    static uint32_t time = 0;

    //Increment time (1 tick = 1ms nominally; need to confirm)
    time++;

    //begin main FSM
	switch(fsm1State)
	{
		case -2:
			stateMachine.current_state = STATE_IDLE;
			//Same power-on delay as FSM2:
			if(time >= AP_FSM2_POWER_ON_DELAY + 3*SECONDS) {
				fsm1State = -1;
				time = 0;
			}

			break;

		case -1:
			stateMachine.current_state = STATE_INIT;
			//turned off for testing without Motor usage
			if(findPoles()) {
				mit_init_current_controller();		//initialize Current Controller with gains
				fsm1State = 0;
				time = 0;
			}

			break;

		case 0:
			//sensor update happens in mainFSM2(void) in main_fsm.c
			isEnabledUpdateSensors = 1;
			//reserve for additional initialization

			fsm1State = 1;
			time = 0;

			break;

		case 1:
			{
				static float* ptorqueDes;

				//populate rigid1.mn.genVars to send to Plan
				packRigidVars(&act1);

				//begin safety check
//			    if (safetyShutoff()) {
//			    	/*motor behavior changes based on failure mode.
//			    	  Bypasses the switch statement if return true
//			    	  but sensors check still runs and has a chance
//			    	  to allow code to move past this block.
//			    	  Only update the walking FSM, but don't output torque.
//			    	*/
//			    	runFlatGroundFSM(ptorqueDes);
//
//			    	return;
//
//			    } else {
//
//			    	runFlatGroundFSM(ptorqueDes);
//					setMotorTorque(&act1, *ptorqueDes);
//	//				twoTorqueFSM( &act1);
//			    }

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
/*
 * Check for safety flags, and act on them.
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
int8_t safetyShutoff(void) {

	switch(isSafetyFlag)

	{
		case SAFETY_OK:

			return 0;

		case SAFETY_ANGLE:
			//check if flag is not still active to be released, else do something about problem.
			if(!isAngleLimit) {
				isSafetyFlag = SAFETY_OK;
				break;
			} else {
				setMotorCurrent(0); // turn off motor. might need something better than this.
			}

			return 1;
			
		case SAFETY_TORQUE:

			//check if flag is not still active to be released, else do something about problem.
			if(!isTorqueLimit) {
				isSafetyFlag = SAFETY_OK;
				break;
			} else {
				setMotorTorque(&act1, 0); //run this in order to update torque genVars sent to Plan
			}

			return 1;

		case SAFETY_TEMP:
			//check if flag is not still active to be released, else do something about problem.
			if( !isTempLimit ) {
				currentOpLimit = CURRENT_LIMIT_INIT;		// return to full power todo: may want to gradually increase
				isSafetyFlag = SAFETY_OK;
				break;
			} else {
				if (currentOpLimit > 0) {
					currentOpLimit--;	//reduce current limit every cycle until we cool down.
				}
			}

			return 0; //continue running FSM

		default:
			return 1;
	}


	return 0;
}

/*
 * Collect all sensor values and update the actuator structure.
 * Throws safety flags on Joint Angle, Joint Torque, since these functions look at transformed values.
 */
void updateSensorValues(struct act_s *actx)
{
	static float* pjointKinematic;

	pjointKinematic = getJointAngleKinematic();

	actx->jointAngle = *(pjointKinematic + 0);
	actx->jointAngleDegrees = actx->jointAngle * 360/angleUnit;
	actx->jointVel = *(pjointKinematic + 1);
	actx->jointVelDegrees = actx->jointVel * 360/angleUnit;
	actx->jointAcc = *(pjointKinematic + 2);
	actx->linkageMomentArm = getLinkageMomentArm(actx->jointAngle);
	actx->axialForce = getAxialForce();
	actx->jointTorque = getJointTorque(&act1);

	actx->motorVel =  *rigid1.ex.enc_ang_vel / 16.384 * angleUnit;	// rad/s
	actx->motorAcc = rigid1.ex.mot_acc;	// rad/s/s

	actx->regTemp = rigid1.re.temp;
	actx->motTemp = getMotorTempSensor();
	actx->motCurr = rigid1.ex.mot_current;
	actx->currentOpLimit = currentOpLimit; // throttled mA

	actx->safetyFlag = isSafetyFlag;

	if(actx->regTemp > PCB_TEMP_LIMIT_INIT || actx->motTemp > MOTOR_TEMP_LIMIT_INIT)
	{
		isSafetyFlag = SAFETY_TEMP;
		isTempLimit = 1;
	} else {
		isTempLimit = 0;
	}

}

//The ADC reads the motor Temp sensor - MCP9700 T0-92. This function converts the result to degrees.
int16_t getMotorTempSensor(void)
{
	static int16_t mot_temp = 0;
//	mot_temp = ((VSENSE_SLOPE * (rigid1.mn.analog[0] - V25_TICKS) \
//					/ TICK_TO_V) + 25);
	mot_temp = (rigid1.mn.analog[0] * (3.3/4096) - 500) / 10; 	//celsius
	rigid1.mn.mot_temp = mot_temp;

	return mot_temp;
}

//
/*
 * Output joint angle, vel, accel in ANG_UNIT, measured from joint zero,
 * Return:
 * 		joint[0] = angle
 * 		joint[1] = velocity
 * 		joint[2] = acceleration
 * 		todo: pass a reference to the act_s structure to set flags.
 */
float* getJointAngleKinematic( void )
{
	static int32_t jointAngleCnts = 0;
	static float last_jointVel = 0;
	static int32_t jointAngleCntsAbsolute = 0;
	static float jointAngleAbsolute = 0;
	static float joint[3] = {0, 0, 0};

	//ANGLE
	//Configuration orientation
	jointAngleCnts = JOINT_ANGLE_DIR * ( jointZero + JOINT_ENC_DIR * (*(rigid1.ex.joint_ang)) );
	joint[0] = jointAngleCnts  * (angleUnit)/JOINT_CPR;

	//Absolute orientation to evaluate against soft-limits
	jointAngleCntsAbsolute = JOINT_ANGLE_DIR * ( jointZeroAbs + JOINT_ENC_DIR * (*(rigid1.ex.joint_ang)) );
	jointAngleAbsolute = jointAngleCnts  * (angleUnit)/JOINT_CPR;

	//Check angle limits, raise flag for safety check
	if(jointAngleAbsolute <= jointMinSoft  || jointAngleAbsolute >= jointMaxSoft) {
		isSafetyFlag = SAFETY_ANGLE;
		isAngleLimit = 1;		//these are all redundant, choose if we want the struct thing.
	} else {
		isAngleLimit = 0;
	}

	//VELOCITY
	joint[1] = 	*(rigid1.ex.joint_ang_vel) * (angleUnit)/JOINT_CPR * SECONDS;

	//ACCEL  -- todo: check to see if this works
	joint[2] = (( joint[1] - last_jointVel )) * (angleUnit)/JOINT_CPR * SECONDS;
	last_jointVel = joint[1];

	return joint;
}


// Output axial force on screw, Returns [Newtons]
float getAxialForce(void)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	static uint16_t strainReading = 0;
	static uint16_t tareOffset = 0;
	static float axialForce = 0;

	strainReading = (rigid1.ex.strain);

	switch(tareState)
	{
		case -1:
			//Tare the balance the first time this gets called.
			timer++;

			if(timer > 250) {
				strainReading = (rigid1.ex.strain);
				tareOffset = strainReading;
				tareState = 0;
			}

			break;

		case 0:
			// Looks correct with simple weight, need to test with a scale
			axialForce =  FORCE_DIR * (strainReading - tareOffset) * forcePerTick;

			break;

		default:
			//problem occurred
			break;
	}

	return axialForce;
}

// Linear Actuator Actual Moment Arm,
// input( jointAngle, theta [rad] )
// return moment arm projected length  [m]
float getLinkageMomentArm(float theta)
{
	static float a = 0, b = 0, T = 0, F = 0;
	static float A = 0, c = 0, r = 0, C_ang = 0;
//	theta_r = ANG_UNIT % 360 ? (theta*M_PI/180) : theta; 	// convert deg to radians if necessary.
//	theta_r = theta * M_PI / 180;	// convert deg to radians.

    const float t = 47; 		// [mm] tibial offset
    const float t_k = 140; 	// [mm] offset from knee along tibia
    const float f = 39;  	// [mm] femur offset
    const float f_k = 18;	// [mm] offset from knee along femur

    const float aIn = t*t + t_k*t_k;
    a = sqrt(aIn);
    const float bIn = f*f + f_k*f_k;
    b = sqrt(bIn);

    const float Tin = t/t_k;
    T = atan(Tin);
    const float Fin = f/f_k;
    F = atan(Fin);

    C_ang = M_PI - theta - (T + F); 	// angle
    c = sqrt( a*a + b*b - 2*a*b*cos(C_ang) );  // length of actuator from pivot to output
    A = acos( (a*a - (c*c + b*b)) / (-2*b*c) );
    r = b * sin(A);

    return r/1000;
}

/*
 *  Determine torque at joint due to moment arm and axial force
 *  input:	struct act_s
 *  return: joint torque [Nm]
 */
float getJointTorque(struct act_s *actx)
{
	static float torque = 0;

	torque = actx->linkageMomentArm * actx->axialForce;

	if(torque >= ABS_TORQUE_LIMIT_INIT || torque <= -ABS_TORQUE_LIMIT_INIT) {
		isSafetyFlag = SAFETY_TORQUE;
		isTorqueLimit = 1;
	} else {
		isTorqueLimit = 0;
	}

	return torque;
}

/*
 * Calculate required motor torque, based on joint torque
 * input:	*actx,  actuator structure reference
 * 			tor_d, 	desired torque at joint [Nm]
 * return:	set motor torque
 * 			Motor Torque request, or maybe current
 */
void setMotorTorque(struct act_s *actx, float tau_des)
{
	static int8_t time = 1; 		// ms
	static float N = 1;				// moment arm [m]
	static float tau_meas = 0;  	//joint torque reflected to motor.
	static float tau_err = 0, tau_err_last = 0;
	static float tau_err_dot = 0, tau_err_int = 0;
	static float tau_motor = 0;		// motor torque signal
	static int32_t dtheta_m = 0, ddtheta_m = 0;	//motor vel, accel
	static int32_t I = 0;			// motor current signal

	N = actx->linkageMomentArm * nScrew;
	dtheta_m = actx->motorVel;
	ddtheta_m = actx->motorAcc;


	// todo: better fidelity may be had if we modeled N_ETA as a function of torque, long term goal, if necessary
	tau_meas =  actx->jointTorque / (N) * 1000;	// measured torque reflected to motor [mNm]
	tau_des = tau_des / (N*N_ETA) *1000;					// desired joint torque, reflected to motor [mNm]

	//output genVars for ActPack monitoring
	rigid1.mn.userVar[5] = tau_meas;
	rigid1.mn.userVar[6] = tau_des;

	tau_err = (tau_des - tau_meas);
	tau_err_dot = (tau_err - tau_err_last)/time;
	tau_err_int = tau_err_int + tau_err;
	tau_err_last = tau_err;

	//PID around motor torque
	tau_motor = tau_err * torqueKp + (tau_err_dot) * torqueKd + (tau_err_int) * torqueKi;

	I = 1 / MOT_KT * ( (int32_t) tau_motor + (MOT_J + MOT_TRANS)*ddtheta_m + MOT_B*dtheta_m);		// + (J_rotor + J_screw)*ddtheta_m + B*dtheta_m
	//I think I needs to be scaled to mA, but not sure yet.

	//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
	if(I > currentOpLimit )
	{
		I = currentOpLimit;
	} else if (I < -currentOpLimit)
	{
		I = -currentOpLimit;
	}

	actx->desiredCurrent = (int32_t) (I * currentScalar); // demanded mA
	setMotorCurrent(actx->desiredCurrent);				// send current command to comm buffer to Execute
}

/*
 * Calculate required motor torque, based on joint torque and using feedforward of desired torque from motor
 * input:	*actx,  actuator structure reference
 * 			tor_d, 	desired torque at joint [Nm]
 */
void setMotorTorqueFF(struct act_s *actx, float tau_des)
{
	float N = 1;				// moment arm [m]
	float tau_meas = 0;  	//joint torque reflected to motor.
	float tau_desired = 0;
	static int32_t prev_tau_des = 0;
	static int32_t prev_tau_error = 0;
	int32_t tau_error = 0;
	int32_t dtau_error = 0;
	static int32_t itau_error = 0;
	int32_t curr_error = 0;
	float tau_comp = 0;
	float tauFF = 0;
	int32_t currFF = 0;
	int32_t dtheta_m = 0, ddtheta_m = 0;	//motor vel, accel
	int32_t I = 0;			// motor current signal

	N = actx->linkageMomentArm * nScrew;
	dtheta_m = actx->motorVel;
	ddtheta_m = actx->motorAcc;


	// todo: better fidelity may be had if we modeled N_ETA as a function of torque, long term goal, if necessary
	tau_meas =  actx->jointTorque / (N) * 1000;	// measured torque reflected to motor [mNm]
	tau_desired = tau_des / (N*N_ETA) *1000;		// desired joint torque, reflected to motor [mNm]


	//calculate feedforward current based on desired - actual joint torque
	tauFF = tau_desired - tau_meas;
	currFF = tauFF / MOT_KT; //feed forward current in mA

	//account for deadzone current
	if (abs(dtheta_m) < 2 && currFF < 0) {
		currFF -= MOT_DEAD_CURR;
	} else if (abs(dtheta_m) < 2 && currFF > 0) {
		currFF += MOT_DEAD_CURR;
	}

	//PID around error
	tau_error = prev_tau_des - tau_meas; //nMn
	itau_error += tau_error;
	dtau_error = tau_error - prev_tau_error; // mNm/s

	tau_comp = tau_error*FF_KP + itau_error*FF_KI + dtau_error*FF_KD;

	//combined current demand
	I = currFF + (1/MOT_KT) * (tau_comp + (MOT_J + MOT_TRANS)*ddtheta_m + MOT_B*dtheta_m);		// + (J_rotor + J_screw)*ddtheta_m + B*dtheta_m


	prev_tau_des = tau_desired;
	prev_tau_error = tau_error;

	//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
	if(I > currentOpLimit )
	{
		I = currentOpLimit;
	} else if (I < -currentOpLimit)
	{
		I = -currentOpLimit;
	}

	actx->desiredCurrent = (int32_t) (I * currentScalar); // demanded mA
	setMotorCurrent(actx->desiredCurrent);				// send current command to comm buffer to Execute

	//output genVars for ActPack monitoring
	rigid1.mn.userVar[5] = tau_meas;
	rigid1.mn.userVar[6] = tau_desired;
}

//UNUSED. See state_machine
/*
 * Simple Biom controller
 * input:	theta_set, desired theta
 * 			k1,k2,b, impedance parameters
 * return: 	tor_d, desired torque
 */
float biomControlImpedance(float theta_set, float k1, float k2, float b)
{
	static float theta = 0, theta_d = 0;
	static float tor_d = 0;

	theta = act1.jointAngle;
	theta_d = act1.jointVel;
	tor_d = k1 *(theta - theta_set) + k2 * (theta-theta_set)*(theta-theta_set)*(theta-theta_set) + b*theta_d;

	return tor_d;

}

void mit_init_current_controller(void) {

	setControlMode(CTRL_CURRENT);
	writeEx.setpoint = 0;			// wasn't included in setControlMode, could be safe for init
	setControlGains(currentKp, currentKi, currentKd, 0);

	// there is another example of this may have been an old initialization, copied from user-mn-ActPack
}

int8_t findPoles(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 100)
			{
				polesState = 1;
			}

			return 0;

		case 1:
			//Send Find Poles command:

			tx_cmd_calibration_mode_rw(TX_N_DEFAULT, CALIBRATION_FIND_POLES);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, mitDlegInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;

			return 0;

		case 2:

			if(timer >= 45*SECONDS)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				return 1;
			}
			return 0;


		default:

			return 0;

	}

	return 0;
}

void packRigidVars(struct act_s *actx) {

	// set float userVars to send back to Plan
	rigid1.mn.userVar[0] = actx->jointAngleDegrees;
	rigid1.mn.userVar[1] = actx->jointVelDegrees;
	rigid1.mn.userVar[2] = actx->linkageMomentArm;
	rigid1.mn.userVar[3] = actx->axialForce;
	rigid1.mn.userVar[4] = actx->jointTorque;
    //userVar[5] = tauMeas
    //userVar[6] = tauDes (impedance controller - spring contribution)
}

void openSpeedFSM(void)
{
	static uint32_t deltaT = 0;
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

void twoTorqueFSM(struct act_s *actx)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	rigid1.mn.genVar[9] = fsm1State;


	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = actx->jointAngle;
				fsm1State = 0;
			}
			break;
		case 0:
//			mit_init_current_controller();
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
			setMotorTorque( actx, 2);
			break;
		case 2:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorTorque( actx, -2);
			break;
	}
}

void oneTorqueFSM(struct act_s *actx)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	rigid1.mn.genVar[9] = fsm1State;


	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = actx->jointAngle;
				fsm1State = 0;
			}
			break;
		case 0:
//			mit_init_current_controller();
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
			setMotorTorque( actx, 2);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorTorque( actx, 0);
			break;
	}
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
