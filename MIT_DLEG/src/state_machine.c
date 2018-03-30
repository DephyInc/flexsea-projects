
#ifdef __cplusplus
extern "C" {
#endif

#include "state_variables.h"
#include "state_machine.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "user-mn-MIT-EMG.h"
#include "flexsea_user_structs.h"
#include "free_ankle_EMG.h"
#include "stand_ankle_EMG.h"
#include "cmd-DLeg.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include <math.h>

WalkingStateMachine stateMachine;
Act_s act1;
GainParams eswGains = {0.04, 0, 0.004, 23};
GainParams lswGains = {0.134, 0, 0.002, 2};
GainParams estGains = {1.35, 0.025, 0.118, USER_OFFSET_ANGLE};
GainParams lstGains = {0, 0, 0, 0}; //currently unused in simple implementation
GainParams lstPowerGains = {4.5, 0, 0.005, 18};
GainParams emgStandGains = {2, 0.025, 0.04, 0};
GainParams emgFreeGains = {0.5, 0, 0.02, 0};

#ifndef BOARD_TYPE_FLEXSEA_PLAN

//Static functions
static float calcJointTorque(GainParams gainParams);

/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
float runFlatGroundFSM(void) {

    static int8_t isTransitioning = 0;
    static uint32_t time_in_state = 0;
	
    stateMachine.on_entry_sm_state = stateMachine.current_state; // save the state on entry, assigned to last_current_state on exit

    float torqueDes = 0;

    // Check for state change, then set isTransitioning flag
    if (stateMachine.current_state == stateMachine.last_sm_state) {
        isTransitioning = 0;
        time_in_state++;
    } else {
        // State transition, reset timers and set entry flag
        time_in_state = 0;
        isTransitioning = 1;

        //update Plan with state changes
//        static uint8_t info[2] = {PORT_USB, PORT_USB};
//        tx_cmd_dleg_w(TX_N_DEFAULT, stateMachine.current_state);
//        packAndSend(P_AND_S_DEFAULT, FLEXSEA_PLAN_1, info, SEND_TO_MASTER); //3rd arg unused
    }

    switch (stateMachine.current_state) {

        case STATE_IDLE:
            //error handling here (should never be in STATE_IDLE by the time you get here)
            break;

        case STATE_INIT:

            stateMachine.current_state = STATE_LATE_SWING;

            break;
					
        case STATE_EARLY_SWING:
            //Put anything you want to run ONCE during state entry.
//			if (isTransitioning) {
//
//			}

        	torqueDes = calcJointTorque(eswGains);

            //Early Swing transition vectors
            // VECTOR(1): Early Swing -> Late Swing
            if (time_in_state >= ESW_TO_LSW_DELAY) {
                stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even if the early swing motion is not finished
            }

            //run any exit code here

            break; // case STATE_EARLY_SWING

        case STATE_LATE_SWING:

        	torqueDes = calcJointTorque(lswGains);

            //Late Swing transition vectors
            // VECTOR (1): Late Swing -> Early Stance (hard heel strike)
            //toDo: better transition criterion than this
            if (act1.jointTorque > HARD_HEELSTRIKE_TORQUE_THRESH) {
                stateMachine.current_state = STATE_EARLY_STANCE;
            }

            break;

        case STATE_EARLY_STANCE:

        	if (isTransitioning) {
				reset_EMG_stand(act1.jointAngleDegrees);
			}

        	if (MIT_EMG_getState() == 1) {

				updateStandJoint(&emgStandGains);

				//if we're still in EMG control even after updating EMG control vars
				if (stateMachine.current_state == STATE_EMG_STAND_ON_TOE) {
					torqueDes = calcJointTorque(emgStandGains);

				//actually in early stance (explicitly declared)
				} else if (stateMachine.current_state == STATE_EARLY_STANCE) {
					torqueDes = calcJointTorque(estGains);
				}

			} else {
				torqueDes = calcJointTorque(estGains); //do early stance control if EMG disconnected
			}

        	//Early Stance transition vectors (only into lstPower for now)

            if (act1.jointTorque > LSTPWR_HS_TORQ_TRIGGER_THRESH) {
                stateMachine.current_state = STATE_LATE_STANCE_POWER;
            }

        	break;


        case STATE_LATE_STANCE_POWER:

        	torqueDes = calcJointTorque(lstPowerGains);

            //Late Stance Power transition vectors
            // VECTOR (1): Late Stance Power -> Early Swing - Condition 1
            if (abs(act1.jointTorque) < ANKLE_UNLOADED_TORQUE_THRESH) {
                stateMachine.current_state = STATE_EARLY_SWING;

            }

            break;

        case STATE_EMG_STAND_ON_TOE:

        	if (MIT_EMG_getState() == 1) {

        		updateStandJoint(&emgStandGains);

        		//if we're still in EMG control even after updating EMG control vars
        		if (stateMachine.current_state == STATE_EMG_STAND_ON_TOE) {
        			torqueDes = calcJointTorque(emgStandGains);

        		//actually in early stance (explicitly declared)
        		} else if (stateMachine.current_state == STATE_EARLY_STANCE) {
        			torqueDes = calcJointTorque(estGains);
        		}

			} else {
				stateMachine.current_state = STATE_EARLY_STANCE;
				reset_EMG_stand(act1.jointAngleDegrees);
        		torqueDes = calcJointTorque(estGains); //do early stance control if EMG disconnected
			}

        	//transition vectors to lstPower here

            break;
		
        case STATE_LSW_EMG:
        	//upon entering, make sure virtual joint is initialized, otherwise program will fail
        	if (isTransitioning) {
        		resetPFDFState();
        	}

        	emgFreeGains.b  = user_data_1.w[6]/100.;

        	//check to make sure EMG is active
        	if (MIT_EMG_getState() == 1) {
        		updateVirtualJoint(&emgFreeGains);
        		torqueDes = calcJointTorque(emgFreeGains);
        	} else {
        		//this should probably by default kick into early stance
        		resetPFDFState();
        		torqueDes = calcJointTorque(estGains);
        	}
        	rigid1.mn.genVar[8] = torqueDes;

        	//toDo: Late Swing EMG transition vectors to Early Stance HOW?! Perhaps load cell

        	break;
		
        default:

            //turn off control.
        	torqueDes = 0;

            break;
	
    }

    //update last state in preparation for next loop
    stateMachine.last_sm_state = stateMachine.on_entry_sm_state;

    return torqueDes;
}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	@param  gainParams struct with all the state's impedance parameters
    @return float desired torque at joint (Nm)
*/
static float calcJointTorque(GainParams gainParams) {

    return gainParams.k1 * (gainParams.thetaDes - act1.jointAngleDegrees) \
         + gainParams.k2 * powf((gainParams.thetaDes - act1.jointAngleDegrees), 3) - gainParams.b * act1.jointVelDegrees;
}

//reset virtual joint to robot joint state
void resetPFDFState(void) {
	PFDF_state[0] = equilibriumAngle;
	PFDF_state[1] = 0;
	PFDF_state[2] = 0;
}

#endif //BOARD_TYPE_FLEXSEA_PLAN

#ifdef __cplusplus
}
#endif


