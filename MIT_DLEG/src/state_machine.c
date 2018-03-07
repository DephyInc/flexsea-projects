#include "state_variables.h"
#include "state_machine.h"
#include <math.h>

static float calcTorque(GainParams gainParams);

WalkingStateMachine stateMachine;
ANKLE_SENSORS_T sensors;
//toDo check that angle setPoints are correct
//toDo incorporate these into Plan GUI. Need to make these global
GainParams eswGains = {0.04, 0, 0.004, 23};
GainParams lswGains = {0.134, 0, 0.002, 2};
GainParams estGains = {1.35, 0.025, 0.118, -5};
GainParams lstGains; //currently unused in simple implementation
GainParams lstPowerGains = {4.5, 0, 0.005, -18};

/** Impedance Control Level-ground Walking FSM
	Based on BiOM ankle and simplified.
	Finds desired torque.

	@param ptorqueDes pointer to float meant to be updated with desired torque
*/
void runFlatGroundFSM(float* ptorqueDes) {

	static int8_t isTransitioning = 0;
	static uint32_t time_in_state = 0;
	
	stateMachine.on_entry_sm_state = stateMachine.current_state; // save the state on entry, assigned to last_current_state on exit

	*ptorqueDes = 0;

	// Check for state change, then set isTransitioning flag
	if (stateMachine.current_state == stateMachine.last_sm_state) {
		isTransitioning = 0;
		time_in_state++;
	} else {
		// State transition, reset timers and set entry flag
		time_in_state = 0;
		isTransitioning = 1;
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

			*ptorqueDes = calcTorque(eswGains);

			//Early Swing transition vectors
			// VECTOR(1): Early Swing -> Late Swing
			if (time_in_state >= ESW_TO_LSW_DELAY) {
				stateMachine.current_state = STATE_LATE_SWING;      //Transition occurs even the early swing motion is not finished
			}

			//run any exit code here

			break; // case STATE_EARLY_SWING

		case STATE_LATE_SWING:

			*ptorqueDes = calcTorque(lswGains);

			//Late Swing transition vectors
			// VECTOR (1): Late Swing -> Early Stance (hard heal strike)
			//toDo real sensor values
			if (sensors.SS_torque_dot < HARD_HEALSTRIKE_SS_TORQ_RATE_THRESH) {
				stateMachine.current_state = STATE_EARLY_STANCE;
			}

			break;

		case STATE_EARLY_STANCE:

			*ptorqueDes = calcTorque(estGains);

			//Early Stance transition vectors
			// VECTOR (1): Early Stance -> Late Stance POWER!
			//toDo real sensor values
			if (sensors.hardstop_torque > LSTPWR_HS_TORQ_TRIGGER_THRESH) {

				stateMachine.current_state = STATE_LATE_STANCE_POWER;
			}
		
			break;
		
		case STATE_LATE_STANCE_POWER:

			*ptorqueDes = calcTorque(lstPowerGains);

			//Late Stance Power transition vectors
			// VECTOR (1): Late Stance Power -> Early Swing - Condition 1
			if (sensors.ankle_total_torque < ANKLE_UNLOADED_TORQUE_THRESH) {
				stateMachine.current_state = STATE_EARLY_SWING;

			}

			break;

		case STATE_EMG_STAND_ON_TOE:
			//toDo with EMG

			break;
		
		default:

			//turn off control.
			*ptorqueDes = 0;

			break;
	
		}

	//update last state in preparation for next loop
	stateMachine.last_sm_state = stateMachine.on_entry_sm_state;
}

/** Impedance Control Torque
	Calculates desired torque based on impedance gain parameters

	@param  gainParams struct with all the state's impedance parameters
    @return float desired torque at joint (Nm)
*/
static float calcTorque(GainParams gainParams) {
	//toDo replace with real values
	float angleCurrent_ = 0;
	float dAngle_ = 0;

	return gainParams.k1 * (angleCurrent_ - gainParams.thetaDes) \
		 + gainParams.k2 * powf((angleCurrent_ - gainParams.thetaDes), 3) - gainParams.b * dAngle_;
}
