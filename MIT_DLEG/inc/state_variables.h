#ifndef INC_STATE_VARIABLES
#define INC_STATE_VARIABLES

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//****************************************************************************
// Structure(s):
//****************************************************************************

enum {
	STATE_IDLE = 0,
	STATE_INIT = 1,
	STATE_EARLY_SWING = 2,
	STATE_LATE_SWING = 3,
	STATE_EARLY_STANCE = 4,
    STATE_LATE_STANCE = 5,
    STATE_LATE_STANCE_POWER = 6,
    STATE_EMG_STAND_ON_TOE = 7,
    STATE_LSW_EMG = 8
};

typedef struct{
	uint16_t current_state;
	uint16_t on_entry_sm_state;
	uint16_t last_sm_state;

} WalkingStateMachine;

//Gain params
#define NUM_STATES				8
#define NUM_IMPEDANCE_TERMS		4

typedef struct{

	float k1;
	float k2;
	float b;
	float thetaDes;

} GainParams;

//****************************************************************************
// Shared variable(s)
//****************************************************************************

#ifdef __cplusplus
}
#endif

#endif //INC_STATE_VARIABLES
