#ifdef BOARD_TYPE_FLEXSEA_PROTOTYPE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-proto.h"
#include "flexsea_user_structs.h"
#include "flexsea_sys_def.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

	
//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void refresh_values(void);
static void init_project(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialization function - call once in main.c, before while()
void init_user(void)
{
	//Common setup:
	init_project();

	//DpEb21
	#if(ACTIVE_PROJECT == PROJECT_SCCD)
	//initSCCD();
	#endif	//PROJECT_SCCD
}

//Call this function in one of the main while time slots.
void user_fsm(void)
{
	//Refresh sensor values - common across projects:
	refresh_values();
	
	//SCCD
	#if(ACTIVE_PROJECT == PROJECT_SCCD)

	#endif	//PROJECT_SCCD
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Common setup
static void init_project(void)
{
	setBoardID(SLAVE_ID);
}

//Refresh sensor values - common across projects
static void refresh_values(void)
{

}

#endif //BOARD_TYPE_FLEXSEA_PROTOTYPE
