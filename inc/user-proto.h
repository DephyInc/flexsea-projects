#ifdef BOARD_TYPE_FLEXSEA_PROTOTYPE

#ifndef INC_USER_PROTO_H
#define INC_USER_PROTO_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "flexsea_board.h"
#include "flexsea_sys_def.h"
//Add your project specific user_x.h file here

//****************************************************************************
// Shared variable(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_user(void);
void user_fsm(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

/*
//List of encoders that can be used by controllers (position, impedance),
//and for commutation:
#define ENC_NONE				0	//No encoder
#define ENC_HALL				1	//Hall effect (motor commutation)
#define ENC_QUADRATURE			2	//Optical or magnetic, AB/I inputs on QEI1
#define ENC_ANALOG				3	//Potentiometer (or other), on ext. analog in.
#define ENC_AS5047				4	//16-bit Magnetic Position Sensor, SPI
#define ENC_AS5048B				5	//14-bit Magnetic Position Sensor, I2C
#define ENC_CUSTOM			  	6	//Heavily modified user variable that cannot
									//be represented CTRL_ENC_FCT
//(later you'll assign what encoder is used by the controllers, for motor
// commutation, and which one is displayed in the GUI)

//Type of motor commutation:
#define COMMUT_BLOCK			0
#define COMMUT_SINE				1
#define COMMUT_NONE				2	//Software test, no motor

//Current sensing strategy:
#define CS_LEGACY				0
#define CS_DEFAULT				1

//Types of motor orientation. Rotation of motor when you are looking at the rotor
#define CLOCKWISE_ORIENTATION 			1
#define COUNTER_CLOCKWISE_ORIENTATION 	-1
*/

//List of projects:
#define PROJECT_SCCD			0		//Single Cell Charger Discharger

//*No external sensor, no sinusoidal commutation

//List of sub-projects:
#define SUBPROJECT_NONE			0
#define SUBPROJECT_A			1
#define SUBPROJECT_B			2
//(ex.: the 2-DoF ankle has 2 Execute. They both use PROJECT_2DOF_ANKLE, and each
// 		of them has a sub-project for specific configs)

//Step 1) Select active project (from list):
//==========================================

#define ACTIVE_PROJECT			PROJECT_SCCD
#define ACTIVE_SUBPROJECT		SUBPROJECT_A

//Step 2) Customize the enabled/disabled sub-modules:
//===================================================

//Barebone FlexSEA-Execute project - no external peripherals (and no motor)
#if(ACTIVE_PROJECT == PROJECT_SCCD)

	//Enable/Disable sub-modules:
	//#define USE_RS485
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	//#define USE_I2C_0			//3V3, IMU & Expansion.
	//#define USE_I2C_1			//5V, Safety-CoP & strain gauge pot.
	//#define USE_EEPROM		//
	//#define USE_FLASH			//
	//#define USE_BLUETOOTH		//
	
	#define RUNTIME_FSM	 		DISABLED
	
	//Slave ID:
	#define SLAVE_ID			FLEXSEA_EXECUTE_1

	//Project specific definitions:
	//...

#endif	//PROJECT_BAREBONE

//****************************************************************************
// Structure(s)
//****************************************************************************

//****************************************************************************
// Default(s)
//****************************************************************************


#endif	//INC_USER_PROTO_H

#endif //BOARD_TYPE_FLEXSEA_PROTOTYPE
