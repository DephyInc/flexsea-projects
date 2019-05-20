#ifndef ACTPACK_DEFS_H_
#define ACTPACK_DEFS_H_

#if (HW_VER < 10)	// RIGID 0.1 and 0.2

	//Enable/Disable sub-modules:
	#define USE_USB
	#define USE_COMM			//Requires USE_RS485 and/or USE_USB
	#define USE_I2C_1			//3V3, IMU & Digital pot
	#define USE_I2C_2			//3V3, Expansion
	#define USE_I2C_3			//Onboard, Regulate & Execute
	#define USE_IMU				//Requires USE_I2C_1
	#define USE_UART3			//Bluetooth
	#define USE_EEPROM			//Emulated EEPROM, onboard FLASH
	#define USE_WATCHDOG		//Independent watchdog (IWDG)
	#define USE_6CH_AMP			//Requires USE_I2C_2. 6-ch Strain Amp.
	#define USE_SPI_PLAN		//Enables the external SPI port

	//Runtime finite state machine (FSM):
	//#define RUNTIME_FSM1		ENABLED	//Enable only if you DO NOT use Plan
	#define RUNTIME_FSM2		ENABLED	//Enable at all time, Mn <> Ex comm.

	#if(ACTIVE_SUBPROJECT == SUBPROJECT_A)

	#define MULTI_DOF_N			0

	#endif

	#if(ACTIVE_SUBPROJECT == SUBPROJECT_B)

	#define MULTI_DOF_N			1

	#endif

#elif (HW_VER < 20) 	//RIGID 1.0

	#include "rigid10.h"

#elif (HW_VER < 30) 	//RIGID 2.0

	#include "rigid20.h"

#else	// RIGID 3.0

	#include "rigid30.h"

#endif	//RIGID 2.0

#endif // ACTPACK_DEFS_H_