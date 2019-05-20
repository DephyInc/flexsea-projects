#ifndef RIGID10_H_
#define RIGID10_H_

//Enable/Disable sub-modules:
#define USE_USB
#define USE_COMM			//Requires USE_RS485 and/or USE_USB
//#define USE_RS485
#define USE_I2C_1			//3V3, IMU & Digital pot
//#define USE_I2C_2			//3V3, Expansion
//#define USE_I2C_3			//Onboard, Regulate & Execute
#define USE_IMU				//Requires USE_I2C_1
//#define USE_UART3			//Bluetooth
#define USE_EEPROM			//Emulated EEPROM, onboard FLASH
#define USE_WATCHDOG		//Independent watchdog (IWDG)
#define USE_XB24C			//Radio module on UART2 (Expansion port)

//Runtime finite state machine (FSM):
//#define RUNTIME_FSM1		ENABLED	//Enable only if you DO NOT use Plan
//#define RUNTIME_FSM2		ENABLED	//Enable at all time, Mn <> Ex comm.

#define MULTI_DOF_N 		0

#if(ACTIVE_SUBPROJECT == RIGHT)

	#define EXO_SIDE	RIGHT
	#define BILATERAL_MASTER

#elif(ACTIVE_SUBPROJECT == LEFT)

	#define EXO_SIDE	LEFT
	#define BILATERAL_SLAVE

#else

	#error "a subproject is required (use A by default)!"

#endif

#endif // RIGID10_H_