#ifndef BMS01_H_
#define BMS01_H_

//Enable/Disable sub-modules:
#define USE_USB
#define USE_COMM			//Requires USE_RS485 and/or USE_USB
//#define USE_I2C_1			//
//#define USE_I2C_2			//
//#define USE_I2C_3			//
//#define USE_EEPROM		//Emulated EEPROM, onboard FLASH
//#define USE_WATCHDOG		//Independent watchdog (IWDG)
#define USE_HABSOLUTE

//Runtime finite state machine (FSM):
#define RUNTIME_FSM1		DISABLED	//Enable only if you DO NOT use Plan
#define RUNTIME_FSM2		ENABLED		//Enable at all time, Mn <> Ex comm.

#define MULTI_DOF_N			0

#endif // BMS01_H_
