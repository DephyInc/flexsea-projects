#ifndef RIGID30_H_
#define RIGID30_H_

//Enable/Disable sub-modules:
#define USE_USB
#define USE_COMM			//Requires USE_RS485 and/or USE_USB
#define USE_I2C_1			//3V3, IMU & Digital pot
#define USE_I2C_3			//Onboard, Regulate & Execute
#define USE_IMU				//Requires USE_I2C_1
#define USE_EEPROM			//Emulated EEPROM, onboard FLASH

//Sub-projects:
//==============
#define PRJ_DEPHY_DPEB45	//DpEb4.5

//Runtime finite state machine (FSM):
#define RUNTIME_FSM1		ENABLED //STATE MACHINE
#define RUNTIME_FSM2		ENABLED //COMM
//#define	MANUAL_GUI_CONTROL

#define BILATERAL
#define USE_UART3
#define USE_UART4

#endif // RIGID30_H_