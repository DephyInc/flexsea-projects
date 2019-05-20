#ifndef RIGID30_H_
#define RIGID30_H_

//Enable/Disable sub-modules:
#define USE_USB
#define USE_COMM			//Requires USE_RS485 and/or USE_USB
#define USE_I2C_1			//3V3, IMU & Digital pot
#define USE_I2C_3			//Onboard, Regulate & Execute
#define USE_IMU				//Requires USE_I2C_1
#define USE_EEPROM			//Emulated EEPROM, onboard FLASH

//Runtime finite state machine (FSM):
#define RUNTIME_FSM1		ENABLED //STATE MACHINE
#define RUNTIME_FSM2		ENABLED //COMM
//#define	MANUAL_GUI_CONTROL

#define BILATERAL
#define USE_UART3
#define USE_UART4
#define USE_BT121

// TODO: it looks like when we're configuring the xbee or flashing
// new firmware onto the BT121 we need the multi packet stuff
// disabled so we just need a good way of setting all the symbols

#ifdef BT121_UPDATE_MODE
	#define TEST_BED_ENABLED
	#define USB_NO_MULTIPACKET
#endif

#ifdef XBEE_CONFIGURATION_MODE
	#define TEST_BED_ENABLED
	#define USB_NO_MULTIPACKET
#endif

//#define HABSOLUTE_UPSTREAM_TUNING

#if(ACTIVE_SUBPROJECT == RIGHT)

	#define EXO_SIDE	RIGHT
	#define BILATERAL_MASTER

#elif(ACTIVE_SUBPROJECT == LEFT)

	#define EXO_SIDE	LEFT
	#define BILATERAL_SLAVE

#else

	#error "a subproject is required (use A by default)!"

#endif

#endif // RIGID30_H_