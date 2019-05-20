#ifndef RIGID10_H_
#define RIGID10_H_
//Enable/Disable sub-modules:
#define USE_USB
#define USE_COMM			//Requires USE_RS485 and/or USE_USB
#define USE_I2C_1			//3V3, IMU & Digital pot
//#define USE_I2C_2			//3V3, Expansion
#define USE_I2C_3			//Onboard, Regulate & Execute
#define USE_IMU				//Requires USE_I2C_1
#define USE_UART3			//Bluetooth
#define USE_EEPROM			//Emulated EEPROM, onboard FLASH
#define USE_WATCHDOG		//Independent watchdog (IWDG)

//Runtime finite state machine (FSM):
#define RUNTIME_FSM1		ENABLED
#define RUNTIME_FSM2		ENABLED

#endif // RIGID10_H_