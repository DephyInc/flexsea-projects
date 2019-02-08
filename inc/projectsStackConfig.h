//Use this file to control the amount of RAM used by flexsea-projects

#ifndef INC_FLEXSEA_PROJECTS_STACK_CONFIG_H
#define INC_FLEXSEA_PROJECTS_STACK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

	#define SC_EN_RI1

#else

	//Default: everything enabled

	#define SC_EN_RI1
	#define SC_EN_RI2
	#define SC_EN_PCK2

#endif

#ifdef __cplusplus
}
#endif

#endif	//INC_FLEXSEA_PROJECTS_STACK_CONFIG_H