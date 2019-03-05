//Use this file to control the amount of RAM used by flexsea-projects

#ifndef INC_FLEXSEA_PROJECTS_STACK_CONFIG_H
#define INC_FLEXSEA_PROJECTS_STACK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#if (defined BOARD_TYPE_FLEXSEA_EXECUTE && !defined BOARD_SUBTYPE_RIGID)

	#define SC_PRJ_EN_RI1
	
#else

	//Default: everything enabled

	#define SC_PRJ_EN_RI1
	#define SC_PRJ_EN_RI2
	#define SC_PRJ_EN_PCK1

#endif

#ifdef __cplusplus
}
#endif

#endif	//INC_FLEXSEA_PROJECTS_STACK_CONFIG_H
