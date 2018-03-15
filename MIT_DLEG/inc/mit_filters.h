/*
 * mit_filters.h
 *
 *  Created on: Mar 14, 2018
 *      Author: Seong Ho
 */

//#ifndef BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_
//#define BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_
#include "arm_math.h"
#define LPF_ORDER             37	// Order + 1 from matlb.

extern float lpf_result;
extern uint16_t lpf_index;

//float32_t lpf_out;
extern float32_t lpf_input[LPF_ORDER*2];
void init_LPF(void);
void update_LPF(float val);
void filter_LPF(float val);



//#endif /* BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_ */
