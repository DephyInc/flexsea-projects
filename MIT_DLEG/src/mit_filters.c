/*
 * mit_filters.c
 *
 *  Created on: Mar 14, 2018
 *      Author: matt
 */






/** ------------------------------------------------------------------- */
#include "arm_math.h"
#include "mit_filters.h"
#include "user-mn.h"
#include "user-mn-MIT-DLeg-2dof.h"
#include "state_machine.h"
#include "state_variables.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
float32_t lpf250[LPF_ORDER] = {
		0.000400000000000000,0.00111000000000000,0.00250000000000000,0.00479000000000000,0.00824000000000000,0.0130300000000000,0.0192400000000000,0.0267900000000000,0.0354200000000000,0.0446700000000000,0.0539600000000000,0.0626000000000000,0.0698600000000000,0.0751100000000000,0.0778600000000000,0.0778600000000000,0.0751100000000000,0.0698600000000000,0.0626000000000000,0.0539600000000000,0.0446700000000000,0.0354200000000000,0.0267900000000000,0.0192400000000000,0.0130300000000000,0.00824000000000000,0.00479000000000000,0.00250000000000000,0.00111000000000000,0.000400000000000000
};
float32_t lpf_buffer[LPF_ORDER];

arm_fir_instance_f32 S_lpf;
float32_t lpf_out;
float32_t lpf_input[LPF_ORDER*2];
uint16_t lpf_index=0;
float lpf_result;

void init_LPF(void)
{
	arm_fir_init_f32(&S_lpf, LPF_ORDER, (float32_t *) &lpf250[0], (float32_t *) &lpf_buffer[0], 1);
	lpf_index = 0;
	memset(lpf_input,0, LPF_ORDER*2*4); //4byte per float32_t * twice larger circular buffer
	lpf_out = 0;
	lpf_result = 0;

	return;
}

void update_LPF(float val)
{
	lpf_index++;
	if(lpf_index>=LPF_ORDER)
		lpf_index = 0;

	lpf_input[lpf_index] = (float32_t)val;
	lpf_input[lpf_index+LPF_ORDER] = (float32_t)val;

	return;
}

void filter_LPF(float val)
{
	update_LPF(val);
    arm_fir_f32(&S_lpf, (float32_t *)( &lpf_input[lpf_index] ) , (float32_t*) &lpf_out, 1);
    lpf_result = (float)lpf_out;

    return;
}
