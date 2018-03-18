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
		-0.000372753877604634,-0.000706153754828076,-0.00119283620617159,-0.00166341967872324,-0.00189343952540585,-0.00155382796980903,-0.000233137397042932,0.00251415259726580,0.00709609473823305,0.0137989205416623,0.0227040521745074,0.0336229733823327,0.0460675410381188,0.0592665472182299,0.0722309114570265,0.0838632155694735,0.0930956483263148,0.0990332744185328,0.101082099477917,0.0990332744185328,0.0930956483263148,0.0838632155694735,0.0722309114570265,0.0592665472182299,0.0460675410381188,0.0336229733823327,0.0227040521745074,0.0137989205416623,0.00709609473823305,0.00251415259726580,-0.000233137397042932,-0.00155382796980903,-0.00189343952540585,-0.00166341967872324,-0.00119283620617159,-0.000706153754828076,-0.000372753877604634
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
