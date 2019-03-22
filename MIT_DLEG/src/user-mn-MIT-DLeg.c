/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-projects' User projects
	Copyright (C) 2018 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developer] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-mn-MIT-DLeg: Demo state machine for DLeg
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-02-24 | jfduval | New release
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_DLEG
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

/*01/23/2019 Demo code: run this on Mn1 (ActPack 0.2B, not MIT's hardware) to
 * control a second ActPack that's running default PROJECT_ACTPACK code. It
 * showcases the MULTI_DOF_N capability.
 */


//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include "user-mn-MIT-DLeg.h"
#include "user-mn-ActPack.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"
#include "flexsea_user_structs.h"
#include "mn-MotorControl.h"
#include "cmd-ActPack.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

#define POWER_ON_DELAY	7000

int32_t angmap[1000] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 4,
     5, 6, 7, 9, 10, 11, 13, 14, 16, 17, 19, 21, 23, 25, 27, 29, 31, 33, 36, 38,
     41, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 74, 77, 80, 84, 88, 91, 95, 99, 103,
     107, 111, 115, 119, 123, 128, 132, 137, 141, 146, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195,
     200, 206, 211, 217, 222, 228, 233, 239, 245, 251, 257, 263, 269, 275, 281, 287, 293, 299, 306, 312,
     318, 325, 331, 338, 345, 351, 358, 365, 372, 379, 386, 393, 400, 407, 414, 421, 428, 435, 443, 450,
     458, 465, 473, 480, 488, 495, 503, 511, 518, 526, 534, 542, 550, 558, 566, 574, 582, 590, 598, 606,
     615, 623, 631, 640, 648, 656, 665, 673, 682, 690, 699, 708, 716, 725, 734, 742, 751, 760, 769, 778,
     787, 796, 805, 814, 823, 832, 841, 850, 859, 868, 877, 886, 896, 905, 914, 924, 933, 942, 952, 961,
     970, 980, 989, 999, 1008, 1018, 1027, 1037, 1046, 1056, 1066, 1075, 1085, 1095, 1104, 1114, 1124, 1133, 1143, 1153,
     1163, 1172, 1182, 1192, 1202, 1212, 1221, 1231, 1241, 1251, 1261, 1271, 1281, 1291, 1301, 1310, 1320, 1330, 1340, 1350,
     1360, 1370, 1380, 1390, 1400, 1410, 1420, 1430, 1440, 1450, 1460, 1470, 1480, 1490, 1500, 1510, 1520, 1530, 1540, 1550,
     1560, 1570, 1580, 1590, 1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670, 1680, 1690, 1699, 1709, 1719, 1729, 1739, 1749,
     1759, 1769, 1779, 1788, 1798, 1808, 1818, 1828, 1837, 1847, 1857, 1867, 1876, 1886, 1896, 1905, 1915, 1925, 1934, 1944,
     1954, 1963, 1973, 1982, 1992, 2001, 2011, 2020, 2030, 2039, 2048, 2058, 2067, 2076, 2086, 2095, 2104, 2114, 2123, 2132,
     2141, 2150, 2159, 2168, 2177, 2186, 2195, 2204, 2213, 2222, 2231, 2240, 2249, 2258, 2266, 2275, 2284, 2292, 2301, 2310,
     2318, 2327, 2335, 2344, 2352, 2360, 2369, 2377, 2385, 2394, 2402, 2410, 2418, 2426, 2434, 2442, 2450, 2458, 2466, 2474,
     2482, 2489, 2497, 2505, 2512, 2520, 2527, 2535, 2542, 2550, 2557, 2565, 2572, 2579, 2586, 2593, 2600, 2607, 2614, 2621,
     2628, 2635, 2642, 2649, 2655, 2662, 2669, 2675, 2682, 2688, 2694, 2701, 2707, 2713, 2719, 2725, 2731, 2737, 2743, 2749,
     2755, 2761, 2767, 2772, 2778, 2783, 2789, 2794, 2800, 2805, 2810, 2815, 2820, 2825, 2830, 2835, 2840, 2845, 2850, 2854,
     2859, 2863, 2868, 2872, 2877, 2881, 2885, 2889, 2893, 2897, 2901, 2905, 2909, 2912, 2916, 2920, 2923, 2926, 2930, 2933,
     2936, 2939, 2942, 2945, 2948, 2951, 2954, 2957, 2959, 2962, 2964, 2967, 2969, 2971, 2973, 2975, 2977, 2979, 2981, 2983,
     2984, 2986, 2987, 2989, 2990, 2991, 2993, 2994, 2995, 2996, 2996, 2997, 2998, 2998, 2999, 2999, 3000, 3000, 3000, 3000,
     2998, 2993, 2984, 2972, 2957, 2938, 2916, 2891, 2863, 2832, 2798, 2762, 2722, 2680, 2636, 2588, 2539, 2487, 2433, 2376,
     2317, 2257, 2194, 2129, 2063, 1994, 1924, 1852, 1779, 1704, 1628, 1550, 1471, 1391, 1310, 1227, 1144, 1059, 974, 888,
     801, 714, 626, 537, 449, 359, 270, 180, 90, 0, -90, -180, -270, -359, -448, -537, -626, -714, -801, -888,
     -974, -1059, -1144, -1227, -1309, -1391, -1471, -1550, -1628, -1704, -1779, -1852, -1924, -1994, -2062, -2129, -2194, -2257, -2317, -2376,
     -2433, -2487, -2539, -2588, -2635, -2680, -2722, -2762, -2798, -2832, -2863, -2891, -2916, -2938, -2956, -2972, -2984, -2993, -2998, -3000,
     -2999, -2998, -2995, -2992, -2987, -2981, -2975, -2967, -2959, -2949, -2939, -2927, -2915, -2902, -2888, -2873, -2858, -2841, -2824, -2806,
     -2787, -2767, -2747, -2726, -2704, -2681, -2658, -2634, -2609, -2584, -2558, -2532, -2504, -2477, -2448, -2419, -2390, -2360, -2329, -2298,
     -2267, -2235, -2202, -2169, -2136, -2102, -2068, -2033, -1998, -1963, -1927, -1891, -1855, -1818, -1781, -1744, -1706, -1668, -1630, -1592,
     -1553, -1515, -1476, -1437, -1398, -1358, -1319, -1279, -1239, -1200, -1160, -1120, -1080, -1040, -1000, -960, -920, -880, -840, -800,
     -761, -721, -681, -642, -602, -563, -524, -485, -447, -408, -370, -332, -294, -256, -219, -182, -145, -109, -73, -37,
     -2, 33, 68, 102, 136, 169, 202, 235, 267, 298, 329, 360, 390, 419, 448, 477, 504, 532, 558, 584,
     609, 634, 658, 681, 704, 726, 747, 767, 787, 806, 824, 841, 858, 873, 888, 902, 915, 927, 939, 949,
     959, 967, 975, 981, 987, 992, 995, 998, 999, 1000, 1000, 1000, 999, 999, 998, 997, 996, 995, 994, 993,
     991, 990, 988, 986, 984, 982, 980, 977, 975, 972, 969, 966, 963, 960, 957, 954, 950, 947, 943, 939,
     935, 931, 927, 923, 919, 914, 910, 905, 901, 896, 891, 886, 881, 876, 871, 866, 860, 855, 849, 844,
     838, 832, 827, 821, 815, 809, 803, 796, 790, 784, 778, 771, 765, 758, 752, 745, 739, 732, 725, 718,
     711, 705, 698, 691, 684, 677, 669, 662, 655, 648, 641, 634, 626, 619, 612, 604, 597, 590, 582, 575,
     567, 560, 552, 545, 537, 530, 522, 515, 507, 500, 493, 485, 478, 470, 463, 455, 448, 440, 433, 425,
     418, 410, 403, 396, 388, 381, 374, 366, 359, 352, 345, 338, 331, 323, 316, 309, 302, 295, 289, 282,
     275, 268, 261, 255, 248, 242, 235, 229, 222, 216, 210, 204, 197, 191, 185, 179, 173, 168, 162, 156,
     151, 145, 140, 134, 129, 124, 119, 114, 109, 104, 99, 95, 90, 86, 81, 77, 73, 69, 65, 61,
     57, 53, 50, 46, 43, 40, 37, 34, 31, 28, 25, 23, 20, 18, 16, 14, 12, 10, 9, 7,
     6, 5, 4, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int32_t ang_init_glob = 0;
FSMSTATE CT_state = CT_BOOT;
uint32_t t_glob = 0;
uint32_t t_ste = 0;
int32_t mot_ang;
int32_t kp = 100, ki = 0, kd = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
static void changeState(int16_t);


//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MIT_DLeg(void)
{


}

//MIT DLeg Finite State Machine.
//Call this function in one of the main while time slots.
void MIT_DLeg_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

	mot_ang = (*rigid1.ex.enc_ang);


    //Increment time (1 tick = 1ms)
    t_glob++;
    t_ste++;

    switch(CT_state)
	{
		case CT_BOOT:	//Initial state - waiting
			if (t_glob > POWER_ON_DELAY)
			{
				setControlMode(CTRL_POSITION, 0);
				setControlGains(kp, ki, kd, 0, 0);
				ang_init_glob = mot_ang;
				changeState(CT_WALK);
			}
			break;
		case CT_WALK:
			if (ctrl[0].active_ctrl != CTRL_POSITION)
			{
				setControlMode(CTRL_POSITION, 0);
				setControlGains(kp, ki, kd, 0, 0);
			}

			setMotorPosition(ang_init_glob+angmap[t_ste%1000], 0);

			if (t_ste > 4000)
			{
				changeState(CT_WALK2);
			}
			break;
		case CT_WALK2:
			setControlMode(CTRL_OPEN, 0);
			setMotorVoltage(0, 0);
			if (t_ste > 4000)
			{
				changeState(CT_WALK);
			}
			break;
		default:
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}



//****************************************************************************
// Private Function(s)
//****************************************************************************

static void openSpeedFSM(void)
{
	static uint32_t deltaT = 0;
	static uint8_t fsm1State = 0;

	/*
	switch(fsm1State)
	{
		case 0:
			setControlMode(CTRL_OPEN, THIS_ACTPACK);
			setControlMode(CTRL_OPEN, SLAVE_ACTPACK);
			setMotorVoltage(0, THIS_ACTPACK);
			setMotorVoltage(0, SLAVE_ACTPACK);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorVoltage(0, THIS_ACTPACK);
			setMotorVoltage(-1000, SLAVE_ACTPACK);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorVoltage(2500, THIS_ACTPACK);
			setMotorVoltage(0, SLAVE_ACTPACK);
			break;
	}
	*/
}

/*
static void twoPositionFSM(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static int8_t fsm1State = -1;
	static int32_t initPos = 0;

	switch(fsm1State)
	{
		case -1:
			//We give FSM2 some time to refresh values
			timer++;
			if(timer > 25)
			{
				initPos = *(rigid1.ex.enc_ang);
				fsm1State = 0;
			}
			break;
		case 0:
			setControlMode(CTRL_POSITION, 0);
			setControlGains(20, 6, 0, 0, 0);	//kp = 20, ki = 6
			setMotorPosition(initPos, 0);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorPosition(initPos + 10000, 0);
			break;
		case 2:
			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorPosition(initPos, 0);
			break;
	}
}
*/

static void changeState(int16_t ns)
{
	t_ste = 0;
	CT_state = ns;

}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG
