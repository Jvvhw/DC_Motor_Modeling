#include "DllHeader.h"
#include "variable.h"
#include "math.h"

void InitDCmachine() {			   
	Ra = 0.25 * 2.;
	Rf = 3.0;
	La = 0.01;
	Lf = 3.0;
	Lamf = 2.0;
	J = 0.02;
	B = 0.1;
}

void InitController() {			   
	Wcc = PI2 * 200.;
	Wsc = PI2 * 2.;

	Kpa_cc = La * Wcc; 
	Kia_cc = Ra * Wcc;
	Kaa_cc = 1. / Kpa_cc;

	Kpf_cc = Lf * Wcc; 
	Kif_cc = Rf * Wcc;
	Kaf_cc = 1. / Kpf_cc;

	Kps_sc = J * Wsc; 
	Kis_sc = B * Wsc;
	Kas_sc = 1. / Kps_sc;
}

void InitINV() {			   
	Is_rated = 10.;
	PWM_mode_set = 0;
	INV_Vdc = 1 / Vdc;
}

DLLEXPORT void plecsSetSizes(struct SimulationSizes* aSizes)
{
	aSizes->numInputs = 22;
	aSizes->numOutputs = 100;
}


//This function is automatically called at the beginning of the simulation
DLLEXPORT void plecsStart(struct SimulationState* aState)
{

}


//This function is automatically called every sample time
//output is written to DLL output port after the output delay
DLLEXPORT void plecsOutput(struct SimulationState* aState)
{

	//INPUT
	Vdc = aState->inputs[0];
	Ia = aState->inputs[1];
	If = aState->inputs[2];
	Te = aState->inputs[3];
	Wm = aState->inputs[4];
	INVf_on = aState->inputs[5];
	INVa_on = aState->inputs[6];
	If_ref = aState->inputs[7];
	Wm_ref = aState->inputs[8];
	// x = aState->inputs[n];

	Tsamp = aState->inputs[21];  //Fixed Tsamp 
	
	if (init == 0) {
		InitDCmachine();
		InitINV();
		InitController();
		init = 1;
	}

	// If control
	if (INVf_on) {
		If_err = If_ref - If;

		Vsf_ref_integ += Tsamp * Kif_cc * (If_err - Kaf_cc * (Vsf_ref - Vsf_ref_set));// 

		Vsf_ref = Vsf_ref_integ + Kpf_cc * If_err;

		if (Vsf_ref > Vdc) Vsf_ref_set = Vdc;
		else if (Vsf_ref < -Vdc) Vsf_ref_set = -Vdc;
		else                    Vsf_ref_set = Vsf_ref;

		nVsf_ref_set = -Vsf_ref_set;

		Duty_Af = 0.5 * Vsf_ref_set * INV_Vdc + 0.5;//0~1

		Duty_Bf = 0.5 * nVsf_ref_set * INV_Vdc + 0.5;//0~1


	}
	else {
		Duty_Af = 0.5;//0~1

		Duty_Bf = 0.5;//0~1

	}
	
	// Speed Control
	if (INVa_on) {
		Wm_err = Wm_ref - Wm;

		Wm_ref_integ += Tsamp * Kis_sc * Wm_err;

		Te_ref = Wm_ref_integ + Kps_sc * Wm_err;

		Ia_ref = Te_ref * 5; // Kt = 0.2 , Can be modified

	}
	else {
		Ia_ref = 15;
	}

	// Ia control
	if (INVa_on) {
		Ia_err = Ia_ref - Ia;
		
		Vs_ref_integ += Tsamp * Kia_cc * (Ia_err - Kaa_cc * (Vs_ref - Vs_ref_set));// 
				
		Vs_ref_ff = Wm * Lamf * If;
		
		Vs_ref = Vs_ref_integ + Vs_ref_ff + Kpa_cc * Ia_err;

		if (Vs_ref > Vdc) Vs_ref_set = Vdc;
		else if (Vs_ref < -Vdc) Vs_ref_set = -Vdc;
		else                    Vs_ref_set = Vs_ref;

		nVs_ref_set = -Vs_ref_set;

		Duty_A = 0.5 * Vs_ref_set * INV_Vdc + 0.5;//0~1

		Duty_B = 0.5 * nVs_ref_set * INV_Vdc + 0.5;//0~1
		


	}
	else {
		Duty_A = 0.5;//0~1

		Duty_B = 0.5;//0~1

	}

		
	//OUTPUT
	aState->outputs[0] = Duty_A;
	aState->outputs[1] = Duty_B;
	aState->outputs[2] = Duty_Af;
	aState->outputs[3] = Duty_Bf;
	// aState->outputs[n] = x ;
	











	
}





