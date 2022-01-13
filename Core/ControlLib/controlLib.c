


#include "controlLib.h"



void LPF1st_Init(LPF1st_TypeDef* h, float cutoff, float Ts)
{
	h->cutoff = cutoff;
	h->coeff = 1.0f / (h->cutoff * Ts + 1.0f);
	h->y = 0.0f;
}

float LPF1st_Update(LPF1st_TypeDef* h, float u)
{
	h->y = h->y * h->coeff + u * (1.0f - h->coeff);
	return h->y;
}

float LPF1st_getY(LPF1st_TypeDef* h)
{
	return h->y;
}




void DOB_Init(DOB_TypeDef* h, float Ktn, float Jmn, float g_dis, float Ts)
{

	LPF1st_Init(&h->lpf, g_dis, Ts);

	h->Ktn = Ktn;
	h->Jmn = Jmn;
	h->gdis = g_dis;
	h->Ts = Ts;
	h->y = 0.0;
}


float DOB_Update(DOB_TypeDef* h, float Iq_ref, float omega_m)
{
	float LPF_in = h->Ktn * Iq_ref + h->gdis * h->Jmn * omega_m;
	h->y = LPF1st_Update(&h->lpf, LPF_in) - h->gdis * h->Jmn * omega_m;
	return h->y;
}


float DOB_GetTdis(DOB_TypeDef* h)
{
	return h->y;
}





float limitter(float u, float min, float max)
{
	if(u < min) return min;
	if(u > max) return max;
	return u;
}






