


#ifndef _CONTROL_LIB_H_
#define _CONTROL_LIB_H_


typedef struct
{
	float cutoff;
	float coeff;
	float y;
}LPF1st_TypeDef;

void LPF1st_Init(LPF1st_TypeDef* h, float cutoff, float Ts);

float LPF1st_Update(LPF1st_TypeDef* h, float u);

float LPF1st_getY(LPF1st_TypeDef* h);




typedef struct
{
	LPF1st_TypeDef lpf;

	float Ktn;
	float Jmn;
	float gdis;
	float Ts;
	float y;

}DOB_TypeDef;

void DOB_Init(DOB_TypeDef* h, float Ktn, float Jmn, float g_dis, float Ts);

float DOB_Update(DOB_TypeDef* h, float Iq_ref, float omega_m);

float DOB_GetTdis(DOB_TypeDef* h);





float limitter(float u, float min, float max);






#endif /* _CONTROL_LIB_H_ */


