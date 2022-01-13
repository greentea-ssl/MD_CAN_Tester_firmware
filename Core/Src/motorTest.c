



#include "motorTest.h"

#include "controlLib.h"

#include "main.h"

#include <math.h>


void MotorTest_Init(MotorTest_TypeDef *hMotorTest)
{
	MotorTest_TypeDef* h = hMotorTest;

	//h->mode = TEST_CURRENT_REF;
	h->mode = TEST_SPEED_CONTROL;

	h->button = 0;
	h->p_button = 0;

	h->omega_ref = 0;
	h->ref_sign = 1;

	h->Iq_res_int16 = 0;
	h->theta_res_uint16 = 0;
	h->omega_res_int16 = 0;

	h->Kp = 0.1;

	h->Jmn = 5.2E-5;
	h->Ktn = (60.0f / (320 * 2 * M_PI));
	h->Ts = 1E-3;
	h->gdis = 20;

	DOB_Init(&h->dob, h->Ktn, h->Jmn, h->gdis, h->Ts);


}


void MotorTest_Update(MotorTest_TypeDef *hMotorTest)
{

	MotorTest_TypeDef* h = hMotorTest;

	h->targetChannel = getChannel();

	LED_blink_times = h->targetChannel;

	h->button = !HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

	if(h->button && !h->p_button)
	{
		h->ref_sign *= -1;
	}
	h->p_button = h->button;


	// Unit conversion
	h->Iq_res = h->Iq_res_int16 * 0.0009765625f; // 1/1024
	h->theta_res =h->theta_res_uint16 * 0.0003834951969714103f; // 2*pi/16384
	h->omega_res = h->omega_res_int16 * 0.03125f; // 1/32


	float Iq_ref_array[4] = {0.0, 0.0, 0.0, 0.0};

	float Tdis_est = DOB_Update(&h->dob, h->Iq_ref, h->omega_res);

	if(h->mode == TEST_CURRENT_REF)
	{
		h->Iq_ref = h->ref_sign * h->volume * 15.0f;
	}
	else if(h->mode == TEST_SPEED_CONTROL)
	{
		h->omega_ref = h->ref_sign * h->volume * 200.0f;

		float error = h->omega_ref - h->omega_res;
		h->Iq_ref = h->Kp * error + Tdis_est / h->Ktn;
		if(h->Iq_ref < -15.0) h->Iq_ref = -15.0;
		if(h->Iq_ref > 15.0) h->Iq_ref = 15.0;

	}

	Iq_ref_array[h->targetChannel] = h->Iq_ref;

	driveMotor_speed(Iq_ref_array);


}


