



#include "motorTest.h"


#include "main.h"




void MotorTest_Init(MotorTest_TypeDef *hMotorTest)
{
	MotorTest_TypeDef* h = hMotorTest;

	h->mode = TEST_CURRENT_REF;

	h->button = 0;
	h->p_button = 0;

	h->omega_ref = 0;
	h->ref_sign = 1;

	h->Iq_res_int16 = 0;
	h->theta_res_uint16 = 0;
	h->omega_res_int16 = 0;


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



	if(h->mode == TEST_CURRENT_REF)
	{
		float Iq_ref[4] = {0.0, 0.0, 0.0, 0.0};

		Iq_ref[h->targetChannel] = h->ref_sign * h->volume * 15.0f;

		driveMotor_speed(Iq_ref);
	}
	else if(h->mode == TEST_SPEED_CONTROL)
	{

	}



}


