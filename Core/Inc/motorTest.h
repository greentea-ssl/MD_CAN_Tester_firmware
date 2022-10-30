

#ifndef _MOTOR_TEST_H_
#define _MOTOR_TEST_H_


#include <stdint.h>

#include "../ControlLib/controlLib.h"




typedef enum
{
	TEST_CURRENT_REF,
	TEST_SPEED_CONTROL,
}TestMode;


typedef struct
{

	TestMode mode;

	DOB_TypeDef dob;
	float Kp;

	float omega_ref;
	int ref_sign;

	float Iq_ref;

	int targetChannel;
	float volume;

	int16_t Iq_res_int16;
	uint16_t theta_res_uint16;
	int16_t omega_res_int16;

	float Iq_res;
	float theta_res;
	float omega_res;
	uint8_t status_code;

	uint8_t button;
	uint8_t p_button;


	// ASR

	float Ktn;
	float Jmn;
	float gdis;
	float Ts;



}MotorTest_TypeDef;



void MotorTest_Init(MotorTest_TypeDef *hMotorTest);


void MotorTest_Update(MotorTest_TypeDef *hMotorTest);





#endif /* MOTOR_TEST_H */


