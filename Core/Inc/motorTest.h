

#ifndef _MOTOR_TEST_H_
#define _MOTOR_TEST_H_


#include <stdint.h>


typedef enum
{
	TEST_CURRENT_REF,
	TEST_SPEED_CONTROL,
}TestMode;


typedef struct
{

	TestMode mode;

	float omega_ref;
	int ref_sign;

	float Iq_ref;

	int targetChannel;
	float volume;


	int16_t Iq_res_int16;
	uint16_t theta_res_uint16;
	int16_t omega_res_int16;


	uint8_t button;
	uint8_t p_button;


}MotorTest_TypeDef;



void MotorTest_Init(MotorTest_TypeDef *hMotorTest);


void MotorTest_Update(MotorTest_TypeDef *hMotorTest);





#endif /* MOTOR_TEST_H */


