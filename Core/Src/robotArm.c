/*
 * robotArm.c
 *
 *  Created on: 11 mar 2022
 *      Author: Mateusz Kozaczek
 */
#include "robotArm.h"
#include "math.h"

void stepper_init(struct stepper_s *_stepper, TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef *_dirPortGPIO, uint16_t _dirPinGPIO)
{

	_stepper->timer.htim = _htim;
	_stepper->timer.channel = _channel;
	_stepper->dirGPIO.PORT = _dirPortGPIO;
	_stepper->dirGPIO.PIN = _dirPinGPIO;
	_stepper->speed = SPEED_DEFAULT;
	stepper_set_speed(_stepper, SPEED_DEFAULT);
}


void stepper_set_angle_degree(struct stepper_s *_stepper, float _angle)
{
	_stepper->mode = working;
	stepper_set_speed(_stepper, _stepper->speed);
	if (_angle > 0) {
		stepper_set_direction(_stepper, CW);
		_stepper->steps_to_count = _angle * (STEP_PER_REVOLUTION * MICRO_STEP) / 360;
	}
	else if (_angle < 0){
		stepper_set_direction(_stepper, CCW);
		_stepper->steps_to_count = -_angle * (STEP_PER_REVOLUTION * MICRO_STEP) / 360;
	}
	_stepper->step_counter = 0;


	if(0 == _stepper->steps_to_count)
	{
		stepper_stop(_stepper);
	}

	HAL_TIM_PWM_Start_IT(_stepper->timer.htim, _stepper->timer.channel);
}
void stepper_set_angle_radian(struct stepper_s *_stepper, float _angle)
{
	_stepper->mode = working;
	stepper_set_speed(_stepper, _stepper->speed);
	if (_angle > 0) {
		stepper_set_direction(_stepper, CW);
		_stepper->steps_to_count = _angle * (STEP_PER_REVOLUTION * MICRO_STEP) / (2.0 * M_PI);
	}
	else if (_angle < 0){
		stepper_set_direction(_stepper, CCW);
		_stepper->steps_to_count = -_angle * (STEP_PER_REVOLUTION * MICRO_STEP) / (2.0 * M_PI);
	}
	_stepper->step_counter = 0;

	if(0 == _stepper->steps_to_count)
	{
		stepper_stop(_stepper);
	}

	HAL_TIM_PWM_Start_IT(_stepper->timer.htim, _stepper->timer.channel);
}

void stepper_stop(struct stepper_s *_stepper)
{
	_stepper->mode = idle;

	__HAL_TIM_SET_COMPARE(_stepper->timer.htim, _stepper->timer.channel, 0);
	HAL_TIM_PWM_Stop(_stepper->timer.htim, _stepper->timer.channel);

}

void stepper_set_direction(struct stepper_s *_stepper, stepper_direction _dir)
{
	_stepper->direction=_dir;
	if(_dir == CCW)
		HAL_GPIO_WritePin(_stepper->dirGPIO.PORT, _stepper->dirGPIO.PIN, GPIO_PIN_SET);
	else if(_dir == CW)
		HAL_GPIO_WritePin(_stepper->dirGPIO.PORT, _stepper->dirGPIO.PIN, GPIO_PIN_RESET);
}
void stepper_set_speed(struct stepper_s *_stepper, uint32_t _speed)
{
	uint32_t counter, freq;

	if(_speed > 100)
	{
		_speed = 100;
	}
	else if(_speed == 0)
	{
		stepper_stop(_stepper);
		return;
	}

	freq = (_speed * (STEPPER_MOTOR_MAX_FREQ_HZ - STEPPER_MOTOR_MIN_FREQ_HZ))/STEPPER_MOTOR_MAX_SPEED;
	counter = HAL_RCC_GetPCLK1Freq() / (_stepper->timer.htim->Init.Prescaler * freq);

	__HAL_TIM_SET_COUNTER(_stepper->timer.htim, 0);
	__HAL_TIM_SET_AUTORELOAD(_stepper->timer.htim, counter - 1);
	__HAL_TIM_SET_COMPARE(_stepper->timer.htim, _stepper->timer.channel, (counter/2) - 1);
}

void robotArm_init(struct robotArm_s*_robotArm){

	HAL_GPIO_WritePin(ENABLE_STEPS_GPIO_Port, ENABLE_STEPS_Pin, GPIO_PIN_RESET);


	stepper_init(&_robotArm->high_Stepper, &HIGH_STEP_HTIM, HIGH_STEP_TIM_CHANNEL, HIGH_DIR_GPIO_Port, HIGH_DIR_Pin);

	stepper_init(&_robotArm->low_Stepper, &LOW_STEP_HTIM, LOW_STEP_TIM_CHANNEL, LOW_DIR_GPIO_Port, LOW_DIR_Pin);

	stepper_init(&_robotArm->rot_Stepper, &ROT_STEP_HTIM, ROT_STEP_TIM_CHANNEL, ROT_DIR_GPIO_Port, ROT_DIR_Pin);

	_robotArm->gripper_Stepper.timer.htim = &GRIPPER_HTIM;
	_robotArm->gripper_Stepper.state = close;

	_robotArm->highBigGear = HIGH_ENDSTOP;
	_robotArm->lowBigGear = LOW_ENDSTOP;
	_robotArm->rotBigGear = ROT_ENDSTOP;
	_robotArm->highSmallGear = HIGH_ENDSTOP * GEAR_RATIO;
	_robotArm->lowSmallGear = LOW_ENDSTOP * GEAR_RATIO;
	_robotArm->rotSmallGear = ROT_ENDSTOP * GEAR_RATIO;


}
void robotArm_Geometry_calculateRadian(struct robotArm_s *_robotArm, float _Xmm, float _Ymm, float _Zmm) {


	float xmm=_Xmm;
	float ymm=_Ymm;
	float zmm=_Zmm;
	float high, low, rot, rrot, rside, alpha, phi, omega;

   rrot =  sqrt((xmm * xmm) + (ymm * ymm));
   rside = sqrt((rrot * rrot) + (zmm * zmm));

   rot = asin(xmm / rrot);
   high = 2.0 * asin(rside / (2.0 * 120.0));

   alpha = (M_PI - high) / 2.0;
   phi = alpha - (M_PI / 2.0);
   omega = acos(rrot / rside);

   if (zmm > 0) {
	   low = - phi - omega;
   } else {
	   low = omega - phi;
   }

    high = (M_PI / 2) - high + low;
   _robotArm->calcHigh = high;
   _robotArm->calcLow = low;
   _robotArm->calcRot = rot;

}

void robotArm_moveToXYZ(struct robotArm_s * _robotArm, float _Xmm, float _Ymm, float _Zmm){

	float highAngleToMove, lowAngleToMove, rotAngleToMove;


	robotArm_Geometry_calculateRadian(_robotArm, _Xmm, _Ymm,_Zmm);

	highAngleToMove=(_robotArm->calcHigh-_robotArm->highBigGear)*GEAR_RATIO;
	lowAngleToMove=(_robotArm->calcLow-_robotArm->lowBigGear)*GEAR_RATIO;
	rotAngleToMove=(_robotArm->calcRot-_robotArm->rotBigGear)*GEAR_RATIO;


		if(lowAngleToMove)stepper_set_angle_radian(&_robotArm->low_Stepper, lowAngleToMove);

		if(highAngleToMove)stepper_set_angle_radian(&_robotArm->high_Stepper, highAngleToMove);

		if(rotAngleToMove)stepper_set_angle_radian(&_robotArm->rot_Stepper, rotAngleToMove);

	while(!robotArm_isIdle(_robotArm));

	_robotArm->Xmm = _Xmm;
	_robotArm->Ymm = _Ymm;
	_robotArm->Zmm = _Zmm;


	_robotArm->highBigGear = _robotArm->calcHigh;
	_robotArm->lowBigGear = _robotArm->calcLow;
	_robotArm->rotBigGear = _robotArm->calcRot;

	_robotArm->highSmallGear=_robotArm->highBigGear * GEAR_RATIO;
	_robotArm->lowSmallGear=_robotArm->lowBigGear * GEAR_RATIO;
	_robotArm->rotSmallGear=_robotArm->rotBigGear * GEAR_RATIO;

}


unsigned int robotArm_isIdle(struct robotArm_s * _robotArm){
	if(_robotArm->high_Stepper.mode==idle &&
	   _robotArm->low_Stepper.mode==idle &&
	   _robotArm->rot_Stepper.mode==idle
	)return 1;
	return 0;
}

void robotArm_Gripper(struct robotArm_s * _robotArm, gripper_state _state){

	if(_robotArm->gripper_Stepper.state==_state) return;

	_robotArm->gripper_Stepper.mode = working;
	_robotArm->gripper_Stepper.step_counter = 0;

	if(_state == open){
		_robotArm->gripper_Stepper.state = open;
		_robotArm->gripper_Stepper.steps_to_count = 500;
	}
	else if (_state == close){
		_robotArm->gripper_Stepper.state = close;
		_robotArm->gripper_Stepper.steps_to_count = 900;
	}
	HAL_TIM_Base_Start_IT(_robotArm->gripper_Stepper.timer.htim);
	while(_robotArm->gripper_Stepper.mode == working);


}
void gripper_stop(struct gripper_s *_gripper)
{
	_gripper->mode = idle;

	HAL_TIM_Base_Stop_IT(_gripper->timer.htim);

}
