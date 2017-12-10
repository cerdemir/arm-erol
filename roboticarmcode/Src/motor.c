#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "motor.h" 
Motor_TypeDef motors[MOTOR_COUNT];


void MotorInit()
{
		motors[0].CNT = &(TIM1->CNT);
		motors[0].PWM= &(TIM9->CCR1);
		motors[0].GPIO_Port = GPIOE;
		motors[0].Direction_Pin1=GPIO_PIN_1;
		motors[0].Direction_Pin2=GPIO_PIN_0;	
		motors[0].Enabled= false;
		*motors[0].CNT = motors[0].LastEncoderCount = 30000;
		*motors[0].PWM= 0;			
		motors[0].CurrentPosition= 0;
		motors[0].PID.Kd= 7;
		motors[0].PID.Ki= 0.001;
		motors[0].PID.Kp= 2;
		
}

void MotorDrive(Motor_TypeDef *motor, int32_t value)
{
		if(value>0)
		{
			*motor->PWM = value;
			HAL_GPIO_WritePin(motor->GPIO_Port,motor->Direction_Pin1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->GPIO_Port,motor->Direction_Pin2,GPIO_PIN_RESET);			
		}
		else if(value<0)
		{
			*motor->PWM = -value;
			HAL_GPIO_WritePin(motor->GPIO_Port,motor->Direction_Pin1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->GPIO_Port,motor->Direction_Pin2,GPIO_PIN_SET);	
		}
		else			
		{
			*motor->PWM = 0;
			HAL_GPIO_WritePin(motor->GPIO_Port,motor->Direction_Pin1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->GPIO_Port,motor->Direction_Pin2,GPIO_PIN_RESET);			
		}
}
void MotorCalcPosition (Motor_TypeDef *motor)
{
		motor->CurrentPosition += *motor->CNT - motor->LastEncoderCount;
		motor->LastEncoderCount = *motor->CNT ;
}
float32_t MotorCalcPID(Motor_TypeDef *motor)
{
	if(!motor->Enabled)
		return 0;
	else if (motor->CurrentError >-2 && motor->CurrentError < 2)
		return 0;
	else		
		return arm_pid_f32(&motor->PID, motor->CurrentError);
}
void MotorSetTarget(Motor_TypeDef *motor,int32_t setPoint)
{
	motor->SetpointPosition = setPoint;
	arm_pid_reset_f32(&motor->PID);
	arm_pid_init_f32(&motor->PID, 1);
}
void MotorCalcError (Motor_TypeDef *motor)
{	
		motor->CurrentError = motor->SetpointPosition - motor->CurrentPosition;
}
