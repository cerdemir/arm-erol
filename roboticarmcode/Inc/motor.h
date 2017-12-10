

#include <stdint.h>
#include "core_cm4.h"   
#include "stm32f407xx.h"   
#include "arm_math.h"
#include <stdbool.h>

#define MOTOR_COUNT 1

typedef struct
{
	__IO uint32_t *PWM;
	__IO uint32_t *CNT;
	GPIO_TypeDef *GPIO_Port;
	uint16_t Direction_Pin1;
	uint16_t Direction_Pin2;		
	bool Enabled;	
	int32_t LastEncoderCount;
	int32_t CurrentPosition;
	int32_t SetpointPosition;		
	int32_t CurrentError;
	int32_t LastError;		
	arm_pid_instance_f32 PID;	
} Motor_TypeDef;

void MotorInit();
void MotorDrive(Motor_TypeDef *motor, int32_t value);
void MotorCalcPosition (Motor_TypeDef *motor);
void MotorCalcError (Motor_TypeDef *motor);
float32_t MotorCalcPID(Motor_TypeDef *motor);
void MotorSetTarget(Motor_TypeDef *motor,int32_t setPoint);




	

