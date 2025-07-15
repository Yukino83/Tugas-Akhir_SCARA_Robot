/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//PID Posisi
typedef struct
{
	float kp;
	float ki;
	float kd;

	float p;
	float i;
	float d;

	float error;
	float prev_error;
	float setpoint;
	float feedback;

	float output;
	float max_output;

	float Offset;
} pid_t ;

pid_t motorT1;
pid_t motorT2;
pid_t motorZ;

pid_t SpeedT1;
pid_t SpeedT2;
pid_t SpeedZ;

int delay = 0;
int status = 0;
int cnt = 0;
int delay2 = 0;
float last_t1 = 0;
float last_t2 = 0;

int flags1 = 0; int flags2 = 0; int flags3 = 0;

//Inverse Kinematics
float l1 = 25.8, l2 = 12.5;
float theta1_rad, theta1, theta2_rad, theta2;
float k1,k2;
float pi = 3.1415926535;

//PID_S SpeedT1;
//PID_S SpeedT2;

int var = 0;
int state = 0;
short enc_read_1 = 0;
short enc_read_2 = 0;
short enc_read_3 = 0;

float enc_vel_1 = 0;
short enc_vel_2 = 0;
float enc_vel_3 = 0;

long int global_enc_1 = 0;
long int global_enc_2 = 0;
long int global_enc_3 = 0;
long int enc_res1 =0;

int dir1 = 0;
int dir2 = 0;
char raw_array[23];
int TIM_CNT = 0, TIM_CNT2 = 0, TIM_LOG = 0;
//short int cnt = 0,
short int speed_z = 999;
float z_cm = 0, dpr = 0.8, gb_rat = 64;
float max_pwm = 900;

//Rasio Ppr Encoder Motor
#define PPR_EN 22
#define GR_RAT_T2 3
#define GR_RAT_T1 4
#define GR_RAT_MOTOR 64

float t1 = 0, t2 = 0;
float Sudut_1 = 0, Sudut_2 = 0;
int rec_T1 = 0, rec_T2 = 0, rec_state = 0;

//Rasio Gear Teta 1 dan Teta 2


//float ppr_t1 = 22 * 64 * gr_rat_t2;
//float ppr_t2 = 22 * 64 * gr_rat_t2;

float ppr_t1 = PPR_EN * GR_RAT_MOTOR * GR_RAT_T1;
float ppr_t2 = PPR_EN * GR_RAT_MOTOR * GR_RAT_T2;

//float error = 0, prev_error = 0, P = 0, I = 0, D = 0, max_speed = 999, min_speed = -999, output = 0;
float input_speed;
int8_t b1a = 0, b1b = 0, b2a = 0, b2b = 0, b3a = 0, b3b = 0;
int limit_1, limit_2, limit_3;
//BEGIN CUSTOM FUNCTION HEREEE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) //using UART1
  	{
    	memcpy(&b1a, raw_array + 3, 1); //parameters : (destination, source, data size) //use & if it's a variable // + 3 because the first 3 datas (start from 0) are already filled with the first 3 characters // int8_t size is 1 byte
    	memcpy(&b1b, raw_array + 4, 1);
    	memcpy(&b2a, raw_array + 5, 1);
    	memcpy(&b3a, raw_array + 6, 1);
    	memcpy(&b2b, raw_array + 7, 1);
    	memcpy(&b3b, raw_array + 8, 1);

    	HAL_UART_Receive_DMA(&huart1,   (uint8_t*)raw_array,   23); //receives the data array again after being parsed
  	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
    {
		if(!(raw_array[0] == 'A' && raw_array[1] == 'B' && raw_array[2] == 'C'))
        {
			HAL_UART_AbortReceive(&huart1);
   	        HAL_UART_Receive_DMA(&huart1,   (uint8_t*)raw_array,   23);
        }
    }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
    {
		HAL_UART_AbortReceive(&huart1);
   	 	HAL_UART_Receive_DMA(&huart1,   (uint8_t*)raw_array,   23);
    }
}

float rad_to_deg(float rad)
{
    float deg = rad * (180/pi);
    return deg;
}

float deg_to_rad(float deg)
{
    float rad = deg * (pi/180);
    return rad;
}


void PID(pid_t *pid, float kp, float ki, float kd, float setpoint, float feedback, float max_output)
{
	pid -> setpoint = setpoint;
	pid -> feedback = feedback;
	pid -> max_output = max_output;

	pid -> kp = kp;
	pid-> ki = ki;
	pid -> kd = kd;

	pid -> error = pid -> setpoint - pid -> feedback;

	if (fabs(pid -> error) <= 0.05) //0.9
	{
		pid -> output = 0;
		pid->i = 0;
		pid->prev_error = 0;
		return;
	}

	pid-> p = pid -> kp * pid -> error;
	pid -> i += pid ->ki * pid -> error;
	pid -> d = pid ->kd * (pid -> error - pid -> prev_error);
	pid -> prev_error = pid ->error;

	if(pid -> i >= pid -> max_output)
	{
		pid -> i =  pid -> max_output;
	}
	else if(pid -> i <  -(pid -> max_output))
	{
		pid -> i = -(pid -> max_output);
	}

	pid -> output = pid -> p + pid -> i + pid -> d;

	if(pid -> output >= pid -> max_output)
	{
		pid -> output =  pid -> max_output;
	}
	else if(pid -> output <  -(pid -> max_output))
	{
		pid -> output = -(pid -> max_output);
	}
}
	
void IK(float x, float y, float vel){
	  // Calculate theta2 using the cosine rule
		theta2_rad = acos(((x * x) + (y * y) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2));
		theta2 = rad_to_deg(theta2_rad);

		// Use atan2 to calculate theta1 to account for quadrant
		k1 = l1 + l2 * cos(theta2_rad);
		k2 = l2 * sin(theta2_rad);
		theta1_rad = atan2(y, x) - atan2(k2, k1);
		theta1 = rad_to_deg(theta1_rad);

//		PID(&motorT1, 100, 0, 0, theta1, t1, vel);
//		PID(&SpeedT1, 50, 0, 0, motorT1.output, enc_vel_1, max_pwm);
//		motor_run(1, (int)SpeedT1.output);
//
//		PID(&motorT2, 100, 0, 0, -theta2, t2, vel);
//		PID(&SpeedT2, 50, 0, 0, motorT2.output, enc_vel_2, max_pwm);
//		motor_run(2, (int)SpeedT2.output);
}



void motor_run(int motor, int speed) //starts from 1 NOT 0
{
	if(speed >= 0)
	{
		dir1 = 1;
		dir2 = 0;
	}
	else if(speed < 0)
	{
		dir1 = 0;
		dir2 = 1;
	}
	speed = abs(speed);
	if(motor == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, dir2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, dir1);
		TIM2->CCR1 = speed; //PA5, TIM2_CH1
	}
	else if(motor == 2)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, dir2);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, dir1);
		TIM3->CCR1 = speed; //PB4, TIM3_CH1
	}
	else if(motor == 3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, dir1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, dir2);
		TIM2->CCR3 = speed; //PB10, TIM2_CH3
	}
}

void limit_switch(){
	limit_1 = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
	limit_2 = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
	limit_3 = !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
}

//TIMER INTERRUPT HEREEEEEE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim12) //timer 12
	{
		if(TIM_CNT >= 10) //to set the sampling time in ms if you set the prescaler correctly. MAKE SURE TO DECLARE THE COUNTING VARIABLE FIRST!!
		{

			//RUN CODE HERE
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1);

			enc_read_1 = ((-1) * TIM1->CNT);
			enc_vel_1 = (float)enc_read_1;


			enc_read_2 = TIM4->CNT;
			enc_vel_2 = TIM4->CNT;

			enc_read_3 = ((-1) * TIM5->CNT);
			enc_vel_3 =  (float)enc_read_3;

			global_enc_1 += (long int)enc_read_1;
			global_enc_2 += (long int)enc_read_2;
			global_enc_3 += (long int)enc_read_3;

			TIM5->CNT = 0;
			TIM1->CNT = 0;
			TIM4->CNT = 0;


			t1 = (360 /ppr_t1) * global_enc_1;
			t2 = (360 / ppr_t2) * global_enc_2;
			z_cm = global_enc_3 / (8706.7 / 5);


//			Sudut_1 =
//			Sudut_2 = t2;



			switch(status) {
			case 0:
				limit_switch();
				if(limit_1 == 0)
				{
				PID(&SpeedT1, 100, 0.5, 10, -10, enc_vel_1, max_pwm);
				motor_run(1, (int)SpeedT1.output);
				}
				else
				{
					motor_run(1, 0);
					global_enc_1 = 0;
				}

				if(limit_2 == 0)
				{
					PID(&SpeedT2, 100, 0.5, 10, 10, enc_vel_2, max_pwm);
					motor_run(2, (int)SpeedT2.output);
				}
				else
				{
					motor_run(2, 0);
					global_enc_2 = 0;
				}

				if(limit_3 == 0)
				{
					PID(&SpeedZ, 150, 25, 0, 32, enc_vel_3, max_pwm);
					motor_run(3, (int)SpeedZ.output);
				}
				else
				{
					motor_run(3, 0);
					global_enc_3 = 0;
				}


				if(limit_1 == 1 && limit_2 == 1 && limit_3 == 1)
//				if(limit_3 == 1)
				{
					cnt++;
					if(cnt >= 100)
					{
						enc_read_1 = 0;
						enc_read_2 = 0;
						enc_read_3 = 0;
						cnt = 0;
						status++;
					}
				}
				break;

			case 1:
				motor_run(1, 0);
				motor_run(2, 0);

				if(limit_3 == 0)
				{
					PID(&SpeedZ, 50, 0, 0, 40, enc_vel_3, max_pwm);
					motor_run(3, (int)SpeedZ.output);

				}
				else
				{
					motor_run(3, 0);
					global_enc_3 = 0;

				}
				if(limit_3 == 1){
					cnt++;
					if(cnt >= 200)
					{
						global_enc_1 = 0;
						global_enc_2 = 0;
						global_enc_3 = 0;

						TIM1->CNT = 0;    // Reset encoder untuk motor 1
						TIM5->CNT = 0;
						TIM4->CNT = 0;
						cnt = 0;
						status++;
//						status = 80;
					}
				}

				break;

			case 2:
				PID(&motorZ, 550, 40, 35, -10, z_cm, 999); // -18cm 600,25,0 Menurunkan Z Axis //-25cm 550,10,
//				PID(&SpeedZ, 100, 0, 0, motorZ.output, enc_vel_3, max_pwm);
				motor_run(3, motorZ.output);
				if(fabs(motorZ.error) <= 0.05)
				{
					delay++;
					if(delay >= 100)
					{

						delay = 0;
						motor_run(3, 0);
						global_enc_3 = 0; //on
						global_enc_1 = 0;
						global_enc_2 = 0;

						status++; //on
//						status = 25;
					}
				}
				break;

			case 3:
				global_enc_3 = 0;
				global_enc_1 = 0;
				global_enc_2 = 0;
				delay++;
				if(delay >= 200)
				{
					status++;
					delay = 0;
				}
				break;

			case 4:	//Home to Start T1 90, T2 -180
				PID(&motorT1, 50, 0.3, 2, 95.0, t1, 10); //100, 0, 0
				PID(&SpeedT1, 100, 0.5, 10, motorT1.output, enc_vel_1, max_pwm); //100,2,5
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 50, 0.3, 2, -178.0, t2, 12);
				PID(&SpeedT2, 100, 0.5, 10, motorT2.output, enc_vel_2, max_pwm); //50
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 1.0) //1.0
//				if(fabs(t1) <= 1.0)
				{

//					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
//					motorT1.error = 0;
//					SpeedT1.i = 0;
					flags1 = 1;
				}
				if (fabs(motorT2.error) <= 1.0) //1.0
				{
//					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
					flags2 = 1;
				}

				if (fabs(motorT1.error) <= 1.0 && fabs(motorT2.error) <= 1.0)
				{
					delay++;
					if(delay >= 100)
					{
						global_enc_1 = 0; //on
						global_enc_2 = 0; //on

						flags1 = 0;
						flags2 = 0; //on
						delay = 0;

						status++;
//						status = 5;

					}

				}
				break;

			case 5:
				global_enc_1 = 0;
				global_enc_2 = 0;
				delay++;
				if (delay >= 200)
				{
					status = 7;
					delay = 0;
				}
				break;

			case 6:
				IK(25,20,10); //Start to object test
//				PID(&motorZ, 550, 0, 0, -10, z_cm, 999);
				motor_run(3, 0);

				PID(&motorT1, 100, 0, 0, theta1, t1, 10);
				PID(&SpeedT1, 100, 0.5, 10, motorT1.output, enc_vel_1, max_pwm); //50
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 50, 0, 0, theta2, t2, 12);
				PID(&SpeedT2, 100, 0.5, 10, motorT2.output, enc_vel_2, max_pwm); //50
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 0.8) //2.0
				{
//					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
					flags1 = 1;
				}
				if (fabs(motorT2.error) <= 0.8) //2.0
				{
//					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
					flags2 = 1;
				}
//				else if (fabs(motorZ.error) <= 1.0){
//					motor_run(3, 0);
//					flags3 = 1;
//
//
//				}

				if (flags1 == 1 && flags2 == 1)
				{
					delay++;
					if(delay >= 300)
					{
//						flags1 = 0;
//						flags2 = 0;
////						flags3 = 0;
//						status++;
						delay = 0;


					}

				}
				break;

			case 7:
//				enc_res1 += (long int)enc_read_1;
//				Sudut_1 = (360 /ppr_t1) * enc_res1;
//				Sudut_T2 = (360 / ppr_t2) * global_enc_2;

//				rec_state = 1;
				IK(25,20,10); //Start to Object
				PID(&SpeedZ, 100, 0, 0, 0, z_cm, max_pwm);

				PID(&motorT1, 50, 0.5, 2, theta1, t1, 12); //100,0,0
				PID(&SpeedT1, 100, 0.5, 10, motorT1.output, enc_vel_1, max_pwm);
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 50, 0.5, 2, theta2, t2, 10);
				PID(&SpeedT2, 100, 0.5, 10, motorT2.output, enc_vel_2, max_pwm); //80, 3, 10
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 1.0)
				{
					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
					last_t1 = t1;
					flags1 = 1;

				}
				if (fabs(motorT2.error) <= 1.0)
				{
					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
					last_t2 = t2; //tambahan baru
					flags2 = 1;

				}
				if (flags1 == 1 && flags2 == 1)
				{
					delay++;
					if(delay > 100)
					{
						motor_run(1, 0);
						motor_run(2, 0);
						flags1 = 0;
						flags2 = 0;
						status++;

						delay = 0;
					}
					enc_read_1 = 0;
					enc_res1 = 0;

				}
				break;

			case 8:
				delay++;
				if (delay >= 50)
				{
					status++;
					delay = 0;
				}
				break;

			case 9:
//				rec_state = 0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 1); //Nyalakan Pompa
				PID(&motorZ, 550, 40, 35, -8, z_cm, 999); //Menurunkan Z Axis
//				PID(&SpeedZ, 100, 0, 0, motorZ.output, enc_vel_3, max_pwm);
				motor_run (3, motorZ.output);

				if(fabs(motorZ.error) <= 0.05)
				{

					delay++;
					if(delay >= 100)
					{
						delay = 0;
						global_enc_3 = 0;
						status++;
					}
				}
				break;

			case 10:
//				rec_state = 1;
				PID(&motorZ, 550, 40, 35, 7, z_cm, 999); //Menaikan Z Axis 550,45,35
//				PID(&SpeedZ, 100, 0, 0, motorZ.output, enc_vel_3, max_pwm);
				motor_run (3, motorZ.output);
				if(fabs(motorZ.error) <= 0.05)
				{
					rec_state = 0;
					delay++;
					if(delay >= 100)
					{
						delay = 0;
						global_enc_3 = 0;
//						rec_state = 0;
						motorT1.error = 0; //tambahan 30juni
						motorT2.error = 0; //tambahan 30juni

						t1 = last_t1; //tambahan baru
						t2 = last_t2;
						status++;

					}
				}
				break;


				//-------------------------------------------------------------------
				//===================================================================
				//-------------------------------------------------------------------






			case 11:
//				enc_res1 += (long int)enc_read_1;
//				Sudut_1 = (360 /ppr_t1) * enc_res1;
				last_t1 = 0;
				last_t2 = 0;

				rec_state = 1;
				IK(-20,30,10); //Object to Placement Position
				PID(&SpeedZ, 100, 0, 0, 0, z_cm, max_pwm);

				PID(&motorT1, 50, 0.25, 10, theta1, t1, 12); //50,0.5,2 // best 2 30,0,0
				PID(&SpeedT1, 100, 0.5, 10, motorT1.output, enc_vel_1, max_pwm); //100, 2, 5
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 50, 0.2, 8, theta2, t2, 10); //50, 0.5, 2.... best2 30,0,0
				PID(&SpeedT2, 100, 0.5, 10, motorT2.output, enc_vel_2, max_pwm); // 100,0,0 ..  80,3,5
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 1.0)
				{
//					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
					flags1 = 1;
				}
				if (fabs(motorT2.error) <= 1.0)
				{
//					PID(&SpeedT2, 100, 5, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
					flags2 = 1;
				}
				if (fabs(motorT1.error) <= 1.0 && fabs(motorT2.error) <= 1.0)
				{
					motor_run(1, 0);
					motor_run(2, 0);
					delay++;
					if(delay >= 100)
					{
						motor_run(1, 0);
						motor_run(2, 0);

						delay = 0;
						flags1 = 0;
						flags2 = 0;

						status++;


					}

				}
				break;

			case 12:
//				PID(&motorT1, 100, 0, 2, theta1, t1, 12); //100, 5, 0
//				PID(&SpeedT1, 100, 0.5, 10, motorT1.output, enc_vel_1, max_pwm); //100, 2, 5
//				motor_run(1, (int)SpeedT1.output);
//
//				PID(&motorT2, 50, 3, 6, theta2, t2, 10); //100, 3, 6.... 50,3,6
//				PID(&SpeedT2, 100, 0.5, 10, motorT2.output, enc_vel_2, max_pwm); // 100,0,0 ..  80,3,5
//				motor_run(2, (int)SpeedT2.output);
//
//				if(fabs(motorT1.error) <= 0.3)
//				{
////					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
//					motor_run(1, 0);
//					flags1 = 1;
//				}
//				else if (fabs(motorT2.error) <= 0.5)
//				{
////					PID(&SpeedT2, 100, 5, 0, 0, enc_vel_2, max_pwm);
//					motor_run(2, 0);
//					flags2 = 1;
//				}
//				if (flags1 == 1 && flags2 == 1)
//				{
//					motor_run(1, 0);
//					motor_run(2, 0);
//					delay++;
//					if(delay > 300)
//					{
//						motor_run(1, 0);
//						motor_run(2, 0);
//
//						delay = 0;
//						flags1 = 0;
//						flags2 = 0;
//						last_t1 = t1;
//						last_t2 = t2;
//
//						status++;
//
//
//					}
//
//				}
				rec_state = 0;
				status++;
				break;

			case 13:
				rec_state = 0;


				PID(&motorZ, 550, 45, 35, -5.5, z_cm, 999); //Menurunkan Z Axis
				motor_run(3, motorZ.output);
				if(fabs(motorZ.error) <= 0.05)
				{
					delay++;
					if(delay >= 50)
					{
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 0);
						delay = 0;
						global_enc_3 = 0;
						status++;
					}
				}
				break;

			case 14:
				rec_state = 0;
				motor_run(3, 0);
				delay++;
				if (delay >= 300)
				{
					delay = 0;
					status++;
				}
				break;

			case 15:
				PID(&motorZ, 550, 40, 35, 6, z_cm, 999); //Menaikan Z Axis
				motor_run(3, motorZ.output);
				if(fabs(motorZ.error) <= 0.05)
				{
					delay++;
					if(delay >= 100)
					{
						t1 = last_t1;
						t2 = last_t2;
						delay = 0;
						status++;
					}
				}
				break;

			case 16:
				IK(28,0,10); //Place to Start
//				PID(&motorZ, 550, 0, 0, -10, z_cm, 999);
				motor_run(3, 0);

				PID(&motorT1, 100, 0, 0, theta1, t1, 12); //100,0,0 best
				PID(&SpeedT1, 100, 2, 5, motorT1.output, enc_vel_1, max_pwm); //100,2,5
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 50, 0, 0, theta2, t2, 10); //50,0,0 best
				PID(&SpeedT2, 50, 2, 5, motorT2.output, enc_vel_2, max_pwm); //50,2,5
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 1)
				{
//					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
					flags1 = 1;
				}
				else if (fabs(motorT2.error) <= 1)
				{
//					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
					flags2 = 1;
				}
//				else if (fabs(motorZ.error) <= 1.0){
//					motor_run(3, 0);
//					flags3 = 1;
//
//
//				}

				if (flags1 == 1 && flags2 == 1) {
					delay++;
					if(delay >= 300){
//						global_enc_1 = 0;
//						global_enc_2 = 0;
////						global_enc_3 = 0;
//						flags1 = 0;
//						flags2 = 0;
////						flags3 = 0;
						delay = 0;
//						status++;


					}
//
				}
				break;

//			case 16:
//				delay++;
//				if (delay >= 100)
//				{
//					motor_run(1, 0);
//					motor_run(2, 0);
//					motor_run(3, 0);
//				}
//				break;

			case 17:
				if(b1a == 1)
				{
					motor_run(1, -900);
				}
				else if(b1b == 1)
				{
					motor_run(1, 900);
				}
				else
				{
					motor_run(1, 0);
				}


				if(b2a == 1)
				{
					motor_run(3, 550);
				}
				else if(b2b == 1)
				{
					motor_run(3, -550);
				}
				else
				{
					motor_run(3, 0);
				}
				break;

			case 40://Start Position
				IK(20,10,10);
				PID(&motorT1, 100, 0, 0, 90, t1, 10);
				PID(&SpeedT1, 100, 2, 5, motorT1.output, enc_vel_1, max_pwm); //50
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 100, 0, 0, -180, t2, 12);
				PID(&SpeedT2, 100, 3, 0, motorT2.output, enc_vel_2, max_pwm); //50
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 1.0)
				{
//					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
					flags1 = 1;
				}
				if (fabs(motorT2.error) <= 1.0)
				{
//					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
					flags2 = 1;
				}

				if (flags1 == 1 && flags2 == 1)
				{
					delay++;
					if(delay >= 300)
					{
						global_enc_1 = 0;
						global_enc_2 = 0;

						flags1 = 0;
						flags2 = 0;
//						flags3 = 0;
						delay = 0;
						status++;

					}

				}
				break;


			case 50:
				IK(20, 10, 10); //Test IK Home to Start
				PID(&SpeedZ, 100, 0, 0, 0, z_cm, max_pwm);

				PID(&motorT1, 100, 0, 0, 90 + theta1, t1, 10);
				PID(&SpeedT1, 50, 0, 0, motorT1.output, enc_vel_1, max_pwm);
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 100, 0, 0, -theta2, t2, 12);
				PID(&SpeedT2, 50, 0, 0, motorT2.output, enc_vel_2, max_pwm);
				motor_run(2, (int)SpeedT2.output);

				if(fabs(motorT1.error) <= 5){
					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
				}
				else if (fabs(motorT2.error) <= 5){
					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
				}

				// Jika sudah mencapai target
				else if (fabs(motorT1.error) <= 5.0 && fabs(motorT2.error) <= 5.0) {
					cnt++;
					if(cnt >= 50) { // Tunggu stabil
						cnt = 0;
						status = 100; // Pindah ke test PID
						motor_run(1, 0);
						motor_run(2, 0);
					}
				}

				break;

			case 80:
//				rec_state = 0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 1); //Nyalakan Pompa
				PID(&motorZ, 550, 45, 35, -10, z_cm, 999); //Menurunkan Z Axis
//				PID(&SpeedZ, 100, 0, 0, motorZ.output, enc_vel_3, max_pwm);
				motor_run (3, motorZ.output);

				if(fabs(motorZ.error) <= 0.05)
				{

					delay++;
					if(delay >= 100)
					{
						delay = 0;
						global_enc_3 = 0;
						status++;
					}
				}
				break;

			case 81:
				rec_state = 1;
				PID(&motorZ, 550, 40, 35, 7, z_cm, 999); //Menaikan Z Axis 550,45,35
//				PID(&SpeedZ, 100, 0, 0, motorZ.output, enc_vel_3, max_pwm);
				motor_run (3, motorZ.output);
				if(fabs(motorZ.error) <= 0.05)
				{
					rec_state = 0;
					delay++;
					if(delay >= 100)
					{
						delay = 0;
//						global_enc_3 = 0;
						rec_state = 0;

//						t1 = last_t1; //tambahan baru
//						t2 = last_t2;
//						status++;

					}
				}
				break;



			case 100:
//				Test PID Kecepatan dan Posisi
				PID(&motorT1, 100, 2, 5, 45, t1, 8); //arah setpoint positif berlawanan limit
				PID(&SpeedT1, 100, 0, 0, motorT1.output, enc_vel_1, max_pwm);
				motor_run(1, (int)SpeedT1.output);

				PID(&motorT2, 50, 0, 0, -90, t2, 8); //arah setpoint minus berlawanan limit
				PID(&SpeedT2, 100, 0, 0, motorT2.output, enc_vel_2, max_pwm);
				motor_run(2, (int)SpeedT2.output);

				PID(&motorZ, 100, 0, 0, -2, z_cm, 40); // setpoint minus turun kebawah
				PID(&SpeedZ, 100, 0, 0, motorZ.output, z_cm, max_pwm);
				motor_run(3, (int)SpeedZ.output);

				if(fabs(motorT1.error) <= 2){
					PID(&SpeedT1, 100, 0, 0, 0, enc_vel_1, max_pwm);
					motor_run(1, 0);
				}
				else if (fabs(motorT2.error) <= 5){
					PID(&SpeedT2, 100, 0, 0, 0, enc_vel_2, max_pwm);
					motor_run(2, 0);
				}
				else if (fabs(motorT1.error) <= 2 && fabs(motorT2.error) <= 5){
//					status++;
					motor_run(1, 0);
					motor_run(2, 0);
				}
				break;

			case 300:
				break;
//				PID(&SpeedT1, 50, 0, 0, 0, enc_vel_1, max_pwm);  // Hitung output PID untuk kecepatan
//				motor_run(1, (int)SpeedT1.output);
			case 500:
//				PID(&motorT1, 100, 2, 5, 30, t1, max_pwm);
				PID(&SpeedT1, 100, 3, 0, -8, enc_vel_1, 999);  // Hitung output PID untuk kecepatan
				motor_run(1, (int)SpeedT1.output);
				break;
			}

			cnt++;
			TIM_CNT = 0; //resets the counter //put it after the main code
		}
		if (TIM_CNT2 == 100)
		{
			if(rec_state == 1)
			{
				TIM_LOG += 100;
				rec_T1 = t1;
				rec_T2 = t2;

			}
			else
			{
				TIM_LOG = 0;
				rec_T1 = rec_T2 = 0;
			}
			TIM_CNT2 = 0;
		}
		TIM_CNT++;
		TIM_CNT2++;

	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1); //turns on built in LED
  //motor speed pins initialization
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  //encoder initialization
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  //serial comm init
  HAL_UART_Receive_DMA(&huart1,   (uint8_t*)raw_array,   21);


//homing();

  //timer interrupt init
  HAL_TIM_Base_Start_IT(&htim12); //timer interrupt in timer12


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84 - 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000 - 1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD3 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
