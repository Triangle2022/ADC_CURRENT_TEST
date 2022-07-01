/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
//#include "FastMath.h"
#include <stdio.h>
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
 ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t dma_buffer_1[2];
uint32_t dma_buffer_2[2];
uint32_t dma_buffer_3[2];
int encoder;
int encoder_updated;

int phase_uni;

double a_current;
double b_current;
double c_current;

/////////////////configuration value//////////////////
const int cpr = 2048; //resolution of the encoder
const float pi = 3.14159265358; //M_PI
const int pole_pair = 10; //10 pole pair
const int period = 250; //250us

//For the angle calculation(global variable)
int phase;
int multiply;
float mech_angle;
float elec_angle;
float pole_pair_angle;

//For the timing calculation(global variable)
float t1;
float t2;
float t3;


float angle;

int32_t a_current_2;
int32_t b_current_2;
int32_t c_current_2;

float alpha_current;
float beta_current;

float d_current;
float q_current;

int speed2 = 1500;

uint32_t tick;
int count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
 * @brief Retargets the C library printf function to the USART
 * @param None
 * @retval None
 */

PUTCHAR_PROTOTYPE{
	if (ch == '\n') HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}


void clark_park(double a, double b, double c)
{

	a_current_2 = ((((double)a-2048)/4096*3.3)/(33/2.7))/0.005;
	b_current_2 = ((((double)b-2048)/4096*3.3)/(33/2.7))/0.005;
	c_current_2 = ((((double)c-2048)/4096*3.3)/(33/2.7))/0.005;


	double alpha = (((double)2/(double)3)*a_current_2)-((double)1/(double)3)*(b_current_2+c_current_2);
	double beta = ((double)1/(double)sqrt(3))*(b_current_2-c_current_2);
	alpha_current = alpha;
	beta_current = beta;

	if(phase_uni == 1)
	{
		angle = (double)0*(M_PI/(double)180);
	}
	else if(phase_uni == 2)
	{
		angle = (double)60*(M_PI/(double)180);
	}
	else if(phase_uni == 3)
	{
		angle = (double)120*(M_PI/(double)180);
	}
	else if(phase_uni == 4)
	{
		angle = (double)180*(M_PI/(double)180);
	}
	else if(phase_uni == 5)
	{
		angle = (double)240*(M_PI/(double)180);
	}
	else if(phase_uni == 6)
	{
		angle = (double)300*(M_PI/(double)180);
	}

	d_current = cos(angle)*alpha_current+sin(angle)*beta_current;
	q_current = -sin(angle)*alpha_current+cos(angle)*beta_current;


}

const float Multiplier = 81.4873308631f;
const float sint[] = {
    0,0.012296,0.024589,0.036879,0.049164,0.061441,0.073708,0.085965,0.098208,0.11044,0.12265,0.13484,0.14702,0.15917,0.17129,0.18339,0.19547,0.20751,0.21952,0.2315,0.24345,0.25535,0.26722,0.27905,0.29084,0.30258,0.31427,0.32592,0.33752,0.34907,0.36057,0.37201,0.38339,0.39472,0.40599,0.41719,0.42834,0.43941,0.45043,0.46137,0.47224,0.48305,0.49378,0.50443,0.51501,0.52551,0.53593,0.54627,0.55653,0.5667,0.57679,0.58679,0.5967,0.60652,0.61625,0.62589,0.63543,0.64488,0.65423,0.66348,0.67263,0.68167,0.69062,0.69946,0.70819,0.71682,0.72534,0.73375,0.74205,0.75023,0.75831,0.76626,0.77411,0.78183,0.78944,0.79693,0.80429,0.81154,0.81866,0.82566,0.83254,0.83928,0.84591,0.8524,0.85876,0.865,0.8711,0.87708,0.88292,0.88862,0.89419,0.89963,0.90493,0.9101,0.91512,0.92001,0.92476,0.92937,0.93384,0.93816,0.94235,0.94639,0.95029,0.95405,0.95766,0.96113,0.96445,0.96763,0.97066,0.97354,0.97628,0.97887,0.98131,0.9836,0.98574,0.98774,0.98958,0.99128,0.99282,0.99422,0.99546,0.99656,0.9975,0.99829,0.99894,0.99943,0.99977,0.99996,1,0.99988,0.99962,0.9992,0.99863,0.99792,0.99705,0.99603,0.99486,0.99354,0.99207,0.99045,0.98868,0.98676,0.98469,0.98247,0.9801,0.97759,0.97493,0.97212,0.96916,0.96606,0.96281,0.95941,0.95587,0.95219,0.94836,0.94439,0.94028,0.93602,0.93162,0.92708,0.9224,0.91758,0.91263,0.90753,0.9023,0.89693,0.89142,0.88579,0.88001,0.87411,0.86807,0.8619,0.8556,0.84917,0.84261,0.83593,0.82911,0.82218,0.81512,0.80793,0.80062,0.7932,0.78565,0.77798,0.7702,0.7623,0.75428,0.74615,0.73791,0.72956,0.72109,0.71252,0.70384,0.69505,0.68616,0.67716,0.66806,0.65886,0.64956,0.64017,0.63067,0.62108,0.6114,0.60162,0.59176,0.5818,0.57176,0.56163,0.55141,0.54111,0.53073,0.52027,0.50973,0.49911,0.48842,0.47765,0.46682,0.45591,0.44493,0.43388,0.42277,0.4116,0.40036,0.38906,0.37771,0.36629,0.35483,0.3433,0.33173,0.32011,0.30843,0.29671,0.28495,0.27314,0.26129,0.2494,0.23748,0.22552,0.21352,0.20149,0.18943,0.17735,0.16523,0.15309,0.14093,0.12875,0.11655,0.10432,0.092088,0.079838,0.067576,0.055303,0.043022,0.030735,0.018443,0.0061479,-0.0061479,-0.018443,-0.030735,-0.043022,-0.055303,-0.067576,-0.079838,-0.092088,-0.10432,-0.11655,-0.12875,-0.14093,-0.15309,-0.16523,-0.17735,-0.18943,-0.20149,-0.21352,-0.22552,-0.23748,-0.2494,-0.26129,-0.27314,-0.28495,-0.29671,-0.30843,-0.32011,-0.33173,-0.3433,-0.35483,-0.36629,-0.37771,-0.38906,-0.40036,-0.4116,-0.42277,-0.43388,-0.44493,-0.45591,-0.46682,-0.47765,-0.48842,-0.49911,-0.50973,-0.52027,-0.53073,-0.54111,-0.55141,-0.56163,-0.57176,-0.5818,-0.59176,-0.60162,-0.6114,-0.62108,-0.63067,-0.64017,-0.64956,-0.65886,-0.66806,-0.67716,-0.68616,-0.69505,-0.70384,-0.71252,-0.72109,-0.72956,-0.73791,-0.74615,-0.75428,-0.7623,-0.7702,-0.77798,-0.78565,-0.7932,-0.80062,-0.80793,-0.81512,-0.82218,-0.82911,-0.83593,-0.84261,-0.84917,-0.8556,-0.8619,-0.86807,-0.87411,-0.88001,-0.88579,-0.89142,-0.89693,-0.9023,-0.90753,-0.91263,-0.91758,-0.9224,-0.92708,-0.93162,-0.93602,-0.94028,-0.94439,-0.94836,-0.95219,-0.95587,-0.95941,-0.96281,-0.96606,-0.96916,-0.97212,-0.97493,-0.97759,-0.9801,-0.98247,-0.98469,-0.98676,-0.98868,-0.99045,-0.99207,-0.99354,-0.99486,-0.99603,-0.99705,-0.99792,-0.99863,-0.9992,-0.99962,-0.99988,-1,-0.99996,-0.99977,-0.99943,-0.99894,-0.99829,-0.9975,-0.99656,-0.99546,-0.99422,-0.99282,-0.99128,-0.98958,-0.98774,-0.98574,-0.9836,-0.98131,-0.97887,-0.97628,-0.97354,-0.97066,-0.96763,-0.96445,-0.96113,-0.95766,-0.95405,-0.95029,-0.94639,-0.94235,-0.93816,-0.93384,-0.92937,-0.92476,-0.92001,-0.91512,-0.9101,-0.90493,-0.89963,-0.89419,-0.88862,-0.88292,-0.87708,-0.8711,-0.865,-0.85876,-0.8524,-0.84591,-0.83928,-0.83254,-0.82566,-0.81866,-0.81154,-0.80429,-0.79693,-0.78944,-0.78183,-0.77411,-0.76626,-0.75831,-0.75023,-0.74205,-0.73375,-0.72534,-0.71682,-0.70819,-0.69946,-0.69062,-0.68167,-0.67263,-0.66348,-0.65423,-0.64488,-0.63543,-0.62589,-0.61625,-0.60652,-0.5967,-0.58679,-0.57679,-0.5667,-0.55653,-0.54627,-0.53593,-0.52551,-0.51501,-0.50443,-0.49378,-0.48305,-0.47224,-0.46137,-0.45043,-0.43941,-0.42834,-0.41719,-0.40599,-0.39472,-0.38339,-0.37201,-0.36057,-0.34907,-0.33752,-0.32592,-0.31427,-0.30258,-0.29084,-0.27905,-0.26722,-0.25535,-0.24345,-0.2315,-0.21952,-0.20751,-0.19547,-0.18339,-0.17129,-0.15917,-0.14702,-0.13484,-0.12265,-0.11044,-0.098208,-0.085965,-0.073708,-0.061441,-0.049164,-0.036879,-0.024589,-0.012296,0
};

float FastSin(float theta){
    while (theta < 0.0f) theta += 6.28318530718f;
    while (theta >= 6.28318530718f) theta -= 6.28318530718f;
    return sint[(int) (Multiplier*theta)] ;
    }
float FastCos(float theta){
    return FastSin(1.57079632679f - theta);
    }


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) //This should be the main loop
{

	///current_define
	if(phase_uni ==1)
	{
		b_current_2 = (int32_t)dma_buffer_2[0]-2048;
		c_current_2 = (int32_t)dma_buffer_3[0]-2048;
		a_current_2 = (-b_current_2)+(-c_current_2);
		angle = (float)0*((float)M_PI/(float)180);
	}
	else if(phase_uni == 2)
	{

		c_current_2 = (int32_t)dma_buffer_3[0]-2048;
		a_current_2 = -(c_current_2/2);
		b_current_2 = -(c_current_2/2);
		angle = (float)60*((float)M_PI/(float)180);
	}
	else if(phase_uni == 3)
	{
		a_current_2 = (int32_t)dma_buffer_1[0]-2048;
		c_current_2 = (int32_t)dma_buffer_3[0]-2048;
		b_current_2 = (-a_current_2)+(-c_current_2);
		angle = (float)120*((float)M_PI/(float)180);
	}
	else if(phase_uni == 4)
	{
		a_current_2 = (int32_t)dma_buffer_1[0]-2048;
		b_current_2 = -(a_current_2/2);
		c_current_2 = -(a_current_2/2);
		angle = (float)180*((float)M_PI/(float)180);
	}
	else if(phase_uni == 5)
	{
		a_current_2 = (int32_t)dma_buffer_1[0]-2048;
		b_current_2 = (int32_t)dma_buffer_2[0]-2048;
		c_current_2 = (-a_current_2)+(-b_current_2);
		angle = (float)240*((float)M_PI/(float)180);
	}
	else if(phase_uni == 6)
	{
		b_current_2 = (int32_t)dma_buffer_2[0]-2048;
		a_current_2 = -(b_current_2/2);
		c_current_2 = -(b_current_2/2);
		angle = (float)300*((float)M_PI/(float)180);
	}

	alpha_current = (float)(0.666666)*(float)a_current_2-(float)(0.333333)*((float)b_current_2+(float)c_current_2);
	beta_current = (float)(0.57735026)*((float)b_current_2-(float)c_current_2);



	//d_current = FastCos((float)angle)*(float)alpha_current+FastSin((float)angle)*(float)beta_current;
	//q_current = -FastSin(angle)*(float)alpha_current+FastCos(angle)*(float)beta_current;


}

void update_encoder()
{
	encoder = (TIM2->CNT>>2);
	if(encoder > 2048)
	{
		encoder = (encoder%2048);
	}
}

void calculation(int encoder)
{
    //set encoder val between 1,2048
    while (encoder < 1) encoder += 2048;
    while (encoder > 2048) encoder -= 2048;

    encoder = (cpr+1)-encoder; // ->when the reverse should be activated

    //basic angle calculation
    mech_angle  = encoder*(2*pi/cpr);
    multiply = (int)(((mech_angle*pole_pair)/(2*pi)));
    elec_angle = (mech_angle*pole_pair)-(float)((2*pi)*(float)multiply);

    //control angle calculation
    multiply = (int)(((elec_angle)/(pi/3)));
    pole_pair_angle = (elec_angle)-(float)((pi/3)*(float)multiply);
    //control angle should be have pi/4 offset compare to the pole_pair_angle

    //calculate the phase
    phase = (elec_angle/(pi/3))+1;

    t1 = period*sin(M_PI/3 - pole_pair_angle);
    t2 = period*sin(pole_pair_angle);
    t3 = period-t1-t2;

    //std::cout << mech_angle << "t1  " << t1 << "  t2  " << t2 << "  t3  " << t3 << " total " <<  t1+t2+t3 << "phase "<< phase << "  angle "<< encoder << "pole_pair_angle " << pole_pair_angle <<std::endl;
}
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim12,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim12) < us);  // wait for the counter to reach the us input in the parameter
}

void drive(int voltage,int phase) //main driving function
{
	switch (phase) {
		case 1:
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(1,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(2,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(7,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(2,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(1,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);

			break;
		case 2:
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(3,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(2,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(7,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(2,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(3,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			break;
		case 3:
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(3,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(4,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(7,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(4,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(3,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			break;
		case 4:
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(4,voltage);
			delay_us((uint16_t)t1/3);
			execute_phase(4,voltage);
			delay_us((uint16_t)t1/3);
			execute_phase(7,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(4,voltage);
			delay_us((uint16_t)t1/3);
			execute_phase(5,voltage);
			delay_us((uint16_t)t2);
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			break;
		case 5:
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(5,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(6,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(7,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(6,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(5,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			break;
		case 6:
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(1,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(6,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(7,voltage);
			delay_us((uint16_t)t3/3);
			execute_phase(6,voltage);
			delay_us((uint16_t)t1/2);
			execute_phase(1,voltage);
			delay_us((uint16_t)t2/2);
			execute_phase(0,voltage);
			delay_us((uint16_t)t3/3);
			break;
	}

}


void execute_phase(int phase,int speed) //Gate mosfet control algorithm
{
	phase_uni = phase;

	switch (phase) {
		case 0:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = 0;
			break;
		case 1:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = 0;
			break;
		case 2:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = 0;
			break;
		case 3:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = 0;
			break;
		case 4:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = speed;
			break;
		case 5:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = speed;
			break;
		case 6:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = speed;
			break;
		case 7:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = speed;
			break;

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim12);

  HAL_TIM_Base_Start(&htim3);
  delay_us(96); // for the pwm gitter control
  HAL_TIM_Base_Start(&htim4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); //enable the gate driver

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buffer_1, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)dma_buffer_2, 2);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)dma_buffer_3, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  execute_phase(1, 1400); //communate for the phase 1 , angle = 0
  HAL_Delay(1000); //delay waiting for the commutation
  __HAL_TIM_SET_COUNTER(&htim2,0); //reset encoder value becuase commutation has been done
  tick = HAL_GetTick();
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  update_encoder();
	  drive(speed2,phase);
	  //drive(2000,30);

//	  		if(HAL_GetTick() - tick > 2000L)
//	  		{
//	  			count +=1;
//	  			tick = HAL_GetTick();
//	  		}
//	  		if(count%2 == 1)
//	  		{
//	  			speed2 = 1600;
//	  		}
//	  		else if(count %2 ==0)
//	  		{
//	  			speed2 = 1600;
//	  		}
/*
	  }
*/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 42-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 2100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim4.Init.Period = 2100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim12.Init.Prescaler = 42-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
