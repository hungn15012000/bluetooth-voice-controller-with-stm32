
#include "main.h"
#include<math.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include"i2c-lcd.h"
#define TRIG_PIN GPIO_PIN_10
#define TRIG_PORT GPIOE
#define SET_POINT 300
#define Kp 6
#define Ki 0.05
#define Kd 4
#define PI 3.14159

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
int startwith(const char *pre, const char *str);
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
char tx_buffer[50];
uint8_t so_thu;
uint8_t temp [3];
float so_mot;
char rx_buffer [50];
float number;
int test_temp;
char str2[20] = {};
char a[5][50] ={
            "go forward",
            "go back",
            "go left",
            "go right",
            "stop",
            };
double temporary;
int enable_1 = 0, enable_2 = 0, enable_3 = 0, enable_4 = 0;
typedef struct
{
  double  enc_cnt_prv,
      enc_cnt,
      rate,
      _dt ,
      _max,
      _min,
      _pre_error,
      _integral,
      _Kp ,
      _Ki ,
      _Kd ,
      set_point ,
      speed_present,
      speed_prev ,
      rate_per_sec ,
      distance ,
      count ,
    error  ,
    error_2,
    out_prev ,
  out;
}PID_struct;
void PIDController_Init(PID_struct *pid) {

  /* Clear controller variables */
  pid->_integral = 0;
  pid->_pre_error  = 0;
  pid -> _dt  = 0.01;
  pid -> enc_cnt = 0;
  pid -> enc_cnt_prv = 0;
  pid -> _Kp = Kp;
  pid -> _Ki = Ki;
  pid -> _Kd = Kd;
  pid -> count = 0;
  pid -> distance = 0;
  pid -> _max = 300;
  pid -> _min = 60;
  pid -> distance = 0;
  pid -> speed_present = 1;
  pid->rate = 2;
  pid->out = 0,
  pid->out_prev=0,
  pid->error=0,
  pid->error_2 = 0;
}
PID_struct MOTOR_1;
PID_struct MOTOR_2;
double PID_equation (PID_struct  *pid, double set_point, double rate)
  {
    pid->error = set_point - rate;

    // Proportional term
    double Pout = pid->_Kp * pid->error;

    // Integral term
    pid->_integral += pid->error * pid->_dt;
    double Iout = pid->_Ki * pid->_integral;

    // Derivative term
    double derivative = (pid->error - pid->_pre_error) / pid->_dt;
    double Dout = pid->_Kd * derivative;

    // Calculate total output
    pid-> out = Pout + Iout + Dout;

    // Restrict to max/min
    if(  pid-> out > pid->_max )
    	 pid-> out = pid->_max;
    else if(  pid-> out < pid->_min )
    	 pid-> out = pid->_min;

    // Save error to previous error
    pid->_pre_error = pid->error;


   return pid->out;
  }
void go_forward_1 (double pulse_width)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2, 999);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, 999-pulse_width);
}
void go_forward_2 (double pulse_width)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, 999);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, 999-pulse_width);
}
void go_backward_1 (double pulse_width)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2, 999-pulse_width);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, 999);
}
void go_backward_2 (double pulse_width)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, 999-pulse_width);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, 999);
}
void stop_all ()
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, 0);
}
void substring(const char *str1, int pos1, int pos2)
{
  int j =0;
  for(int i = pos1 ; i <= pos2 ; i++)
  {
    str2[j++] = str1[i];
  }
}
void initial ()
{
	HAL_Delay(1000);
	lcd_clear_display();
	lcd_goto_XY(1, 0);
	lcd_send_string("  get set");
	PIDController_Init(&MOTOR_1);
	PIDController_Init(&MOTOR_2);
	HAL_Delay(2000);
	lcd_goto_XY(2, 0);
	lcd_send_string("running!!!!");
}

void go_straight ( double a) //  a is cm // default v is 44,2cm/1s
{
		initial ();
		a = (a/6.5/PI)*360;
		  MOTOR_1.enc_cnt = 0;
		  __HAL_TIM_GET_COUNTER(&htim2) =0;
	  while( 1){
		  MOTOR_1.enc_cnt = __HAL_TIM_GET_COUNTER(&htim2);
		  if(MOTOR_1.enc_cnt>= 1000000)
			  MOTOR_1.enc_cnt = 0;
		  MOTOR_1.rate =  (MOTOR_1.enc_cnt/1540)*360;
		  double temp = PID_equation(&MOTOR_1, a, MOTOR_1.rate);
		  go_forward_1(temp) ;
		  go_forward_2(temp) ;
		  if(MOTOR_1.rate >= a)
							  {
								  lcd_clear_display();
								  lcd_goto_XY(1, 0);
								  lcd_send_string("     stop!!!!!!!!");
								  break;
								  }

	      HAL_Delay(10);
	  }

	  stop_all();
	  HAL_Delay(2000);
}
void go_back (double a)
{
	initial ();
	a = (a/6.5/PI)*360;
	MOTOR_1.rate = 0;
	__HAL_TIM_GET_COUNTER(&htim2) =0;
		 while( 1){
			  MOTOR_1.enc_cnt = -__HAL_TIM_GET_COUNTER(&htim2);
			  if(MOTOR_1.enc_cnt>= 1000000)
				  MOTOR_1.enc_cnt = 0;
			  MOTOR_1.rate =  (MOTOR_1.enc_cnt/1540)*360;
			  double temp = PID_equation(&MOTOR_1, a, MOTOR_1.rate);
			  go_backward_1(temp) ;
			  go_backward_2(temp) ;
			  if(MOTOR_1.rate >= a)
					  {
						  lcd_clear_display();
						  lcd_goto_XY(1, 0);
						  lcd_send_string("     stop!!!!!!!!");
						  break;
						  }

		      HAL_Delay(10);
		  }
		 stop_all();
		 HAL_Delay(2000);
}
void go_left ( double a)
{
	initial ();
	MOTOR_1.rate = 0;
	__HAL_TIM_GET_COUNTER(&htim2) =0;
	 while( 1){
				  MOTOR_1.enc_cnt =-__HAL_TIM_GET_COUNTER(&htim2);
				  if(MOTOR_1.enc_cnt>= 1000000)
					  MOTOR_1.enc_cnt = 0;
				  MOTOR_1.rate =  (MOTOR_1.enc_cnt/1540)*360;
				  double temp = PID_equation(&MOTOR_1, a, MOTOR_1.rate);
				  go_forward_2(temp);
				  go_backward_1(temp);
				  if(MOTOR_1.rate >=a)
				  {
					  lcd_clear_display();
					  lcd_goto_XY(1, 0);
					  lcd_send_string("     stop!!!!!!!!");
				 	  break;
				  }
			      HAL_Delay(10);
			  }
			 stop_all();
			 HAL_Delay(2000);
}
void go_right ( double a)
{
	initial();

	MOTOR_1.rate = 0;
	__HAL_TIM_GET_COUNTER(&htim2) =0;
	 while( 1){
				  MOTOR_1.enc_cnt =__HAL_TIM_GET_COUNTER(&htim2);
				  if(MOTOR_1.enc_cnt>= 1000000)
					  MOTOR_1.enc_cnt = 0;
				  MOTOR_1.rate =  (MOTOR_1.enc_cnt/1540)*360;
				  double temp = PID_equation(&MOTOR_1, a, MOTOR_1.rate);
				  go_forward_1(temp);
				  go_backward_2(temp);
				  if(MOTOR_1.rate >= a)
				  {
					  lcd_clear_display();
					  lcd_goto_XY(1, 0);
					  lcd_send_string("     stop!!!!!!!!");
				 	  break;
				  }
			      HAL_Delay(10);
			  }
			 stop_all();
			 HAL_Delay(2000);

}

 int startwith(const char *pre, const char *str)
{
  return strncmp(pre, str, strlen(pre))==0 ;
}
void compare ()
{

  for (int i = 0;i<5;i++ )
  {
    if(startwith(a[i],rx_buffer))
    {
    substring(rx_buffer,strlen(a[i]),strlen(rx_buffer));
    number = atoi(str2);
    if(number != 0)
	{
		lcd_goto_XY(1, 0);
		lcd_send_string("data is received");
	}
    else if( number == 0)
    {
    	lcd_goto_XY(1, 0);
    	lcd_send_string("no data is received");
    	 for(int i = 0;i<50;i++)
    	    	rx_buffer[i]='\0';
    	    break;
    }
    switch(i)
    {
    case  0: go_straight (number);
        break;
    case  1: go_back (number);
        break;
    case  2: go_left (number);
        break;
    case  3: go_right (number);
        break;
    case  4: stop_all();
        break;

    }
    number = 0;
    for(int i = 0;i<50;i++)
    	rx_buffer[i]='\0';
    break;
    }
  }
}


void delay (uint32_t time)
{
  __HAL_TIM_SET_COUNTER(&htim1,0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < time);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
  {
    if (Is_First_Captured==0) // if the first value is not captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
      Is_First_Captured = 1;  // set the first captured as true
      // Now change the polarity to falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    else if (Is_First_Captured==1)   // if the first is already captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
      __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

      if (IC_Val2 > IC_Val1)
      {
        Difference = IC_Val2-IC_Val1;
      }

      else if (IC_Val1 > IC_Val2)
      {
        Difference = (0xffff - IC_Val1) + IC_Val2;
      }

      Distance = Difference * .034/2;
      Is_First_Captured = 0; // set it back to false

      // set polarity to rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
    }
  }
}

void HCSR04_Read (void)
{
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  delay(10);  // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
// define 2 encoder motor

void put_to_lcd()
{
		  lcd_clear_display();
    	  lcd_goto_XY(1,0);
          lcd_send_string("  Distance");
          lcd_goto_XY(1,strlen("Distance "));
          char so_in[2] ="\0\0";
          int so_nguyen = Distance;
          sprintf(so_in, "%d", so_nguyen);
          lcd_send_string(so_in);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if(htim->Instance == htim9.Instance)
    {

      HAL_UART_Receive(&huart6, &so_thu,sizeof(so_thu) , 0);
            so_mot = so_thu;
            HCSR04_Read();
            put_to_lcd();
            lcd_goto_XY(2, 0);
            if(number ==0)
            	{
            	lcd_send_string("wait response");
            	}
       HAL_UART_Receive(&huart1,(uint8_t*)rx_buffer,50, 500);
        compare();
//       go_straight(50);
//       go_straight(0);
//       HAL_Delay(10000);

    }

}
int _write  (int file,char *ptr, int len)
{
  int i=0;
  for (i=0;i<len;i++)
  {
    ITM_SendChar((*ptr++));
  }
  return len;
}

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
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
lcd_init();

//lcd_send_string("Hungdeptrai");
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim9);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 71;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
