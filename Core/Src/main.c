/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLING_TIME_MIN    (1000 * 60)

#define PULSE_PER_CIRCLE  13000

#define R_MOTOR_NOR SET
#define R_MOTOR_REV !R_MOTOR_NOR

#define L_MOTOR_NOR RESET
#define L_MOTOR_REV !L_MOTOR_NOR

#define DEBUG
#define TAG "main"

#ifdef DEBUG
#define printf_t(format, ...) printf( format "\r\n", ##__VA_ARGS__)
#define info(format, ...) printf("[\t"TAG"]info:" format "\r\n", ##__VA_ARGS__)
#define debug(format, ...) printf("[\t"TAG"]debug:" format "\r\n", ##__VA_ARGS__)
#define error(format, ...) printf("[\t"TAG"]error:" format "\r\n",##__VA_ARGS__)
#else
#define printf(format, ...)
#define info(format, ...)
#define debug(format, ...)
#define error(format, ...)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int64_t RMotorCount, LMotorCount;
double RVelocity,LVelocity;                         //Cycle per minute
double RVelBias, LVelBias;
double RTargetVel, LTargetVel;
double velKp=800.0, velKi=1600.0;
double RPWM=0, LPWM=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
    * @breif    Start to computing the speeds of motors.
    * @note     None
    * @param    None
    * @retval   None
    */
void MotorSpeedCompute_Start(){
    HAL_TIM_Base_Start_IT(&htim1);
}

/**
    * @breif    Start to count the encoder.
    * @note     None
    * @param    None
    * @retval   None
    */
void Encoder_Start(){

}

/**
    * @breif    Start to generate PWM signals to control motors.
    * @note     None
    * @param    None
    * @retval   None
    */
void MotorPWM_Start(){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/**
    * @breif    Use incremental PI algorithm to renew the PWM signals.
    * @note     None
    * @param    None
    * @retval   None
    */
void IncrementalPI(void) {
    static double RVelBias, LVelBias, LastRVelBias, LastLVelBias;
    RVelBias = RVelocity - RTargetVel;                //compute current bias
    LVelBias = LVelocity - LTargetVel;                //compute current bias
    RPWM -= velKp * (RVelBias - LastRVelBias) / 100 + velKi * RVelBias / 1000;        //incremental PI controller
    LPWM -= velKp * (LVelBias - LastLVelBias) / 100 + velKi * LVelBias / 1000;        //incremental PI controller
    if (RPWM < 0) { RPWM = 0.0; }
    if (LPWM < 0) { LPWM = 0.0; }
    if (RPWM - 1000.0 > 0) { RPWM = 1000.0; }
    if (LPWM - 1000.0 > 0) { LPWM = 1000.0; }
    LastRVelBias = RVelBias;                    //save last velocity bias
    LastLVelBias = LVelBias;                    //save last velocity bias
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, RPWM);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, LPWM);                                 //Renew PWN
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    int cont = 0;
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
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    Encoder_Start();
    MotorPWM_Start();
    MotorSpeedCompute_Start();
    RTargetVel = 80;

        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, RESET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        cont++;
        cont%=1000;
        if(cont == 0)   RTargetVel = 50;
        if(cont == 500)  RTargetVel = 230;
        IncrementalPI();
        printf("%lf, %lf, %lf, %d\r\n", RVelocity, RTargetVel, RPWM, cont);
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {                                         //Computing Motor's velocity
        static int64_t lastRMotorCount, lastLMotorCount;
        int16_t differenceOfRMotorCount = RMotorCount - lastRMotorCount, differenceOfLMotorCount = LMotorCount - lastLMotorCount;
        RVelocity = (double) differenceOfRMotorCount * SAMPLING_TIME_MIN / PULSE_PER_CIRCLE;
        LVelocity = (double) differenceOfLMotorCount * SAMPLING_TIME_MIN / PULSE_PER_CIRCLE;
        lastRMotorCount = RMotorCount;
        lastLMotorCount = LMotorCount;                                              //Save Motor count
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENCODER1_A_Pin){                                                //Encoder count
        if(HAL_GPIO_ReadPin(ENCODER1_B_GPIO_Port, ENCODER1_B_Pin) == R_MOTOR_NOR)   RMotorCount++;
        else    RMotorCount--;
    }
    else if(GPIO_Pin == ENCODER2_A_Pin){
        if(HAL_GPIO_ReadPin(ENCODER2_B_GPIO_Port, ENCODER2_B_Pin) * HAL_GPIO_ReadPin(ENCODER2_A_GPIO_Port, ENCODER2_A_Pin)== L_MOTOR_NOR)   LMotorCount++;
        else    LMotorCount--;
    }
}

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
