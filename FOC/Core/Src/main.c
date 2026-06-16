/* 
 ******************************************************************************
  �ջ����������

  ******************************************************************************
 */
 
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "foc_math.h"
#include "oled.h"
#include "mt6816.h"
#include "ec11.h"
#include "vofa.h"
#include "menu.h"
#include "foc.h"
#include "sensorless.h"
#include <math.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_SPI3_Init();
  MX_CORDIC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_TIM1_Init();
  
  /* USER CODE BEGIN 2 */
  
  // 1. ���ѻ�������
  HAL_OPAMP_Start(&hopamp1);
  HAL_OPAMP_Start(&hopamp2);
  
  OLED_Init();
  MT6816_Init();
  DAC_Verf_1_65V_Start(); // �˷�ƫ��
  
  OLED_Clear();
  OLED_Printf(0, 0, "FOC Starting...");

  // 2. ����ִ�� FOC ��ʼ����ˮ��
  FOC_Calibrate_ADC(); 
  FOC_Align_Zero();    
  
  Motor_Identify_RsLs();  // auto-measure Rs/Ls
  // ?? ����ؼ�����ʼ�� PID �������޷���֮ǰ����©���ˣ�
  FOC_Init();          
  
  // 3. ���� ADC �жϣ��ѿ���Ȩ������̨�� FOC_Control_Loop��
  HAL_ADCEx_InjectedStart_IT(&hadc1); 
  HAL_ADCEx_InjectedStart(&hadc2);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Menu_Analyse();
      HAL_Delay(100);
      
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) 
    {
        // �����е�������������ոշ�װ�õĺں���
        FOC_Control_Loop(); 
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
















///**
//  ******************************************************************************
//  �����������

//  ******************************************************************************
//  */
///* USER CODE END Header */

///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "adc.h"
//#include "cordic.h"
//#include "dac.h"
//#include "dma.h"
//#include "opamp.h"
//#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
//#include "mt6816.h"
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "oled.h"
//#include <math.h>
//#include "vofa.h"
///* USER CODE END Includes */

///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */

//#define PI 3.14159265358979f
//#define ARR_VALUE 4250.0f  // ��Ķ�ʱ�� ARR ֵ

//// ?? ��������ȫ�ֱ�������������ԭʼ�� ADC �Ĵ���ֵ��
//uint32_t raw_u_global = 0; 
//uint32_t raw_v_global = 0;

//float real_Iu = 0.0f;
//float real_Iv = 0.0f;

//// ��������˷�ƫ���� 2048 (1.65V)������ϵ���� 0.008 (�������Ӳ����)
//float offset_u = 2048.0f; 
//float offset_v = 2048.0f;
//float k_amps = 0.0080058f; 

//float real_mt_angle = 0.0f; // ��������ʵ�Ƕ�

///* USER CODE BEGIN 0 */

//// ==========================================
//// ?? ���Ĳ�����ADC ��̬���У׼����
//// ==========================================
//void FOC_Calibrate_ADC1(void)
//{
//    OLED_Clear();
//    OLED_Printf(0, 0, "Calibrating...");

//    // 1. ǿ����� 50% ռ�ձȡ���ʱ U/V/W ���� 6V����Ȧѹ��Ϊ 0��������������Ϊ 0A
//    TIM1->CCR1 = (uint32_t)(0.5f * ARR_VALUE);
//    TIM1->CCR2 = (uint32_t)(0.5f * ARR_VALUE);
//    TIM1->CCR3 = (uint32_t)(0.5f * ARR_VALUE);
//    
//    // 2. ��ʱ 100ms�����˷ŵ�·�ĵ��ݳ�ŵ�ƽ�ȣ�������ϵͳ�ĵ��밲������
//    HAL_Delay(100);

//    // 3. �ռ� 500 ����ʵ�� ADC �Ĵ���ֵ
//    uint32_t sum_u = 0;
//    uint32_t sum_v = 0;
//    
//    // ��Ϊ��� ADC �ж��Ѿ��ں�̨�� 20kHz ������ raw_u_global �� raw_v_global
//    // ����ֻ��Ҫÿ�� 1ms ȥ��͵�顱һ������ֵ��ץ 500 ����ƽ�����������ܿ��˴�ϵײ�ʱ��ķ��ա�
//    for(int i = 0; i < 500; i++) 
//    {
//        sum_u += raw_u_global;
//        sum_v += raw_v_global;
//        HAL_Delay(1);
//    }

//    // 4. �������ʵ�����ƫ�ã�ֱ�Ӹ��ǵ�����������д���� 2048.0f��
//    offset_u = (float)sum_u / 500.0f;
//    offset_v = (float)sum_v / 500.0f;

//    // 5. �Ѳ��������ʵ��������Ļ�ϣ��������ۿ����ǲ��� 1920 ���ң�
//    OLED_Clear();
//    OLED_Printf(0, 0, "U offset: %.1f", offset_u);
//    OLED_Printf(0, 16, "V offset: %.1f", offset_v); // �����ָ� 16��������� OLED ����� Y ����
//    HAL_Delay(3000); // ͣ�� 3 �룬���㿴������
//}

///* USER CODE END 0 */
//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//    if (hadc == &hadc1) 
//    {
//// 1. ����ȫ�ֱ�����������ѭ������ VOFA+
//        raw_u_global = hadc1.Instance->JDR1;
//        raw_v_global = hadc2.Instance->JDR1; 

//        // 2. �����ʵ������ʵ�İ���(A)ֵ
//     real_Iu = (offset_u - (float)raw_u_global) * k_amps;
//	real_Iv = (offset_v - (float)raw_v_global) * k_amps;
//    }
//}

//// ==========================================
//// ?? ���Ĳ�����
//// ==========================================
//float open_loop_angle = 0.0f; 
//// ֱ�Ӹ� 0.10f (10% ռ�ձȣ�Լ 1.2V)��
//// ���� 1 ŷķ���裬�����Լ 1.2A ��ǿ����������������������
//float target_voltage  = 0.10f; 

//// ==========================================
//// ���� SVPWM �������� (û���κζ����߼�)
//// ==========================================
//void OpenLoop_SVPWM_Run(void)
//{
//    // 1. ��������������Ҳ� (��� 120 ��)
//    float Ua = target_voltage * cosf(open_loop_angle);
//    float Ub = target_voltage * cosf(open_loop_angle - 2.0f * PI / 3.0f);
//    float Uc = target_voltage * cosf(open_loop_angle + 2.0f * PI / 3.0f);

//    // 2. �����ֵ����Сֵ
//    float U_max = fmaxf(Ua, fmaxf(Ub, Uc));
//    float U_min = fminf(Ua, fminf(Ub, Uc));
//    
//    // 3. ��������ģ��ѹ (�������ؼ�)
//    float U_com = -(U_max + U_min) / 2.0f;

//    // 4. ���Ҳ���������������ƽ�Ƶ� 0~1 ����
//    float DutyA = Ua + U_com + 0.5f;
//    float DutyB = Ub + U_com + 0.5f;
//    float DutyC = Uc + U_com + 0.5f;

//    // 5. ����ɱȽϼĴ���(CCR)��ֵ��ֱ���Ƹ��ײ�Ӳ��
//    TIM1->CCR1 = (uint32_t)(DutyA * ARR_VALUE);
//    TIM1->CCR2 = (uint32_t)(DutyB * ARR_VALUE);
//    TIM1->CCR3 = (uint32_t)(DutyC * ARR_VALUE);

//    // 6. �ǶȲ��� (���ƿ�����ת�ٶ�)
//    open_loop_angle += 0.5f; 
//    if (open_loop_angle > 2.0f * PI) {
//        open_loop_angle -= 2.0f * PI;
//    }
//}
///* USER CODE END PTD */

///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */

///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
///* USER CODE END PM */

///* Private variables ---------------------------------------------------------*/
///* USER CODE BEGIN PV */
///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
///* USER CODE BEGIN PFP */
///* USER CODE END PFP */

///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
///* USER CODE END 0 */

///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* MCU Configuration--------------------------------------------------------*/
//  HAL_Init();
//  SystemClock_Config();

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_USART3_UART_Init();
//  MX_DAC1_Init();
//  MX_SPI3_Init();
//  MX_CORDIC_Init();
//  MX_ADC1_Init();
//  MX_ADC2_Init();
//  MX_OPAMP1_Init();
//  MX_OPAMP2_Init();
//  MX_TIM1_Init();

//  /* USER CODE BEGIN 2 */
//  
//  // 1. ��������
//	VOFA_Init();
//  DAC_Verf_1_65V_Start();
//  HAL_OPAMP_Start(&hopamp1);
//  HAL_OPAMP_Start(&hopamp2);
//  HAL_ADCEx_InjectedStart_IT(&hadc1); 
//  HAL_ADCEx_InjectedStart(&hadc2);
//	
//  MT6816_Init();
// 
//  OLED_Init();
//  OLED_Clear();
//  OLED_Printf(0, 0, "Push to Start!");

//  // 2. ���� 6 · PWM ���
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//     HAL_TIM_Base_Start(&htim1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	TIM1->CCR4 = (uint32_t)(ARR_VALUE - 5.0f);
//  // ?? 3. ������Ҫ�������ʹ����բ��û����䲨�γ���ȥ��
//  __HAL_TIM_MOE_ENABLE(&htim1); 
//FOC_Calibrate_ADC1();
//  /* USER CODE END 2 */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//      // ѭ������
//      OpenLoop_SVPWM_Run();
//      MT6816_Update();
//      real_mt_angle = mt6816.elec_angle_rad;
//	  VOFA_Send_DMA(open_loop_angle, real_mt_angle, 0.0f, 0.0f);
//	// VOFA_Send_DMA((float)raw_u_global, (float)raw_v_global, real_Iu, real_Iv);
//      // ��ʱ 1ms ˢ�¡�һȦ��ҪԼ 600ms���ٶ�����
//      HAL_Delay(1);
//      
//    /* USER CODE END WHILE */
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}

///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
//  RCC_OscInitStruct.PLL.PLLN = 85;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

//void Error_Handler(void)
//{
//  __disable_irq();
//  while (1)
//  {
//  }
//}