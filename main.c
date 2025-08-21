/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
	*/
  //��������ã���ʼ���������ȴ���ͨ��stm32cubemx������ɡ� �����Ҫ�����µ����裬������stm32cubeMX����һ���ļ��е�.ioc�ļ��������������裬�����µ�����ȵȡ�
	//��������֮����generate code�������ļ��Ĵ���ᱻ���£�����ֻ��Ҫ�޸�main�ļ��Ĵ��롣
	//�Լ�д�Ĵ�����Ҫд�ں��������ע�� */ user code begin �� user code end ֮�䣬��Ȼÿ��generate code ֮��ᱻ�����


	//ʹ��PWM����WS2812����Ҫ����ws2812ÿ���Ƶ�rgb��Ϣ�������ƣ���WS2812ͨ���ߵ�ƽ����ʱ��ĳ��� �������߼� 0 ���߼� 1��
	//�ߵ�ƽ0.819΢��͵�ƽ0.431΢�����ʶ����߼�1�� �ߵ�ƽ0.403΢�룬�͵�ƽ0.847΢����Ա�ʶ��Ϊ�߼�0. ����趨pwm����Ϊ1.25΢�롣
	//�趨Ԥ��Ƶϵ��89������߼�1��Ҫռ�ձ�Ϊ59/90�� �߼�0��Ҫռ�ձ�Ϊ29/90���Դ˾����߼�0��1ʱ����ʱ���ıȽϼĴ�������ֵ��Ҫ���ĳ�29��59
	//ͨ��DMA�� RGB_buffur �������Ƶ� ��ʱ���ıȽϼĴ����������Զ������Ӧ�Ĳ��Σ�����Ҫ CPU һֱæ���л���ƽ��
	
	//PA9PA10���Ӵ���ͨ��usbת�������ӵ����ԣ�ͨ�ŵĲ�����Ϊ115200bps
	
	//PA11PA12���ӵ�CAN���շ�����������Ϊ125kbps�� CAN���շ�����Ҫ���ӵ�+5v�ĵ�Դ��������stlink��һ��5v���߳����ṩ
	
	//ͨ��PWM��ÿ���Ƶ�RGB��Ϣ����ws2812. ÿ���Ƶ���Ϣ��Ҫ24λ��RGB���������� ÿ���ƻ��ǰ24λ���������ߣ�Ȼ���ʣ�µ����ݴ�����һ���ƣ���ˣ�ֻ��Ҫ����ÿ���Ƶ�˳�򣬽�RGB��Ϣд��һ�����顣
	//60���Ƶ���Ϣ��СΪ60*24
	
	//
	
	//ws2812_set_RGB �趨��������ĳ��index�ĵƵ�rgb��Ϣ ��ws2812_send_color��stm32���ָ����pwm���ƿ�ʼ����
	//���ֵƵ�Ч���������������������ټ���delay��������ʵ�֡�
	
	//ͨ��cantest��stm32���Ͳ���ѡ��ģʽ�Ͳ����� ��������1000kbps
	//���磺00 10           ѡ��ģʽ0 ������ ������������30ms*16 = 480 ms
	//      01 32 10 A0 FF  ѡ��ģʽ1 ����� ÿ50msǰ��һ���� RGBֵ10 A0 FF
	//      02 32           ѡ��ģʽ2 ���� ÿ50msǰ��һ����
	//      03 32 10 A0 FF  ѡ��ģʽ3 ��˸ 50ms��һ�� RGBֵΪ10 A0 FF
	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONE_PULSE        (59)     // ��ʾ�����߼�1ʱ�ĸߵ�ƽ����ʱ��Լ0.819΢��                   
#define ZERO_PULSE       (29)    // ��ʾ�����߼�0ʱ�ĸߵ�ƽ����ʱ�䣬Լ0.403΢��        
#define RESET_PULSE      (80)    // ��ʾ��λ�źţ��͵�ƽ���ĳ���ʱ��                       
#define LED_NUMS         (60)    //�Ƶ�������60��WS2812                      
#define LED_DATA_LEN     (24)    //ÿ������Ҫ24λ���ݣ�8λG + 8λR + 8λB                      
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN) // �ܵ����ݳ���

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t static RGB_buffur[ WS2812_DATA_LEN+RESET_PULSE] = { 0 };
//����һ�����飬������������ƴ�Ҫ���͵� PWM �������С�
//ÿ��Ԫ�ض�Ӧһ�� PWM ���ڵ�ռ�ձȣ����ȽϼĴ�����ֵ����
//ǰ��Ԥ���� RESET_PULSE �� 0�����������͵�ƽ��λ�źš�

volatile uint8_t rxByte; //û��
volatile uint8_t RxData[8]; //���յ�����Ϣ�洢��RxData

volatile uint8_t rxFlag = 0;	// ���յ���Ϣ��flag�����ڴ�ӡһ�� ��ʾĳ��ģʽ�Ѽ������Ϣ
volatile uint8_t StartFlag = 0; //û��

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//���еƵ�RGB��Ϣ��Ҫ����һ���������������趨��index���Ƶ���Ϣ
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t index)
{
		if (index >= LED_NUMS) return; //��ֹ���ʵ������ƴ����ȵĵơ�
	
    uint16_t* p = (RGB_buffur+RESET_PULSE) + (index * LED_DATA_LEN); 
	  //������� index ���Ƶ������� RGB_buffur �����ʼλ�á�
    //ÿ���� 24 λ��24��PWM���壩������ƫ������ index * 24��
		//�ټ��Ͽ�ͷ�� RESET_PULSE �հ����� 
	
    for (uint16_t i = 0;i < 8;i++) //ѭ�� 8 �Σ��ֱ��� G��R��B �� 8 λ����
    {
  
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
		
      //�����ǹؼ��� (G << i) �� G ���� i λ���ò�ͬ�� bit �Ƶ����λ�� 
			//& 0x80 �жϵ�ǰ���λ�Ƿ�Ϊ 1�� ����� 1���ʹ� ONE_PULSE������� ZERO_PULSE��
			// �����Ͱ�һ���ֽڵ� 8 λ������ת���� 8 �����塣
      //ע�⣺��Ϊ WS2812 Э��Ҫ�� ���λ�ȷ��ͣ�����������ͨ�������� &0x80 ��ʵ��
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE; //ͬ�������ɫ�� 8 λ��
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE; //ͬ��������ɫ�� 8 λ��
			
			

    }
}


//һ�����趨���еƵ�RGB��Ϣ����ֻ��ͬһ����ɫ��
void ws2812_set_All_RGB(uint8_t R, uint8_t G, uint8_t B)
{
for (int i = 0 ; i<LED_NUMS;i++ ){
		
						ws2812_set_RGB(R, G, B, i);
						
		}
}

//�� DMA �� RGB_buffur �������Ƶ� ��ʱ���ıȽϼĴ����������Զ������Ӧ�Ĳ��Σ�
//����Ҫ CPU һֱæ���л���ƽ��
void ws2812_send_color()
{
 
    HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(WS2812_DATA_LEN+RESET_PULSE));      

}
//��ȫ��
void ws2812_clear()
{
    ws2812_set_All_RGB(0,0,0);
		ws2812_send_color();
}
//����t������һ����Χ���ں�����Ч������Ҫ�õ�
uint8_t lerp(uint8_t from, uint8_t to, float t) {
	t = (t < 0) ? 0 : (t > 1) ? 1 : t;

  return (uint8_t)(from + (to - from) * t);
}

//��˸��Ч��
void ws2812_twinkle(int blink_time, uint8_t R, uint8_t G, uint8_t B)
{
    ws2812_set_All_RGB(0,0,0);
    for (int i = 0; i < 5; i++) {
        int index = rand() % LED_NUMS;
        ws2812_set_RGB(R, G, B, index); 
    }

    ws2812_send_color();
    HAL_Delay(blink_time);
}
//���ǵ�
void ws2812_meteor(int speed)
{
    static int pos = 0;  // ��ǰ������ͷ����λ��
    static uint8_t trail[LED_NUMS] = {0}; // ÿ���Ƶ��������飬�൱�ڹ켣����

    for (int i = 0; i < LED_NUMS; i++) {
        if (trail[i] > 30)
            trail[i] -= 30; // ÿ��ѭ��������˥�� 30
        else
            trail[i] = 0;  // ������ֱ�ӹ��㣬���⸺��

        ws2812_set_RGB(trail[i], trail[i], trail[i],i);  // ���õ� i ���Ƶ���ɫ���Ҷȼ�����ɫ��
    }


    trail[pos] = 255; // �ѵ�ǰ����ͷ����Ϊ����

    ws2812_send_color(); // �ѻ���������ɫ���ݷ����ƴ�

    pos = (pos + 1) % LED_NUMS; /// ����ͷ��ǰ�ƶ���ѭ���ص���ͷ
    HAL_Delay(speed); // �����ƶ��ٶ�
}

//�����Ч��
void ws2812_running_light(uint8_t delaytime, uint8_t R, uint8_t G, uint8_t B)
{
    static uint16_t current_index = 0;

   
		ws2812_set_All_RGB(0,0,0);

    ws2812_set_RGB( R, G, B,current_index); 

    ws2812_send_color();

    current_index++;
    if (current_index >= LED_NUMS) current_index = 0;

		HAL_Delay(delaytime);
}
//������Ч��
void ws2812_breath_light(uint8_t breathTime)
{	 	
	const uint8_t baseColor[3]    = {  0,   0,  50};  // ��ʼ��ɫ
	const uint8_t midColor[3]     = { 80,   0,  80};  // �м���ɫ
	const uint8_t peakColor[3]    = {255, 255, 255};  //  ������ɫ  
 
	static uint16_t count = 0;

	
  float elapsed = (count) % breathTime;
	
			
		
  float angle = (elapsed / (float)breathTime) * 2* 3.14159265357;
	

  float brightnessRatio = 0.5 * (1 - cos(angle));

	// �����Ǻ���������ɫ�仯���������ۣ������Ա仯���ϣ�
  float colorPhase = brightnessRatio * 2.0;
  uint8_t r, g, b;

  if (colorPhase <= 1.0) {
   
    r = lerp(baseColor[0], midColor[0], colorPhase);
    g = lerp(baseColor[1], midColor[1], colorPhase);
    b = lerp(baseColor[2], midColor[2], colorPhase);
  } else {
  
    float t = colorPhase - 1.0;
    r = lerp(midColor[0], peakColor[0], t);
    g = lerp(midColor[1], peakColor[1], t);
    b = lerp(midColor[2], peakColor[2], t);
  }
    
		count ++;
		
	
		if (count>=breathTime){
		
				count = 0;
		}
		HAL_Delay(30);
		ws2812_set_All_RGB(r,g,b);
		ws2812_send_color();
	
    
}



void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_1);
}





			
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// �������������printf�������Դ�ӡ������
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
//can���ͺ���
void Can_senddata(uint8_t *buf, int len)
{
		CAN_TxHeaderTypeDef txHeader;
		uint32_t MailBoxID = 0;
		if (len>8)
			len=8;
		
		
		txHeader.DLC = len;
		txHeader.ExtId = 0x12345678;
		txHeader.IDE = CAN_ID_EXT;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.TransmitGlobalTime = DISABLE;
		
			
		HAL_CAN_AddTxMessage(&hcan,&txHeader,buf,&MailBoxID);
		

}
//can����������
void Filter_Init()
{
	CAN_FilterTypeDef sFilterCfg;
	sFilterCfg.FilterActivation = CAN_FILTER_ENABLE;
	sFilterCfg.FilterBank = 0;
	sFilterCfg.FilterFIFOAssignment = CAN_FilterFIFO0;
	sFilterCfg.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterCfg.FilterScale = CAN_FILTERSCALE_32BIT;
	
	
	sFilterCfg.FilterMaskIdHigh = 0x0000;
	sFilterCfg.FilterMaskIdLow = 0x0000;
	sFilterCfg.FilterIdHigh = 0x0000;
	sFilterCfg.FilterIdLow = 0x0000;
	
	HAL_CAN_ConfigFilter(&hcan,&sFilterCfg);

}
// can�����ж�
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{		
		CAN_RxHeaderTypeDef RxHeader;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        printf("CAN Received: ID=0x%03X Data=", RxHeader.StdId);
        for (uint8_t i = 0; i < RxHeader.DLC; i++)
        {
            printf("%d ", RxData[i]);
        }
				
				uint8_t mode = RxData[0];
				printf("mode=%02X\r\n", mode);
        printf("\r\n");
    }
		rxFlag = 1;
		
		
		
		
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Filter_Init(); //��������ʼ��
	HAL_CAN_Start(&hcan); //����CAN����
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING); //���������ж�

//	Can_senddata("hello",5);
	HAL_Delay(1000);
  while (1)
  {		
		
			uint8_t mode = RxData[0]; //���յ��ĵ�һ�����־���ģʽ�������������Ϊ��ͬ��Ч�Ĳ���
			
//			printf("mode=%d\r\n",mode);
		switch(mode){
			case 0x00:
			{			if (rxFlag == 1){
					printf("Case 0 triggered!\r\n");
				rxFlag = 0; //����ֵ��ӡһ��case triggered��
								}
						int newBreath = RxData[1];
						if (newBreath > 10 && newBreath < 500){ 
            int breathtime = newBreath;
						ws2812_breath_light(breathtime);
						}
			
			}
			break;
			
			case 0x01:
			{			
						if (rxFlag == 1){
					printf("Case 1 triggered!\r\n");
							rxFlag = 0;
						}
						int delaytime = RxData[1];
						uint8_t R = RxData[2];
						uint8_t G = RxData[3];
						uint8_t B = RxData[4];
						ws2812_running_light(delaytime, R, G, B);
			
			}
			break;
			
			case 0x02:
			{			
						if (rxFlag == 1){
					printf("Case 2 triggered!\r\n");
							rxFlag = 0;
			}
						int delaytime = RxData[1];
						ws2812_meteor(delaytime);
				
			
			}
			break;
			
			case 0x03:
			{	
						if (rxFlag == 1){
					printf("Case 3 triggered!\r\n");
							rxFlag = 0;
			}
						int delaytime = RxData[1];
						uint8_t R = RxData[2];
						uint8_t G = RxData[3];
						uint8_t B = RxData[4];
						ws2812_twinkle(delaytime,R,G,B);
			
			}
			break;
			
			case 0x04:
			{			
						printf("Case 4 triggered!\r\n");
				rxFlag = 0;
						ws2812_clear();
						ws2812_send_color();
				
			
			}
			break;
			
			case 0x05:
			{
						ws2812_clear();
						ws2812_send_color();
				
			
			}
			break;
			
			default:
            printf("No matching case.\r\n");
            break;
			
			
		}
		
	}
					
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
