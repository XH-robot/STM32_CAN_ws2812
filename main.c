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
  //外设的配置，初始化，启动等代码通过stm32cubemx软件生成。 如果需要开启新的外设，可以用stm32cubeMX打开上一级文件夹的.ioc文件，重新配置外设，启动新的外设等等。
	//配置完了之后点击generate code，各个文件的代码会被更新，我们只需要修改main文件的代码。
	//自己写的代码需要写在后面代码中注释 */ user code begin 和 user code end 之间，不然每次generate code 之后会被清除。


	//使用PWM控制WS2812，需要告诉ws2812每个灯的rgb信息（二进制）。WS2812通过高电平持续时间的长短 来区分逻辑 0 和逻辑 1，
	//高电平0.819微妙，低电平0.431微秒可以识别出逻辑1， 高电平0.403微秒，低电平0.847微秒可以被识别为逻辑0. 因此设定pwm周期为1.25微秒。
	//设定预分频系数89，因此逻辑1需要占空比为59/90， 逻辑0需要占空比为29/90。以此决定逻辑0和1时，定时器的比较寄存器的数值需要被改成29和59
	//通过DMA把 RGB_buffur 的数据推到 定时器的比较寄存器，就能自动输出对应的波形，不需要 CPU 一直忙着切换电平。
	
	//PA9PA10连接串口通过usb转串口连接到电脑，通信的波特率为115200bps
	
	//PA11PA12连接到CAN的收发器，波特率为125kbps。 CAN的收发器需要连接到+5v的电源，可以由stlink接一条5v的线出来提供
	
	//通过PWM将每个灯的RGB信息告诉ws2812. 每个灯的信息需要24位的RGB二进制数。 每个灯会把前24位的数据拿走，然后把剩下的数据传到下一个灯，因此，只需要按照每个灯的顺序，将RGB信息写到一个数组。
	//60个灯的信息大小为60*24
	
	//
	
	//ws2812_set_RGB 设定数组里面某个index的灯的rgb信息 ，ws2812_send_color让stm32输出指定的pwm，灯开始亮。
	//各种灯的效果基本依靠这两个函数再加上delay函数等来实现。
	
	//通过cantest向stm32发送参数选择模式和参数， 波特率是1000kbps
	//锟斤拷锟界：00 10           选择模式0 呼吸灯 呼吸的周期是30ms*16 = 480 ms
	//      01 32 10 A0 FF  选择模式1 跑马灯 每50ms前进一个灯 RGB值10 A0 FF
	//      02 32           选择模式2 流星 每50ms前进一个灯
	//      03 32 10 A0 FF  选择模式3 闪烁 50ms闪一次 RGB值为10 A0 FF
	
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
#define ONE_PULSE        (59)     // 表示发送逻辑1时的高电平持续时间约0.819微秒                   
#define ZERO_PULSE       (29)    // 表示发送逻辑0时的高电平持续时间，约0.403微秒        
#define RESET_PULSE      (80)    // 表示复位信号（低电平）的持续时间                       
#define LED_NUMS         (60)    //灯的数量，60颗WS2812                      
#define LED_DATA_LEN     (24)    //每个灯需要24位数据（8位G + 8位R + 8位B                      
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN) // 总的数据长度

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t static RGB_buffur[ WS2812_DATA_LEN+RESET_PULSE] = { 0 };
//这是一个数组，用来存放整条灯带要发送的 PWM 脉冲序列。
//每个元素对应一个 PWM 周期的占空比（即比较寄存器的值）。
//前面预留了 RESET_PULSE 个 0，用来产生低电平复位信号。

volatile uint8_t rxByte; //没用
volatile uint8_t RxData[8]; //接收到的信息存储在RxData

volatile uint8_t rxFlag = 0;	// 接收到信息的flag，用于打印一次 提示某个模式已激活的信息
volatile uint8_t StartFlag = 0; //没用

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//所有灯的RGB信息需要存在一个数组里，这个函数设定第index个灯的信息
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t index)
{
		if (index >= LED_NUMS) return; //防止访问到超过灯带长度的灯。
	
    uint16_t* p = (RGB_buffur+RESET_PULSE) + (index * LED_DATA_LEN); 
	  //计算出第 index 个灯的数据在 RGB_buffur 里的起始位置。
    //每个灯 24 位（24个PWM脉冲），所以偏移量是 index * 24。
		//再加上开头的 RESET_PULSE 空白区。 
	
    for (uint16_t i = 0;i < 8;i++) //循环 8 次，分别处理 G、R、B 的 8 位数据
    {
  
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
		
      //这里是关键： (G << i) 把 G 左移 i 位，让不同的 bit 移到最高位。 
			//& 0x80 判断当前最高位是否为 1。 如果是 1，就存 ONE_PULSE，否则存 ZERO_PULSE。
			// 这样就把一个字节的 8 位，依次转换成 8 个脉冲。
      //注意：因为 WS2812 协议要求 最高位先发送，所以这里是通过左移再 &0x80 来实现
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE; //同理，处理红色的 8 位。
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE; //同理，处理蓝色的 8 位。
			
			

    }
}


//一次性设定所有灯的RGB信息，但只能同一个颜色。
void ws2812_set_All_RGB(uint8_t R, uint8_t G, uint8_t B)
{
for (int i = 0 ; i<LED_NUMS;i++ ){
		
						ws2812_set_RGB(R, G, B, i);
						
		}
}

//用 DMA 把 RGB_buffur 的数据推到 定时器的比较寄存器，就能自动输出对应的波形，
//不需要 CPU 一直忙着切换电平。
void ws2812_send_color()
{
 
    HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_1,(uint32_t *)RGB_buffur,(WS2812_DATA_LEN+RESET_PULSE));      

}
//灯全灭
void ws2812_clear()
{
    ws2812_set_All_RGB(0,0,0);
		ws2812_send_color();
}
//限制t不超过一定范围，在呼吸灯效果中需要用到
uint8_t lerp(uint8_t from, uint8_t to, float t) {
	t = (t < 0) ? 0 : (t > 1) ? 1 : t;

  return (uint8_t)(from + (to - from) * t);
}

//闪烁灯效果
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
//流星灯
void ws2812_meteor(int speed)
{
    static int pos = 0;  // 当前“流星头”的位置
    static uint8_t trail[LED_NUMS] = {0}; // 每个灯的亮度数组，相当于轨迹缓存

    for (int i = 0; i < LED_NUMS; i++) {
        if (trail[i] > 30)
            trail[i] -= 30; // 每次循环让亮度衰减 30
        else
            trail[i] = 0;  // 不够就直接归零，避免负数

        ws2812_set_RGB(trail[i], trail[i], trail[i],i);  // 设置第 i 个灯的颜色，灰度级（白色）
    }


    trail[pos] = 255; // 把当前流星头设置为最亮

    ws2812_send_color(); // 把缓冲区的颜色数据发给灯带

    pos = (pos + 1) % LED_NUMS; /// 流星头往前移动，循环回到开头
    HAL_Delay(speed); // 控制移动速度
}

//跑马灯效果
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
//呼吸灯效果
void ws2812_breath_light(uint8_t breathTime)
{	 	
	const uint8_t baseColor[3]    = {  0,   0,  50};  // 初始颜色
	const uint8_t midColor[3]     = { 80,   0,  80};  // 中间颜色
	const uint8_t peakColor[3]    = {255, 255, 255};  //  最终颜色  
 
	static uint16_t count = 0;

	
  float elapsed = (count) % breathTime;
	
			
		
  float angle = (elapsed / (float)breathTime) * 2* 3.14159265357;
	

  float brightnessRatio = 0.5 * (1 - cos(angle));

	// 用三角函数，让颜色变化更符合人眼（比线性变化符合）
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
// 定义这个函数让printf函数可以打印到串口
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
//can发送函数
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
//can过滤器设置
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
// can接收中断
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
	Filter_Init(); //过滤器初始化
	HAL_CAN_Start(&hcan); //启动CAN外设
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING); //启动接收中断

//	Can_senddata("hello",5);
	HAL_Delay(1000);
  while (1)
  {		
		
			uint8_t mode = RxData[0]; //接收到的第一个数字决定模式，后面的数字作为不同灯效的参数
			
//			printf("mode=%d\r\n",mode);
		switch(mode){
			case 0x00:
			{			if (rxFlag == 1){
					printf("Case 0 triggered!\r\n");
				rxFlag = 0; //用于值打印一次case triggered！
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
