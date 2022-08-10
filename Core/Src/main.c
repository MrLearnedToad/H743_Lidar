/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lidar.h"
#include "fdcan_bsp.h"
#include "math.h"
#include "usart.h"
#include "arm_math.h"
#include "rgb.h"
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

float last_pos_x=0,last_pos_y=0;
uint8_t rxtemp=0;
int rocker[4];
uint8_t turret_buffer[8];
uint8_t dma_buffer[4200]={0};
uint8_t raw_data_buffer[4200]={0};
uint8_t raw_data_flag=0;
uint8_t tof_buffer[10]={0};
short tof_read;
uint8_t Rx_buffer[16]={0};
uint8_t huart3_rxbuffer[16]={0};
uint8_t R1_buf[16]={0};
int global_clock;
int speed_clock;
uint32_t speed_timer=0;
float vx[10],vy[10],ababa;
uint8_t pos_reset=0;
uint8_t error=0;
uint8_t block_color=0;
uint8_t communciation_error_counter=0;
extern uint8_t get_block_flag;
uint8_t last;
//int package_recieve=0,package_valid=0;
//int start_nbr=0;
    
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DMA_recieve(void);
void send_log(uint8_t ID,float data1,float data2,float data3,float data4,UART_HandleTypeDef *uart);
void send_msg(void);
void send_init_msg(UART_HandleTypeDef *uart,uint8_t ID);
extern uint16_t RGB_DEFAULT[2];
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
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_UART8_Init();
  MX_TIM8_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	uint8_t lidar_cmd_buf[9];
	
	last=0xA5;
    HAL_Delay(20);
    FDCAN1_Init(&hfdcan1);
    HAL_Delay(2000);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_Delay(200);
    FDCAN2_Init(&hfdcan2);
    HAL_Delay(20);
	lidar_cmd_buf[0]=0xA5;
	lidar_cmd_buf[1]=0x25;
	HAL_UART_Transmit(&huart3,lidar_cmd_buf,2,10);
	HAL_Delay(1400);
    
    MX_USART3_UART_Init();
    
    RGB_DEFAULT[0]=0;
    RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,0.2f);
    RGB_Init(&htim8,TIM_CHANNEL_3);
	
	lidar_cmd_buf[0]=0xA5;
	lidar_cmd_buf[1]=0x82;
	lidar_cmd_buf[2]=0x05;
	lidar_cmd_buf[3]=0;
	lidar_cmd_buf[4]=0;
	lidar_cmd_buf[5]=0;
	lidar_cmd_buf[6]=0;
	lidar_cmd_buf[7]=0;
	lidar_cmd_buf[8]=0x22;
	HAL_UART_Transmit(&huart3,lidar_cmd_buf,9,10);
	HAL_UART_Receive_IT(&huart3,dma_buffer,7);
	//HAL_UART_Receive_IT(&huart2,R1_buf,16);
	
	
    HAL_TIM_Base_Start_IT(&htim6);
    extern uint8_t block_num;
    

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 48;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void send_msg(void)
{
    uint8_t msg_buffer[30];
    
    HAL_UART_Transmit(&huart8,msg_buffer,16,20);
    //HAL_UART_Transmit(&huart3,msg_buffer,16,200);
    return;
}

void send_init_msg(UART_HandleTypeDef *uart,uint8_t ID)
{
    uint8_t msg_buffer[16];
    msg_buffer[0]='?';
    msg_buffer[1]='!';
    msg_buffer[2]=ID;
//    int ax,ay,az;
//    ax=data1*1000;
//    ay=data2*1000;
//    memcpy(msg_buffer+3,&ax,4);
//    memcpy(msg_buffer+7,&ay,4);
//    az=(int)(gyro.z*1000);
//    memcpy(msg_buffer+11,&az,4);
    msg_buffer[15]='!';
    HAL_UART_Transmit(uart,msg_buffer,16,200);
    return;
}

void send_log(uint8_t ID,float data1,float data2,float data3,float data4,UART_HandleTypeDef *uart)
{
    uint8_t abaaba[18];
    abaaba[0]=0x02;
    abaaba[17]=ID;
    memcpy(abaaba+1,&data1,4);
    memcpy(abaaba+5,&data2,4);
    memcpy(abaaba+9,&data3,4);
    memcpy(abaaba+13,&data4,4);
    HAL_UART_Transmit(uart,abaaba,18,1);
    return;
}

void send_log2(float data1,float data2,float data3,float data4,UART_HandleTypeDef *uart)
{
    uint8_t abaaba[20];
    memcpy(abaaba,&data1,4);
    memcpy(abaaba+4,&data2,4);
    memcpy(abaaba+8,&data3,4);
    memcpy(abaaba+12,&data4,4);
    abaaba[19]=0x7f;
    abaaba[18]=0x80;
    abaaba[17]=0x00;
    abaaba[16]=0x00;
    HAL_UART_Transmit(uart,abaaba,20,1);
    return;
}

void send_log3(float data1,float data2,float data3,uint8_t id,UART_HandleTypeDef *uart)
{
    uint8_t msg_buffer[16];
    int ax,ay,az;
    msg_buffer[0]='?';
    msg_buffer[1]='!';
    msg_buffer[2]=id;
    ax=data1*1000;
    ay=data2*1000;
    memcpy(msg_buffer+3,&ax,4);
    memcpy(msg_buffer+7,&ay,4);
    az=(int)(data3*1000);
    memcpy(msg_buffer+11,&az,4);
    msg_buffer[15]='!';
    
    HAL_UART_Transmit(uart,msg_buffer,16,1);
    return;
}


void tof_recieve()
{
    if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_ORE) != RESET) 
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }
    uint8_t buffer[4]={0};
    int i=0;
    for(;i<10;i++)
    {
        if(tof_buffer[i]==0x59)
        {
            break;
        }
    }
    if(i==9&&tof_buffer[i]!=0x59) return;
    
    for(int k=0;k<4;k++)
    {
        buffer[k]=tof_buffer[i];
        if(k==1&&tof_buffer[i]!=0x59)
            return;
        if(i==9)
            i=0;
        else
            i++;
            
    }
    
    if(*((short*)(buffer+2))<=1000&&*((short*)(buffer+2))>=0)
        tof_read=*((short*)(buffer+2))/0.88f;
    else
        tof_read=0;
    return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(lidar_init_flag==0&&huart==&huart3)
	{
		lidar_init_flag=1;
		HAL_UART_Receive_DMA(&huart3,dma_buffer,4200);
		RGB_DEFAULT[0]=50;
		RGB_Color(&htim8,TIM_CHANNEL_3,RGB_DEFAULT,0.2f);
		return;
	}
	
}

uint8_t cmp_buf[5]={0};

uint8_t ababab(uint8_t *src,uint8_t *dest)
{
	for(int i=0;i<5;i++)
	{
		if(src[i]!=dest[i])
		{
			return 1;
		}
	}
	return 0;
}

void msg_transfer(float angle,float dist,int id)
{
	int intangle=(angle*1000.0f),intdist=(dist*1000.0f);
	uint8_t msg_buf[8];
	memcpy(msg_buf,&intangle,4);
	memcpy(msg_buf+4,&intdist,4);
	FDCAN_SendData(&hfdcan1,msg_buf,0x114+id,8);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else if(htim->Instance == TIM6)	/**/
	{
		if(raw_data_flag==1)
		{
			for(int i=0;i<50;i++)
			{
				data_update(raw_data_buffer+i*84);
				raw_data_flag=0;
			}
		}
		else if(error!=1)
		{
			
			float angle,dist;
			
			Tran_XY_To_Angle_Distance(BOHH[3],&angle,&dist);
			if(angle!=-190)
				msg_transfer(angle,dist,0);
			Tran_XY_To_Angle_Distance(BOHH[2],&angle,&dist);
			if(angle!=-190)
				msg_transfer(angle,dist,1);
			Tran_XY_To_Angle_Distance(BOHH[0],&angle,&dist);
			if(angle!=-190)
				msg_transfer(angle,dist,2);
			Tran_XY_To_Angle_Distance(BOHH[1],&angle,&dist);
			if(angle!=-190)
				msg_transfer(angle,dist,3);
			
		}
	}
	else if(htim->Instance == TIM7)
	{
//		if(__HAL_UART_GET_FLAG(huart3,UART_FLAG_ORE) != RESET) 
//		{
//			__HAL_UART_CLEAR_OREFLAG(huart3);
//			HAL_UART_Receive_IT(&huart3,dma_buffer,5);
//		}
        speed_clock++;
		
		
         if(speed_clock==2)
		 {
			if(ababab(cmp_buf,&dma_buffer[4195]))
			{
				if(dma_buffer[0]>>4!=0xA)
					error=1;
				memcpy(cmp_buf,&dma_buffer[4195],5);
				memcpy(raw_data_buffer,dma_buffer,4200);
				raw_data_flag=1;
			}
            speed_clock=0;
		 }
        speed_timer++;
	}
    else if(htim->Instance == TIM16)
    {
        
    }
  /* USER CODE END Callback 1 */
}

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
