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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "socket.h"
#include "wizchip_conf.h"
#include "w5x00_network.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FWUP_SOCKET 0
#define FWUP_PORT   5000

#define DATA_BUF_SIZE 2048
#define APPLICATION_ADDR 0x08020000 //FLASH_SECTOR_5
#define DATA_END_TIME 3000 //3 sec
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
wiz_NetInfo net_info =
  {
      .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
      .ip = {192, 168, 2, 100},                     // IP address
      .sn = {255, 255, 255, 0},                    // Subnet Mask
      .gw = {192, 168, 2, 1},                     // Gateway
      .dns = {8, 8, 8, 8},                         // DNS server
#ifdef APP_DHCP
      .dhcp = NETINFO_DHCP                         // Dynamic IP
#else
      .dhcp = NETINFO_STATIC                       // Static IP
#endif
  };
uint8_t g_eth_buf[DATA_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FMC_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFF);

  return ch;
}

void Flash_Erase_Sector(uint32_t sector);
void Flash_Write_Data(uint32_t writeAddr, uint8_t *data, uint32_t dataSize);
void DeinitializePeripherals(void);
void JumpToApplication(uint32_t appAddr);
  
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
  uint16_t size;
  uint32_t writeAddr = APPLICATION_ADDR, start_time = 0, end_time = 0, total_size = 0;
  int ret;
  
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
  MX_USART3_UART_Init();
  MX_FMC_Init();
  /* USER CODE BEGIN 2 */

  wizchip_network_initialize(&net_info);
  wizchip_network_information(&net_info);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch(getSn_SR(FWUP_SOCKET))
    {
      case SOCK_ESTABLISHED :
        if(getSn_IR(FWUP_SOCKET) & Sn_IR_CON)
        {
          uint8_t destip[4];
          uint16_t destport;
          
          getSn_DIPR(FWUP_SOCKET, destip);
          destport = getSn_DPORT(FWUP_SOCKET);
          printf("%d:Connected - %d.%d.%d.%d : %d\r\n",FWUP_SOCKET, destip[0], destip[1], destip[2], destip[3], destport);
          Flash_Erase_Sector(FLASH_SECTOR_5);
          printf("Flash_Erase_Sector(FLASH_SECTOR_5) complete\r\n");
          setSn_IR(FWUP_SOCKET,Sn_IR_CON);
        }
        if((size = getSn_RX_RSR(FWUP_SOCKET)) > 0)
        {
          if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
          memset(g_eth_buf, 0xFF, DATA_BUF_SIZE);
          ret = recv(FWUP_SOCKET, g_eth_buf, size);
          if(ret <= 0) break;
          size = (uint16_t) ret;
          printf("recv size = %d\r\n", size);
          
          Flash_Write_Data(writeAddr, g_eth_buf, size);
          writeAddr += size;
          total_size += size;
          start_time = HAL_GetTick();
          if (!start_time) start_time = 1;
        }
        
        if (start_time) {
          end_time = HAL_GetTick();
          if ((end_time - start_time) > DATA_END_TIME)
          {
            disconnect(FWUP_SOCKET);
            HAL_Delay(10);
            close(FWUP_SOCKET);
            printf("total_size = %d\r\n", total_size);
            printf("JumpToApplication to 0x%08X\r\n", APPLICATION_ADDR);
            HAL_Delay(1000);
            DeinitializePeripherals();
            JumpToApplication(APPLICATION_ADDR);
            while(1);
          }
        }
        break;
      
      case SOCK_CLOSE_WAIT :
        if((ret = disconnect(FWUP_SOCKET)) != SOCK_OK) break;
        printf("%d:Socket Closed\r\n", FWUP_SOCKET);
        break;
      
      case SOCK_INIT :
        if( (ret = listen(FWUP_SOCKET)) != SOCK_OK) break;
        printf("%d:Listen, TCP server loopback, port [%d]\r\n", FWUP_SOCKET, FWUP_PORT);
        break;
      
      case SOCK_CLOSED:
        if((ret = socket(FWUP_SOCKET, Sn_MR_TCP, FWUP_PORT, 0x00)) != FWUP_SOCKET) break;
          printf("%d:Socket opened\r\n",FWUP_SOCKET);
        break;
      
      default:
        break;
     }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  // overwrite
  Timing.AddressSetupTime = 1;
  Timing.AddressHoldTime = 1;
  Timing.DataSetupTime = 6;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 2;
  Timing.DataLatency = 2;
  Timing.AccessMode = FMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5x00_RST_GPIO_Port, W5x00_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : W5x00_BRDY0_Pin W5x00_BRDY1_Pin W5x00_BRDY2_Pin W5x00_BRDY3_Pin */
  GPIO_InitStruct.Pin = W5x00_BRDY0_Pin|W5x00_BRDY1_Pin|W5x00_BRDY2_Pin|W5x00_BRDY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : W5x00_INT_Pin */
  GPIO_InitStruct.Pin = W5x00_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(W5x00_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5x00_RST_Pin */
  GPIO_InitStruct.Pin = W5x00_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5x00_RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Flash_Erase_Sector(uint32_t sector)
{
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t SectorError;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInitStruct.Sector = sector;
    eraseInitStruct.NbSectors = 1;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&eraseInitStruct, &SectorError) != HAL_OK)
      printf("HAL_FLASHEx_Erase Error\r\n");
    HAL_FLASH_Lock();
}

void Flash_Write_Data(uint32_t writeAddr, uint8_t *data, uint32_t dataSize)
{
  HAL_FLASH_Unlock();
  for (uint32_t i = 0; i < dataSize; i ++)
  {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, writeAddr + i, *(data+i) ) != HAL_OK)
      {
          printf("HAL_FLASH_Program Error addr = 0x%08X\r\n", writeAddr);
          break;
      }
  }
  HAL_FLASH_Lock();
}

void DeinitializePeripherals(void)
{
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    __HAL_RCC_GPIOA_FORCE_RESET();
    __HAL_RCC_GPIOB_FORCE_RESET();
    __HAL_RCC_GPIOC_FORCE_RESET();
    __HAL_RCC_GPIOD_FORCE_RESET();
    __HAL_RCC_GPIOE_FORCE_RESET();
    __HAL_RCC_GPIOF_FORCE_RESET();
    __HAL_RCC_GPIOG_FORCE_RESET();
    __HAL_RCC_GPIOH_FORCE_RESET();
    __HAL_RCC_GPIOI_FORCE_RESET();
    __HAL_RCC_GPIOA_RELEASE_RESET();
    __HAL_RCC_GPIOB_RELEASE_RESET();
    __HAL_RCC_GPIOC_RELEASE_RESET();
    __HAL_RCC_GPIOD_RELEASE_RESET();
    __HAL_RCC_GPIOE_RELEASE_RESET();
    __HAL_RCC_GPIOF_RELEASE_RESET();
    __HAL_RCC_GPIOG_RELEASE_RESET();
    __HAL_RCC_GPIOH_RELEASE_RESET();
    __HAL_RCC_GPIOI_RELEASE_RESET();

    HAL_DeInit();
}

void JumpToApplication(uint32_t appAddr)
{
    uint32_t appResetHandlerAddr;
    void (*AppJump)(void);

    SCB->VTOR = appAddr;
    __set_MSP((*(uint32_t *)appAddr));
    appResetHandlerAddr = (*((uint32_t *)((appAddr) + 4U)));
    AppJump = (void (*)(void))appResetHandlerAddr;

    AppJump();
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
