/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Simple Stepper Motor Test - Relative Move One Floor
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Modules/Stepper/stepper_motor.h"
#include "../Modules/oled/oled.h"
#include "../Modules/CAN/can.h"
#include "../Modules/RS485/rs485.h"
#include "../Modules/RS485/rs485_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS_PER_FLOOR  10800  // 一层楼的标准步数
#define STAY_TIME        2000   // 每层停留时间(ms)

/* RS485通信帧类型 - 从rs485_protocol.h */
#define CMD_PHOTO_SENSOR    0x10
#define CMD_DOOR_OPEN       0x20
#define CMD_DOOR_CLOSE      0x21
#define CMD_DOOR_STATUS     0x22
#define CMD_FLOOR_CALL      0x30
#define CMD_DIRECTION_SET   0x40
#define CMD_STATUS_REQUEST  0x50
#define CMD_STATUS_RESPONSE 0x51
#define CMD_ERROR           0xF0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
StepperMotor_t stepper;  // 步进电机对象

/* 位置控制 */
int32_t encoder_offset = 0;      // 编码器初始读数
int32_t floor_positions[4] = {0}; // 各楼层位置
uint8_t current_floor = 1;       // 当前楼层
uint8_t target_floor = 1;        // 目标楼层
bool is_moving = false;          // 是否正在移动

/* 固定运行序列 */
const uint8_t floor_sequence[] = {1, 2, 3, 2, 1};  // 运行序列
uint8_t sequence_index = 0;      // 当前序列索引
bool waiting_at_floor = false;    // 是否在楼层等待
uint32_t arrive_time = 0;        // 到达时间
uint32_t move_start_time = 0;    // 开始移动时间

/* 调试信息 */
char debug_msg[32] = "System OK";  // 调试消息

#define MAX_FLOORS 3
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Display_EncoderValue(void);
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
  MX_CAN1_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* 初始化OLED */
  Display_Init();
  Display_Clear();
  Display_Text(0, 0, "Elevator Test");
  Display_Update();
  
  /* 初始化RS485通信 */
  printf("\r\n=== Simple Elevator Test ===\r\n");
  printf("Sequence: 1->2->3->2->1\r\n\r\n");
  rs485_init();
  
  /* 初始化步进电机 */
  printf("Initializing stepper motor...\r\n");
  StepperMotor_Init(&stepper);
  HAL_Delay(100);
  
  StepperMotor_Enable(&stepper);
  HAL_Delay(100);
  
  /* 读取编码器初始值 - 多次尝试 */
  printf("Reading encoder position...\r\n");
  for (int retry = 0; retry < 10; retry++) {
    StepperMotor_Update(&stepper);
    HAL_Delay(100);  // 等待通信
    if (stepper.current_position != 0 || retry > 5) {
      break;  // 读到非零值或尝试多次后退出
    }
    printf("  Retry %d: position = %ld\r\n", retry + 1, stepper.current_position);
  }
  encoder_offset = stepper.current_position;
  printf("Encoder reading complete: %ld\r\n", encoder_offset);
  
  /* 计算各楼层位置 */
  floor_positions[1] = encoder_offset;                       // 1楼
  floor_positions[2] = encoder_offset + STEPS_PER_FLOOR;     // 2楼
  floor_positions[3] = encoder_offset + 2 * STEPS_PER_FLOOR; // 3楼
  
  printf("Encoder initial value: %ld\r\n", encoder_offset);
  printf("Floor positions:\r\n");
  for (int i = 1; i <= 3; i++) {
    printf("  Floor %d: %ld\r\n", i, floor_positions[i]);
  }
  
  /* 显示初始状态 */
  Display_EncoderValue();
  
  /* 启动序列 - 2秒后开始 */
  printf("\r\nStarting sequence in 2 seconds...\r\n\r\n");
  HAL_Delay(2000);
  
  /* 开始第一个移动 */
  sequence_index = 1;
  target_floor = floor_sequence[sequence_index];
  waiting_at_floor = true;  // 先进入等待状态，触发第一次移动
  arrive_time = HAL_GetTick() - STAY_TIME;  // 立即触发
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_display_time = 0;
  uint32_t last_photo_floor = 0;  // 记录最后光电检测到的楼层
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 更新步进电机状态 */
    StepperMotor_Update(&stepper);
    
    /* 处理RS485接收 - 主要是光电传感器事件 */
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
      if (rx_buffer[0] == CMD_PHOTO_SENSOR && rx_len >= 2) {
        uint8_t floor = rx_buffer[1];
        last_photo_floor = floor;  // 记录光电检测到的楼层
        
        printf("Photo sensor: Floor %d detected\r\n", floor);
        
        /* 简单校正：更新楼层位置 */
        floor_positions[floor] = stepper.current_position;
        
        /* 如果是目标楼层，直接认为到达 */
        if (floor == target_floor && is_moving) {
          printf("*** Arrived at target floor %d by photo sensor ***\r\n", floor);
          
          /* 停止电机 */
          StepperMotor_Stop(&stepper);
          
          /* 更新状态 */
          current_floor = floor;
          is_moving = false;
          waiting_at_floor = true;
          arrive_time = HAL_GetTick();
          
          /* 通知从控停止 */
          uint8_t tx_buffer[4] = {CMD_DIRECTION_SET, 0, 0, 0};
          rs485_send_packet_dma(tx_buffer, 4);
          
          sprintf(debug_msg, "At F%d", current_floor);
        }
      }
    }
    
    /* 简化的序列控制 - 只检查等待和移动 */
    if (waiting_at_floor) {
      /* 等待2秒 */
      if (HAL_GetTick() - arrive_time >= STAY_TIME) {
        waiting_at_floor = false;
        sequence_index++;
        
        /* 移动到下一层或重启序列 */
        if (sequence_index < sizeof(floor_sequence)) {
          target_floor = floor_sequence[sequence_index];
          sprintf(debug_msg, "Go F%d", target_floor);
          printf("\r\nMoving to floor %d\r\n", target_floor);
          
          /* 发送方向 */
          uint8_t direction = (target_floor > current_floor) ? 1 : 2;
          uint8_t tx_buffer[4] = {CMD_DIRECTION_SET, direction, 0, 0};
          rs485_send_packet_dma(tx_buffer, 4);
          
          /* 启动电机 */
          StepperMotor_MoveAbsolute(&stepper, floor_positions[target_floor]);
          is_moving = true;
          move_start_time = HAL_GetTick();
        } else {
          /* 序列完成，重启 */
          printf("\r\n*** Sequence complete, restarting ***\r\n\r\n");
          HAL_Delay(2000);
          sequence_index = 1;
          target_floor = floor_sequence[sequence_index];
        }
      }
    }
    
    /* 超时保护：15秒还没到就用最后的光电楼层 */
    if (is_moving && (HAL_GetTick() - move_start_time > 15000)) {
      printf("Timeout! Using last photo floor: %lu\r\n", last_photo_floor);
      
      /* 停止电机 */
      StepperMotor_Stop(&stepper);
      
      /* 使用最后光电检测到的楼层，如果没有就用目标楼层 */
      current_floor = (last_photo_floor > 0) ? last_photo_floor : target_floor;
      is_moving = false;
      waiting_at_floor = true;
      arrive_time = HAL_GetTick();
      
      sprintf(debug_msg, "Timeout F%d", current_floor);
    }
    
    /* 更新OLED显示 */
    if (current_time - last_display_time >= 200) {
      last_display_time = current_time;
      Display_EncoderValue();
    }
    
    HAL_Delay(10);
    
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level for OLED */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins for OLED : PD11 PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  显示状态到OLED
  */
static void Display_EncoderValue(void)
{
  char buffer[32];
  
  Display_Clear();
  
  /* 第1行：楼层信息 */
  sprintf(buffer, "Floor: %d -> %d", current_floor, target_floor);
  Display_Text(0, 0, buffer);
  
  /* 第2行：序列进度 */
  sprintf(buffer, "Seq[%d]: ", sequence_index);
  for(int i = 0; i < sizeof(floor_sequence); i++) {
    char num[4];
    sprintf(num, "%d ", floor_sequence[i]);
    strcat(buffer, num);
  }
  Display_Text(0, 16, buffer);
  
  /* 第3行：状态 */
  if (waiting_at_floor) {
    uint32_t wait_sec = (HAL_GetTick() - arrive_time) / 1000;
    sprintf(buffer, "Wait: %lu/%d sec", wait_sec, STAY_TIME/1000);
  } else if (is_moving) {
    uint32_t move_sec = (HAL_GetTick() - move_start_time) / 1000;
    sprintf(buffer, "Moving: %lu sec", move_sec);
  } else {
    sprintf(buffer, "Ready");
  }
  Display_Text(0, 32, buffer);
  
  /* 第4行：消息 */
  Display_Text(0, 48, debug_msg);
  
  Display_Update();
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