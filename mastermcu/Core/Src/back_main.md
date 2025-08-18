/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Event-driven Elevator Control System with FSM
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
#include "../Modules/Button/button_handler.h"
#include "../Modules/Blackboard/blackboard.h"
#include "../Modules/FSM/elevator_fsm.h"
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
#define STEPS_PER_FLOOR     10800   // 每层步数
#define STOP_THRESHOLD      300     // 停止阈值
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

/* 测试状态结构 - 用于步进电机和光电传感器精准停层测试 */
typedef struct {
    /* 位置信息 */
    int32_t encoder_offset;      // 编码器偏移
    int32_t current_position;    // 当前位置
    int32_t target_position;     // 目标位置
    uint8_t current_floor;       // 当前楼层
    uint8_t target_floor;        // 目标楼层
    
    /* 运动状态 */
    bool is_moving;              // 是否在移动
    
    /* 光电传感器 */
    uint8_t last_photo_floor;    // 最后触发的楼层
    uint32_t photo_trigger_time; // 触发时间
    int32_t calibration_error;   // 校准误差
    
    /* 测试统计 */
    uint32_t stops_count;        // 停层次数
    uint32_t calibration_count;  // 校准次数
    uint8_t test_mode;           // 0=手动, 1=单次, 2=循环
} TestState_t;

static TestState_t test_state = {0};

/* 辅助函数声明 */
void ProcessPhotoSensor(uint8_t floor);
void StartMoveToFloor(uint8_t floor);
int32_t GetTargetPosition(uint8_t floor);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
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
  Display_Text(0, 0, "OLED TEST OK");
  Display_Text(0, 16, "Starting...");
  Display_Update();
  HAL_Delay(1000);  // 显示1秒确认OLED工作
  
  /* 初始化RS485通信 */
  printf("\r\n=== Event-Driven Elevator System ===\r\n");
  printf("With FSM and SCAN scheduling\r\n\r\n");
  rs485_init();
  
  /* 初始化按钮 */
  Button_Init();
  printf("Buttons initialized\r\n");
  
  /* 初始化步进电机 */
  printf("Initializing stepper motor...\r\n");
  StepperMotor_Init(&stepper);
  HAL_Delay(100);
  
  StepperMotor_Enable(&stepper);
  HAL_Delay(100);
  
  /* 读取编码器初始值 */
  printf("Reading encoder position...\r\n");
  for (int retry = 0; retry < 10; retry++) {
    StepperMotor_Update(&stepper);
    HAL_Delay(100);
    if (stepper.current_position != 0 || retry > 5) {
      break;
    }
    printf("  Retry %d: position = %ld\r\n", retry + 1, stepper.current_position);
  }
  
  /* === 步进电机和光电传感器精准停层测试 === */
  /* 初始化测试状态 - 假设在1楼 */
  test_state.encoder_offset = stepper.current_position;
  test_state.current_floor = 1;
  test_state.current_position = stepper.current_position;
  test_state.is_moving = false;
  test_state.test_mode = 0;
  
  printf("System initialized\r\n");
  printf("Encoder offset: %ld\r\n", test_state.encoder_offset);
  printf("Floor positions:\r\n");
  for (int i = 1; i <= 3; i++) {
    int32_t floor_pos = test_state.encoder_offset + (i - 1) * STEPS_PER_FLOOR;
    printf("  Floor %d: %ld\r\n", i, floor_pos);
  }
  
  /* 显示初始状态 */
  printf("\r\nSystem ready. Button test mode\r\n");
  printf("Waiting for button presses...\r\n\r\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_display_time = 0;
  
  printf("\r\n=== STEPPER & PHOTO SENSOR TEST ===\r\n");
  printf("Test Controls:\r\n");
  printf("  PC5 (1UP): Move to Floor 1\r\n");
  printf("  PC1 (2UP): Move to Floor 2\r\n");
  printf("  PC3 (3UP): Move to Floor 3\r\n");
  printf("  PC0 (2DN): Start cycle test\r\n");
  printf("  PC2 (3DN): Emergency stop\r\n");
  printf("\r\n");
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    static uint8_t ext_button_floor = 0;
    static uint8_t ext_button_type = 0;
    static uint32_t ext_button_time = 0;
    static uint8_t cabin_floor = 0;
    static uint32_t cabin_time = 0;
    static uint8_t photo_floor = 0;
    static uint32_t photo_time = 0;
    
    /* === 按钮扫描 - 用于测试控制 === */
    static bool button_pressed[NUM_BUTTONS] = {false};
    for (int i = 0; i < NUM_BUTTONS; i++) {
      Button_t* btn = &buttons[i];
      GPIO_PinState state = HAL_GPIO_ReadPin(btn->port, btn->pin);
      
      if (state == GPIO_PIN_RESET && !button_pressed[i]) {
        button_pressed[i] = true;
        
        /* 根据按钮执行不同测试 */
        if (btn->floor == 1 && btn->type == BUTTON_TYPE_UP) {
          StartMoveToFloor(1);
          test_state.test_mode = 1;
        }
        else if (btn->floor == 2 && btn->type == BUTTON_TYPE_UP) {
          StartMoveToFloor(2);
          test_state.test_mode = 1;
        }
        else if (btn->floor == 3 && btn->type == BUTTON_TYPE_UP) {
          StartMoveToFloor(3);
          test_state.test_mode = 1;
        }
        else if (btn->floor == 2 && btn->type == BUTTON_TYPE_DOWN) {
          /* 开始循环测试 */
          printf("\r\n[TEST] Starting cycle test\r\n");
          test_state.test_mode = 2;
          test_state.stops_count = 0;
          test_state.calibration_count = 0;
          StartMoveToFloor(1);
        }
        else if (btn->floor == 3 && btn->type == BUTTON_TYPE_DOWN) {
          /* 紧急停止 */
          printf("\r\n[STOP] Emergency stop!\r\n");
          StepperMotor_Stop(&stepper);
          test_state.is_moving = false;
          test_state.test_mode = 0;
        }
        
        printf("Button: Floor %d %s\r\n", btn->floor,
               btn->type == BUTTON_TYPE_UP ? "UP" : "DOWN");
      }
      else if (state == GPIO_PIN_SET) {
        button_pressed[i] = false;
      }
    }
    
    /* === RS485接收内呼 === */
    static uint32_t rs485_check_time = 0;
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
      printf("RS485 RX: %d bytes [", rx_len);
      for(int i = 0; i < rx_len; i++) {
        printf("0x%02X ", rx_buffer[i]);
      }
      printf("]\r\n");
      
      /* 处理光电传感器触发 */
      if (rx_buffer[0] == CMD_PHOTO_SENSOR && rx_len >= 2) {
        uint8_t floor = rx_buffer[1];
        ProcessPhotoSensor(floor);
      }
      else if (rx_buffer[0] == CMD_CABIN_CALL && rx_len >= 2) {
        /* 内呼也可以用作测试 */
        uint8_t floor = rx_buffer[1];
        printf("CABIN: Floor %d\r\n", floor);
        StartMoveToFloor(floor);
        test_state.test_mode = 1;
      }
    }
    
    /* 每秒检查RS485状态 */
    if (current_time - rs485_check_time >= 1000) {
      rs485_check_time = current_time;
      // printf("RS485 check at %lu\r\n", current_time/1000);
    }
    
    /* === 更新步进电机状态 === */
    StepperMotor_Update(&stepper);
    test_state.current_position = stepper.current_position;
    
    /* === 检查是否到达目标位置 === */
    if (test_state.is_moving && test_state.target_floor > 0) {
      int32_t error = abs(test_state.current_position - test_state.target_position);
      if (error < STOP_THRESHOLD) {
        /* 位置到达 */
        StepperMotor_Stop(&stepper);
        test_state.is_moving = false;
        test_state.current_floor = test_state.target_floor;
        test_state.stops_count++;
        
        printf("Reached floor %d by position (error=%ld)\r\n", 
               test_state.current_floor, error);
        
        /* 循环测试模式 */
        if (test_state.test_mode == 2) {
          HAL_Delay(2000);  // 停留2秒
          uint8_t next_floor = test_state.current_floor + 1;
          if (next_floor > 3) next_floor = 1;
          StartMoveToFloor(next_floor);
        }
      }
    }
    
    /* === 更新OLED显示（每100ms） === */
    if (current_time - last_display_time >= 100) {
      last_display_time = current_time;
      
      char line1[20], line2[20], line3[20], line4[20];
      
      Display_Clear();
      
      /* 第1行：状态和楼层 */
      if (test_state.is_moving) {
        sprintf(line1, "MOVING F%d->F%d", 
                test_state.current_floor, test_state.target_floor);
      } else if (test_state.test_mode == 2) {
        sprintf(line1, "CYCLE TEST F%d", test_state.current_floor);
      } else {
        sprintf(line1, "IDLE Floor %d", test_state.current_floor);
      }
      Display_Text(0, 0, line1);
      
      /* 第2行：位置信息 */
      sprintf(line2, "Pos:%ld", test_state.current_position);
      Display_Text(0, 16, line2);
      
      /* 第3行：误差或光电信息 */
      if (test_state.is_moving) {
        int32_t error = test_state.target_position - test_state.current_position;
        sprintf(line3, "Err:%ld", error);
      } else if (current_time - test_state.photo_trigger_time < 3000) {
        sprintf(line3, "Ph%d E:%ld", 
                test_state.last_photo_floor, test_state.calibration_error);
      } else {
        sprintf(line3, "Stop:%lu Cal:%lu", 
                test_state.stops_count, test_state.calibration_count);
      }
      Display_Text(0, 32, line3);
      
      /* 第4行：测试模式 */
      if (test_state.test_mode == 2) {
        sprintf(line4, "Cycle: S=%lu", test_state.stops_count);
      } else {
        sprintf(line4, "Manual Mode");
      }
      Display_Text(0, 48, line4);
      
      Display_Update();
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
  
  /* Configure button pins as interrupt with pull-up */
  /* PC0, PC1, PC2, PC3, PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on button press
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* Enable EXTI interrupts for buttons */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);  // PC0
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);  // PC1
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);  // PC2
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);  // PC3
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);  // PC5
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* 获取目标楼层位置 */
int32_t GetTargetPosition(uint8_t floor) {
    if (floor < 1 || floor > 3) return 0;
    return test_state.encoder_offset + (floor - 1) * STEPS_PER_FLOOR;
}

/* 开始移动到指定楼层 */
void StartMoveToFloor(uint8_t floor) {
    if (floor < 1 || floor > 3) return;
    if (test_state.is_moving) return;
    
    test_state.target_floor = floor;
    test_state.target_position = GetTargetPosition(floor);
    
    printf("\r\n[MOVE] Floor %d -> %d (pos %ld -> %ld)\r\n",
           test_state.current_floor, floor,
           test_state.current_position, test_state.target_position);
    
    /* 发送移动命令 */
    StepperMotor_MoveAbsolute(&stepper, test_state.target_position);
    test_state.is_moving = true;
}

/* 处理光电传感器触发 */
void ProcessPhotoSensor(uint8_t floor) {
    printf("\r\n[PHOTO] Floor %d triggered at pos %ld\r\n", 
           floor, test_state.current_position);
    
    test_state.last_photo_floor = floor;
    test_state.photo_trigger_time = HAL_GetTick();
    
    /* 如果正在移动且到达目标楼层 - 立即停止 */
    if (test_state.is_moving && floor == test_state.target_floor) {
        printf("[PHOTO] Target floor reached! Stopping immediately.\r\n");
        
        /* 立即停止电机 */
        StepperMotor_Stop(&stepper);
        test_state.is_moving = false;
        
        /* 计算校准误差 */
        int32_t expected_pos = GetTargetPosition(floor);
        test_state.calibration_error = test_state.current_position - expected_pos;
        
        printf("[PHOTO] Calibration: expected=%ld, actual=%ld, error=%ld\r\n",
               expected_pos, test_state.current_position, test_state.calibration_error);
        
        /* 如果误差不太大，增加校准计数 */
        if (abs(test_state.calibration_error) < 1000) {
            test_state.calibration_count++;
            printf("[PHOTO] Calibration successful (count=%lu)\r\n", test_state.calibration_count);
        } else {
            printf("[PHOTO] Calibration error too large: %ld steps\r\n", 
                   test_state.calibration_error);
        }
        
        test_state.current_floor = floor;
        test_state.stops_count++;
        
        /* 循环测试模式 - 自动继续 */
        if (test_state.test_mode == 2) {
            HAL_Delay(2000);  // 停留2秒
            uint8_t next_floor = test_state.current_floor + 1;
            if (next_floor > 3) next_floor = 1;
            StartMoveToFloor(next_floor);
        }
    }
    /* 如果只是经过中间楼层 - 仅校准 */
    else if (test_state.is_moving) {
        printf("[PHOTO] Passing floor %d, calibrating only\r\n", floor);
        
        /* 计算校准误差 */
        int32_t expected_pos = GetTargetPosition(floor);
        test_state.calibration_error = test_state.current_position - expected_pos;
        
        printf("[PHOTO] Pass calibration: error=%ld steps\r\n", 
               test_state.calibration_error);
        
        if (abs(test_state.calibration_error) < 1000) {
            test_state.calibration_count++;
        }
        
        /* 更新当前楼层 */
        test_state.current_floor = floor;
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