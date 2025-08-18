/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_test.c
  * @brief          : Master MCU - 基本停层测试（光电触发立即停止）
  ******************************************************************************
  * @attention
  * 
  * 测试内容：
  * 1. 按钮控制移动到指定楼层
  * 2. 接收光电传感器信号立即停止
  * 3. 发送方向信息给Slave MCU
  * 4. OLED显示实时状态
  * 
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
#include "../Modules/RS485/rs485_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FLOORS          3
#define STEPS_PER_FLOOR     10800
#define STOP_THRESHOLD      300
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
/* 测试状态结构 */
typedef struct {
    /* 位置信息 */
    int32_t encoder_offset;      // 编码器偏移（开机读取）
    int32_t current_position;    // 当前位置
    int32_t target_position;     // 目标位置
    uint8_t current_floor;       // 当前楼层
    uint8_t target_floor;        // 目标楼层
    
    /* 运动状态 */
    bool is_moving;              // 是否在移动
    
    /* 光电传感器 */
    uint8_t last_photo_floor;    // 最后触发的楼层
    uint32_t photo_trigger_time; // 触发时间
    uint32_t photo_stop_count;   // 光电停止次数
    
    /* RS485通信 */
    uint32_t rs485_rx_count;     // 接收计数
    uint32_t rs485_tx_count;     // 发送计数
} TestState_t;

static TestState_t test_state = {0};
static StepperMotor_t stepper;
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
int32_t GetTargetPosition(uint8_t floor);
void StartMoveToFloor(uint8_t floor);
void ProcessPhotoSensor(uint8_t floor);
void ProcessButtons(void);
void ProcessRS485(void);
void UpdateOLEDDisplay(void);
/* USER CODE END PFP */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  
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
  Display_Text(0, 0, "BASIC STOP TEST");
  Display_Text(0, 16, "Initializing...");
  Display_Update();
  HAL_Delay(500);
  
  printf("\r\n=== MASTER MCU - BASIC STOP TEST ===\r\n");
  printf("Photo sensor triggered immediate stop\r\n\r\n");
  
  /* 初始化RS485通信 */
  rs485_init();
  printf("[RS485] Initialized on UART5\r\n");
  
  /* 初始化按钮 */
  Button_Init();
  printf("[BUTTON] Initialized (interrupt mode)\r\n");
  
  /* 初始化步进电机 */
  printf("[STEPPER] Initializing...\r\n");
  StepperMotor_Init(&stepper);
  HAL_Delay(100);
  
  StepperMotor_Enable(&stepper);
  HAL_Delay(100);
  
  /* 读取编码器初始值作为偏移 */
  printf("[ENCODER] Reading initial position...\r\n");
  for (int retry = 0; retry < 10; retry++) {
    StepperMotor_Update(&stepper);
    HAL_Delay(100);
    if (stepper.current_position != 0 || retry > 5) {
      break;
    }
  }
  
  /* 设置初始状态 - 假设在1楼 */
  test_state.encoder_offset = stepper.current_position;
  test_state.current_floor = 1;
  test_state.current_position = stepper.current_position;
  test_state.is_moving = false;
  
  printf("[INIT] System ready\r\n");
  printf("  Encoder offset: %ld\r\n", test_state.encoder_offset);
  printf("  Starting floor: %d\r\n", test_state.current_floor);
  printf("  Floor positions:\r\n");
  for (int i = 1; i <= MAX_FLOORS; i++) {
    printf("    Floor %d: %ld steps\r\n", i, GetTargetPosition(i));
  }
  
  printf("\r\n[CONTROLS]\r\n");
  printf("  1UP (PC5): Go to Floor 1\r\n");
  printf("  2UP (PC1): Go to Floor 2\r\n");  
  printf("  3UP (PC3): Go to Floor 3\r\n");
  printf("  3DN (PC2): Emergency stop\r\n");
  printf("\r\nWaiting for commands...\r\n\r\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_display_time = 0;
  uint32_t last_status_time = 0;
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 更新步进电机状态 */
    StepperMotor_Update(&stepper);
    test_state.current_position = stepper.current_position;
    
    /* 处理按钮输入 */
    ProcessButtons();
    
    /* 处理RS485接收 */
    ProcessRS485();
    
    /* 检查是否到达目标位置（备用停止） */
    if (test_state.is_moving && test_state.target_floor > 0) {
      int32_t error = abs(test_state.current_position - test_state.target_position);
      if (error < STOP_THRESHOLD) {
        /* 编码器位置到达 */
        printf("[ENCODER STOP] Reached floor %d by position\r\n", test_state.target_floor);
        StepperMotor_Stop(&stepper);
        test_state.is_moving = false;
        test_state.current_floor = test_state.target_floor;
        
        /* 通知Slave停止 */
        uint8_t tx_buffer[4];
        tx_buffer[0] = CMD_DIRECTION_SET;
        tx_buffer[1] = DIR_STOP;
        tx_buffer[2] = test_state.current_floor;
        tx_buffer[3] = test_state.current_floor;
        rs485_send_packet_dma(tx_buffer, 4);
        test_state.rs485_tx_count++;
      }
    }
    
    /* 更新OLED显示（每100ms） */
    if (current_time - last_display_time >= 100) {
      last_display_time = current_time;
      UpdateOLEDDisplay();
    }
    
    /* 每5秒打印状态 */
    if (current_time - last_status_time >= 5000) {
      last_status_time = current_time;
      printf("[STATUS] Floor=%d, Moving=%s, PhotoStop=%lu, RX=%lu, TX=%lu\r\n",
             test_state.current_floor,
             test_state.is_moving ? "YES" : "NO",
             test_state.photo_stop_count,
             test_state.rs485_rx_count,
             test_state.rs485_tx_count);
    }
    
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  获取目标楼层位置
  */
int32_t GetTargetPosition(uint8_t floor) {
    if (floor < 1 || floor > MAX_FLOORS) return 0;
    return test_state.encoder_offset + (floor - 1) * STEPS_PER_FLOOR;
}

/**
  * @brief  开始移动到指定楼层
  */
void StartMoveToFloor(uint8_t floor) {
    if (floor < 1 || floor > MAX_FLOORS) return;
    if (test_state.is_moving) {
        printf("[MOVE] Already moving, ignore\r\n");
        return;
    }
    
    if (floor == test_state.current_floor) {
        printf("[MOVE] Already at floor %d\r\n", floor);
        return;
    }
    
    test_state.target_floor = floor;
    test_state.target_position = GetTargetPosition(floor);
    
    printf("\r\n========================================\r\n");
    printf("[MOVE] Start: Floor %d -> Floor %d\r\n", test_state.current_floor, floor);
    printf("  Current pos: %ld\r\n", test_state.current_position);
    printf("  Target pos:  %ld\r\n", test_state.target_position);
    printf("  Distance:    %ld steps\r\n", test_state.target_position - test_state.current_position);
    
    /* 发送方向命令给Slave */
    uint8_t direction = (floor > test_state.current_floor) ? DIR_UP : DIR_DOWN;
    
    uint8_t tx_buffer[4];
    tx_buffer[0] = CMD_DIRECTION_SET;
    tx_buffer[1] = direction;
    tx_buffer[2] = test_state.current_floor;
    tx_buffer[3] = floor;
    
    rs485_send_packet_dma(tx_buffer, 4);
    test_state.rs485_tx_count++;
    printf("[RS485 TX] Direction %s, F%d->F%d\r\n",
           direction == DIR_UP ? "UP" : "DOWN",
           test_state.current_floor, floor);
    
    /* 启动步进电机 */
    StepperMotor_MoveAbsolute(&stepper, test_state.target_position);
    test_state.is_moving = true;
    printf("[STEPPER] Moving started\r\n");
    printf("========================================\r\n\r\n");
}

/**
  * @brief  处理光电传感器触发
  */
void ProcessPhotoSensor(uint8_t floor) {
    printf("\r\n[PHOTO] Floor %d detected at pos %ld\r\n", 
           floor, test_state.current_position);
    
    test_state.last_photo_floor = floor;
    test_state.photo_trigger_time = HAL_GetTick();
    
    /* 如果正在移动且到达目标楼层 - 立即停止 */
    if (test_state.is_moving && floor == test_state.target_floor) {
        printf("========================================\r\n");
        printf("[PHOTO STOP] TARGET FLOOR REACHED!\r\n");
        printf("  Floor: %d\r\n", floor);
        printf("  Position: %ld\r\n", test_state.current_position);
        
        /* 立即停止电机 */
        StepperMotor_Stop(&stepper);
        test_state.is_moving = false;
        test_state.current_floor = floor;
        test_state.photo_stop_count++;
        
        /* 计算位置误差 */
        int32_t expected_pos = GetTargetPosition(floor);
        int32_t error = test_state.current_position - expected_pos;
        
        printf("  Expected pos: %ld\r\n", expected_pos);
        printf("  Error: %ld steps\r\n", error);
        printf("  Photo stops: %lu\r\n", test_state.photo_stop_count);
        printf("========================================\r\n\r\n");
        
        /* 发送停止方向给Slave */
        uint8_t tx_buffer[4];
        tx_buffer[0] = CMD_DIRECTION_SET;
        tx_buffer[1] = DIR_STOP;
        tx_buffer[2] = floor;
        tx_buffer[3] = floor;
        rs485_send_packet_dma(tx_buffer, 4);
        test_state.rs485_tx_count++;
        printf("[RS485 TX] Direction STOP at F%d\r\n", floor);
    }
    /* 如果只是经过中间楼层 */
    else if (test_state.is_moving) {
        printf("[PHOTO] Passing floor %d (target is F%d)\r\n", 
               floor, test_state.target_floor);
        
        /* 更新当前楼层但不停止 */
        test_state.current_floor = floor;
    }
}

/**
  * @brief  处理按钮输入
  */
void ProcessButtons(void) {
    static bool button_pressed[NUM_BUTTONS] = {false};
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        Button_t* btn = &buttons[i];
        GPIO_PinState state = HAL_GPIO_ReadPin(btn->port, btn->pin);
        
        if (state == GPIO_PIN_RESET && !button_pressed[i]) {
            button_pressed[i] = true;
            
            /* 处理按钮 */
            if (btn->floor == 1 && btn->type == BUTTON_TYPE_UP) {
                printf("\r\n[BUTTON] 1UP pressed -> Floor 1\r\n");
                StartMoveToFloor(1);
            }
            else if (btn->floor == 2 && btn->type == BUTTON_TYPE_UP) {
                printf("\r\n[BUTTON] 2UP pressed -> Floor 2\r\n");
                StartMoveToFloor(2);
            }
            else if (btn->floor == 3 && btn->type == BUTTON_TYPE_UP) {
                printf("\r\n[BUTTON] 3UP pressed -> Floor 3\r\n");
                StartMoveToFloor(3);
            }
            else if (btn->floor == 3 && btn->type == BUTTON_TYPE_DOWN) {
                /* 紧急停止 */
                printf("\r\n[BUTTON] EMERGENCY STOP!\r\n");
                StepperMotor_Stop(&stepper);
                test_state.is_moving = false;
                
                /* 通知Slave停止 */
                uint8_t tx_buffer[4];
                tx_buffer[0] = CMD_DIRECTION_SET;
                tx_buffer[1] = DIR_STOP;
                tx_buffer[2] = test_state.current_floor;
                tx_buffer[3] = test_state.current_floor;
                rs485_send_packet_dma(tx_buffer, 4);
                test_state.rs485_tx_count++;
            }
        }
        else if (state == GPIO_PIN_SET) {
            button_pressed[i] = false;
        }
    }
}

/**
  * @brief  处理RS485接收
  */
void ProcessRS485(void) {
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
        test_state.rs485_rx_count++;
        
        /* 光电传感器触发命令: [CMD_PHOTO_SENSOR][Floor][0][0] */
        if (rx_buffer[0] == CMD_PHOTO_SENSOR && rx_len >= 2) {
            uint8_t floor = rx_buffer[1];
            ProcessPhotoSensor(floor);
        }
    }
}

/**
  * @brief  更新OLED显示
  */
void UpdateOLEDDisplay(void) {
    char line1[20], line2[20], line3[20], line4[20];
    
    Display_Clear();
    
    /* 第1行：状态 */
    if (test_state.is_moving) {
        sprintf(line1, "MOVING F%d->F%d", 
                test_state.current_floor, test_state.target_floor);
    } else {
        sprintf(line1, "IDLE Floor %d", test_state.current_floor);
    }
    Display_Text(0, 0, line1);
    
    /* 第2行：位置 */
    sprintf(line2, "Pos:%ld", test_state.current_position);
    Display_Text(0, 16, line2);
    
    /* 第3行：光电信息 */
    if (HAL_GetTick() - test_state.photo_trigger_time < 2000) {
        sprintf(line3, "PHOTO F%d STOP!", test_state.last_photo_floor);
    } else {
        sprintf(line3, "PhotoStop:%lu", test_state.photo_stop_count);
    }
    Display_Text(0, 32, line3);
    
    /* 第4行：RS485状态 */
    sprintf(line4, "RX:%lu TX:%lu", 
            test_state.rs485_rx_count, test_state.rs485_tx_count);
    Display_Text(0, 48, line4);
    
    Display_Update();
}

/* USER CODE END 4 */

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

/* Peripheral initialization functions */
static void MX_CAN1_Init(void)
{
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
}

static void MX_UART5_Init(void)
{
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
}

static void MX_USART3_UART_Init(void)
{
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
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  
  /* Configure GPIO pin Output Level for OLED */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  /* Configure PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Configure OLED pins : PD11 PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /* Configure button pins: PC0, PC1, PC2, PC3, PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}