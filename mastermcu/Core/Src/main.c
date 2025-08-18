/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Master MCU - 完整电梯控制系统（FSM + SCAN调度）
  ******************************************************************************
  * @attention
  * 
  * 系统功能：
  * 1. FSM状态机控制（IDLE -> PREPARING -> MOVING -> DOOR_OPERATING）
  * 2. SCAN调度算法处理多个呼叫
  * 3. 光电传感器精确停层
  * 4. 按钮呼叫处理（楼层外呼）
  * 5. RS485通信（接收Slave的键盘内呼和光电信号）
  * 6. OLED实时状态显示
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
#include "../Modules/Blackboard/blackboard.h"
#include "../Modules/FSM/elevator_fsm.h"
#include "../Modules/RS485/rs485_protocol.h"
#include "../Modules/RS485/rs485_test.h"
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
#define PREPARING_TIME_MS   1000   // PREPARING状态持续时间
#define DOOR_OPEN_TIME_MS   3000   // 开门持续时间
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
StepperMotor_t stepper;
static uint32_t last_display_time = 0;
static uint32_t last_status_time = 0;
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
void ProcessButtons(void);
void ProcessRS485(void);
void UpdateOLEDDisplay(void);
void ProcessStepperControl(void);
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
  Display_Text(0, 0, "ELEVATOR SYSTEM");
  Display_Text(0, 16, "Initializing...");
  Display_Update();
  HAL_Delay(500);
  
  printf("\r\n=== MASTER MCU - ELEVATOR CONTROL SYSTEM ===\r\n");
  printf("FSM + SCAN Scheduling Algorithm\r\n\r\n");
  
  /* 初始化RS485通信 */
  rs485_init();
  printf("[RS485] Initialized on UART5\r\n");
  
  /* 初始化RS485测试模块 */
  RS485_Test_Init();
  RS485_Test_SetMode(TEST_MODE_SIMPLE);  // 开始简单测试
  
  /* 初始化按钮 */
  Button_Init();
  printf("[BUTTON] Initialized\r\n");
  
  /* 初始化Blackboard */
  Blackboard_Init();
  printf("[BLACKBOARD] Initialized\r\n");
  
  /* 初始化FSM */
  FSM_Init();
  printf("[FSM] Initialized - State: IDLE\r\n");
  
  /* 初始化步进电机 */
  printf("[STEPPER] Initializing...\r\n");
  StepperMotor_Init(&stepper);
  HAL_Delay(100);
  
  printf("[STEPPER] Enabling motor...\r\n");
  StepperMotor_Enable(&stepper);
  HAL_Delay(500);  // 增加延时确保电机使能
  
  /* 检查电机状态 */
  printf("[STEPPER] Motor enabled: %s\r\n", 
         StepperMotor_IsEnabled(&stepper) ? "YES" : "NO");
  printf("[STEPPER] Motor connected: %s\r\n",
         StepperMotor_IsConnected(&stepper) ? "YES" : "NO");
  
  /* 读取编码器初始值作为偏移 */
  printf("[ENCODER] Reading initial position...\r\n");
  for (int retry = 0; retry < 10; retry++) {
    StepperMotor_Update(&stepper);
    HAL_Delay(100);
    if (stepper.current_position != 0 || retry > 5) {
      break;
    }
  }
  
  /* 设置编码器偏移到Blackboard */
  Blackboard_SetEncoderOffset(stepper.current_position);
  g_blackboard.current_floor = 1;  // 假设从1楼开始
  g_blackboard.motor_position = stepper.current_position;
  
  printf("[INIT] System ready\r\n");
  printf("  Encoder offset: %ld\r\n", g_blackboard.encoder_offset);
  printf("  Starting floor: %d\r\n", g_blackboard.current_floor);
  printf("  Floor positions:\r\n");
  for (int i = 1; i <= MAX_FLOORS; i++) {
    printf("    Floor %d: %ld steps\r\n", i, Blackboard_GetTargetPosition(i));
  }
  
  printf("\r\n[CONTROLS]\r\n");
  printf("  1UP (PC5): Floor 1 Up Call\r\n");
  printf("  2UP (PC1): Floor 2 Up Call\r\n");  
  printf("  2DN (PC0): Floor 2 Down Call\r\n");
  printf("  3DN (PC2): Floor 3 Down Call\r\n");
  printf("  * Cabin calls from Slave MCU keyboard\r\n");
  printf("\r\nSystem running...\r\n\r\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 运行RS485测试 */
    RS485_Test_Process();
    
    /* 更新步进电机状态 */
    StepperMotor_Update(&stepper);
    g_blackboard.motor_position = stepper.current_position;
    
    /* 处理按钮输入（楼层外呼） */
    ProcessButtons();
    
    /* 处理RS485接收（内呼和光电信号） */
    ProcessRS485();
    
    /* 运行FSM状态机 */
    FSM_Process();
    
    /* 处理步进电机控制命令 */
    ProcessStepperControl();
    
    /* 更新OLED显示（每100ms） */
    if (current_time - last_display_time >= 100) {
      last_display_time = current_time;
      UpdateOLEDDisplay();
    }
    
    /* 每5秒打印状态 */
    if (current_time - last_status_time >= 5000) {
      last_status_time = current_time;
      Blackboard_PrintStatus();
    }
    
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  处理按钮输入（楼层外呼）
  */
void ProcessButtons(void) {
    static bool button_pressed[NUM_BUTTONS] = {false};
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        Button_t* btn = &buttons[i];
        GPIO_PinState state = HAL_GPIO_ReadPin(btn->port, btn->pin);
        
        if (state == GPIO_PIN_RESET && !button_pressed[i]) {
            button_pressed[i] = true;
            
            /* 处理楼层外呼 */
            if (btn->type == BUTTON_TYPE_UP) {
                printf("\r\n[BUTTON] Floor %d UP call\r\n", btn->floor);
                Blackboard_AddUpCall(btn->floor);
                Blackboard_PushEvent(EVENT_BUTTON_PRESS, btn->floor);
            }
            else if (btn->type == BUTTON_TYPE_DOWN) {
                printf("\r\n[BUTTON] Floor %d DOWN call\r\n", btn->floor);
                Blackboard_AddDownCall(btn->floor);
                Blackboard_PushEvent(EVENT_BUTTON_PRESS, btn->floor);
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
        printf("\r\n[RS485 DEBUG] Received %d bytes: ", rx_len);
        for(int i = 0; i < rx_len && i < 8; i++) {
            printf("0x%02X ", rx_buffer[i]);
        }
        printf("\r\n");
        /* 光电传感器触发命令 */
        if (rx_buffer[0] == CMD_PHOTO_SENSOR && rx_len >= 2) {
            uint8_t floor = rx_buffer[1];
            printf("\r\n[RS485 RX] Photo sensor floor %d\r\n", floor);
            
            /* 推送光电传感器事件 */
            Blackboard_PushEvent(EVENT_PHOTO_SENSOR, floor);
            
            /* 如果正在移动且到达目标楼层，让FSM处理停止 */
            if (g_blackboard.state == STATE_MOVING && 
                floor == g_blackboard.target_floor) {
                FSM_HandlePhotoSensor(floor);
            }
            /* 校准当前楼层位置 */
            else if (g_blackboard.state == STATE_MOVING) {
                printf("[PHOTO] Passing floor %d\r\n", floor);
                g_blackboard.current_floor = floor;
                Blackboard_CalibratePosition(floor);
            }
        }
        /* 轿厢内呼命令 */
        else if (rx_buffer[0] == CMD_CABIN_CALL && rx_len >= 2) {
            uint8_t floor = rx_buffer[1];
            printf("\r\n========================================\r\n");
            printf("[RS485 RX] CABIN CALL received!\r\n");
            printf("  Floor: %d\r\n", floor);
            printf("  Current floor: %d\r\n", g_blackboard.current_floor);
            printf("  Current state: %s\r\n", Blackboard_GetStateName(g_blackboard.state));
            printf("========================================\r\n");
            
            Blackboard_AddCabinCall(floor);
            Blackboard_PushEvent(EVENT_CABIN_CALL, floor);
            
            /* 如果在IDLE状态，立即触发FSM处理 */
            if (g_blackboard.state == STATE_IDLE) {
                printf("[RS485] Triggering FSM from IDLE for cabin call\r\n");
            }
        }
        /* 门状态反馈 */
        else if (rx_buffer[0] == CMD_DOOR_STATUS && rx_len >= 2) {
            uint8_t status = rx_buffer[1];
            g_blackboard.door_state = status ? DOOR_OPEN : DOOR_CLOSED;
            printf("[RS485 RX] Door %s\r\n", status ? "OPEN" : "CLOSED");
        }
    }
}

/**
  * @brief  处理步进电机控制命令
  */
void ProcessStepperControl(void) {
    MotorCommand_t cmd = Blackboard_GetMotorCommand();
    
    if (cmd != MOTOR_CMD_NONE) {
        printf("[MOTOR] Processing command: %d\r\n", cmd);
        switch (cmd) {
            case MOTOR_CMD_MOVE_TO:
                {
                    int32_t target_pos = Blackboard_GetMotorCommandParam();
                    printf("[MOTOR] Move to position %ld\r\n", target_pos);
                    StepperMotor_MoveAbsolute(&stepper, target_pos);
                    
                    /* 发送方向信息给Slave */
                    uint8_t direction = (target_pos > stepper.current_position) ? DIR_UP : DIR_DOWN;
                    uint8_t tx_buffer[4];
                    tx_buffer[0] = CMD_DIRECTION_SET;
                    tx_buffer[1] = direction;
                    tx_buffer[2] = g_blackboard.current_floor;
                    tx_buffer[3] = g_blackboard.target_floor;
                    rs485_send_packet_dma(tx_buffer, 4);
                }
                break;
                
            case MOTOR_CMD_MOVE_TO_FLOOR:
                {
                    uint8_t target_floor = (uint8_t)Blackboard_GetMotorCommandParam();
                    int32_t target_pos = Blackboard_GetTargetPosition(target_floor);
                    printf("[MOTOR] Move to floor %d (position %ld)\r\n", target_floor, target_pos);
                    StepperMotor_MoveAbsolute(&stepper, target_pos);
                    
                    /* 发送方向信息给Slave */
                    uint8_t direction = (target_pos > stepper.current_position) ? DIR_UP : DIR_DOWN;
                    uint8_t tx_buffer[4];
                    tx_buffer[0] = CMD_DIRECTION_SET;
                    tx_buffer[1] = direction;
                    tx_buffer[2] = g_blackboard.current_floor;
                    tx_buffer[3] = target_floor;
                    rs485_send_packet_dma(tx_buffer, 4);
                }
                break;
                
            case MOTOR_CMD_STOP:
                printf("[MOTOR] Stop\r\n");
                StepperMotor_Stop(&stepper);
                
                /* 通知Slave停止 */
                uint8_t tx_buffer[4];
                tx_buffer[0] = CMD_DIRECTION_SET;
                tx_buffer[1] = DIR_STOP;
                tx_buffer[2] = g_blackboard.current_floor;
                tx_buffer[3] = g_blackboard.current_floor;
                rs485_send_packet_dma(tx_buffer, 4);
                break;
                
            case MOTOR_CMD_EMERGENCY_STOP:
                printf("[MOTOR] EMERGENCY STOP!\r\n");
                StepperMotor_Stop(&stepper);
                break;
        }
        
        Blackboard_ClearMotorCommand();
    }
}

/**
  * @brief  更新OLED显示
  */
void UpdateOLEDDisplay(void) {
    char line1[20], line2[20], line3[20], line4[20];
    
    Display_Clear();
    
    /* 第1行：状态和楼层 */
    const char* state_str = Blackboard_GetStateName(g_blackboard.state);
    if (g_blackboard.state == STATE_MOVING) {
        sprintf(line1, "%s F%d->F%d", state_str,
                g_blackboard.current_floor, g_blackboard.target_floor);
    } else {
        sprintf(line1, "%s Floor %d", state_str, g_blackboard.current_floor);
    }
    Display_Text(0, 0, line1);
    
    /* 第2行：呼叫状态 */
    char calls[20] = "";
    int call_count = 0;
    for (uint8_t f = 1; f <= MAX_FLOORS; f++) {
        if (g_blackboard.up_calls[f]) {
            sprintf(calls + strlen(calls), "%d^ ", f);
            call_count++;
        }
        if (g_blackboard.down_calls[f]) {
            sprintf(calls + strlen(calls), "%dv ", f);
            call_count++;
        }
        if (g_blackboard.cabin_calls[f]) {
            sprintf(calls + strlen(calls), "%d* ", f);
            call_count++;
        }
    }
    if (call_count > 0) {
        sprintf(line2, "Call:%s", calls);
    } else {
        sprintf(line2, "No calls");
    }
    Display_Text(0, 16, line2);
    
    /* 第3行：位置信息 */
    sprintf(line3, "Pos:%ld", g_blackboard.motor_position);
    Display_Text(0, 32, line3);
    
    /* 第4行：方向和门状态 */
    const char* dir_str = (g_blackboard.direction == DIR_UP) ? "UP" :
                         (g_blackboard.direction == DIR_DOWN) ? "DN" : "--";
    const char* door_str = (g_blackboard.door_state == DOOR_OPEN) ? "OPEN" :
                          (g_blackboard.door_state == DOOR_CLOSED) ? "CLOSE" : "MOVE";
    sprintf(line4, "Dir:%s Door:%s", dir_str, door_str);
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

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */