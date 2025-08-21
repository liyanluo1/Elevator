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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FLOORS          3
#define STEPS_PER_FLOOR     10800      // 每层步数
#define STOP_THRESHOLD      300        // 停止阈值（未使用）
#define DOOR_SIMULATION_MS  2000       // 模拟开关门时间（第一阶段测试）
#define NORMAL_SPEED        500        // 正常运行速度
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
StepperMotor_t stepper;
static uint32_t last_display_time = 0;
static uint32_t last_status_time = 0;
extern uint32_t door_operation_start;  // FSM中的门操作开始时间
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
void ProcessButtons(void);
void ProcessRS485(void);
void UpdateOLEDDisplay(void);
void UpdateOLEDRxCount(uint32_t total, uint32_t cabin);
void ProcessStepperControl(void);
void USART2_SendString(const char* str);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  
  /* USER CODE BEGIN 2 */
  
  /* 初始化OLED */
  Display_Init();
  Display_Clear();
  Display_Text(0, 0, "ELEVATOR SYSTEM");
  Display_Text(0, 16, "Initializing...");
  Display_Update();
  HAL_Delay(500);
  
  /* 测试USART2调试输出 */
  USART2_SendString("\r\n=== USART2 DEBUG PORT ACTIVATED ===\r\n");
  USART2_SendString("PA2 (TX) / PA3 (RX) @ 115200\r\n");
  USART2_SendString("This is a secondary debug channel\r\n\r\n");
  
  printf("\r\n=== MASTER MCU - ELEVATOR CONTROL SYSTEM ===\r\n");
  printf("FSM + SCAN Scheduling Algorithm\r\n");
  printf("USART2 (PA2/PA3) enabled for additional debug\r\n\r\n");
  
  /* 初始化RS485通信 */
  rs485_init();
  Display_Clear();
  Display_Text(0, 0, "RS485 Init OK");
  Display_Update();
  HAL_Delay(500);
  
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
  
  /* 设置初始位置 - 假设从1楼开始，光电被触发 */
  printf("\r\n[INIT] System starts at floor 1 (photo sensor triggered)\r\n");
  
  /* 设置编码器偏移到Blackboard */
  Blackboard_SetEncoderOffset(stepper.current_position);
  g_blackboard.current_floor = 1;  // 从1楼开始
  g_blackboard.motor_position = stepper.current_position;
  
  /* 电机速度使用默认值 */
  printf("[STEPPER] Using default speed\r\n");
  
  printf("[INIT] System ready\r\n");
  printf("  Encoder offset: %ld\r\n", g_blackboard.encoder_offset);
  printf("  Starting floor: %d\r\n", g_blackboard.current_floor);
  printf("  Floor positions:\r\n");
  for (int i = 1; i <= MAX_FLOORS; i++) {
    printf("    Floor %d: %ld steps\r\n", i, Blackboard_GetTargetPosition(i));
  }
  
  printf("\r\n[CONTROLS]\r\n");
  printf("  1UP (PA6): Floor 1 Up Call (Changed from PC5)\r\n");
  printf("  2UP (PC1): Floor 2 Up Call\r\n");  
  printf("  2DN (PC0): Floor 2 Down Call\r\n");
  printf("  3DN (PC2): Floor 3 Down Call\r\n");
  printf("  * Cabin calls from Slave MCU keyboard\r\n");
  printf("\r\nSystem running...\r\n\r\n");
  
  /* 清除初始化信息，显示运行状态 */
  Display_Clear();
  UpdateOLEDDisplay();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 更新步进电机状态 */
    StepperMotor_Update(&stepper);
    g_blackboard.motor_position = stepper.current_position;
    
    /* 检测电机是否到达目标并停止 */
    static bool was_moving = false;
    bool is_moving = StepperMotor_IsMoving(&stepper);
    if (was_moving && !is_moving && g_blackboard.state == STATE_MOVING) {
      /* 电机刚停止，清除目标楼层呼叫 */
      if (g_blackboard.target_floor > 0 && g_blackboard.target_floor <= MAX_FLOORS) {
        printf("[MAIN] Motor stopped at target floor %d, clearing calls\r\n", 
               g_blackboard.target_floor);
        
        /* 同时输出到USART2 */
        char debug_msg[100];
        sprintf(debug_msg, "[USART2] Motor stop detected, clearing F%d calls\r\n", 
                g_blackboard.target_floor);
        USART2_SendString(debug_msg);
        
        /* 清除呼叫 */
        Blackboard_ClearCall(g_blackboard.target_floor);
        
        /* 强制更新当前楼层（因为光电可能没触发） */
        g_blackboard.current_floor = g_blackboard.target_floor;
        sprintf(debug_msg, "[USART2] Force update current floor to %d\r\n", 
                g_blackboard.current_floor);
        USART2_SendString(debug_msg);
        
        /* 清零target_floor，表示已到达 */
        g_blackboard.target_floor = g_blackboard.current_floor;
        
        /* 转到门操作状态（非阻塞） */
        Blackboard_SetState(STATE_DOOR_OPERATING);
        door_operation_start = HAL_GetTick();
        
        sprintf(debug_msg, "[USART2] Starting door operation\r\n");
        USART2_SendString(debug_msg);
      }
    }
    was_moving = is_moving;
    
    /* 处理按钮输入（楼层外呼） */
    ProcessButtons();
    
    /* 每500ms输出一次状态到USART2 */
    static uint32_t last_usart2_time = 0;
    if (current_time - last_usart2_time >= 500) {
        last_usart2_time = current_time;
        char status_msg[100];
        sprintf(status_msg, "[U2] S:%d F:%d->%d C:%d%d%d H:%d%d%d\r\n",
                g_blackboard.state,
                g_blackboard.current_floor,
                g_blackboard.target_floor,
                g_blackboard.cabin_calls[1],
                g_blackboard.cabin_calls[2],
                g_blackboard.cabin_calls[3],
                g_blackboard.up_calls[1] || g_blackboard.down_calls[1],
                g_blackboard.up_calls[2] || g_blackboard.down_calls[2],
                g_blackboard.up_calls[3] || g_blackboard.down_calls[3]);
        USART2_SendString(status_msg);
    }
    
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
  * @brief  处理按钮输入（楼层外呼） - 中断模式
  * @note   现在使用中断模式，此函数处理中断产生的按键事件
  */
void ProcessButtons(void) {
    /* 调用Button模块的处理函数，处理中断标志 */
    Button_Process();
    
    /* 检查每个按钮是否被按下 */
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (Button_IsPressed(i)) {
            Button_t* btn = &buttons[i];
            
            /* 处理楼层外呼 - 推送事件到队列 */
            if (btn->type == BUTTON_TYPE_UP) {
                printf("\r\n[BUTTON] Floor %d UP call\r\n", btn->floor);
                
                /* USART2调试 */
                char btn_msg[50];
                sprintf(btn_msg, "[U2-BTN] F%d UP pressed\r\n", btn->floor);
                USART2_SendString(btn_msg);
                
                /* 事件驱动：推送事件，由FSM处理 */
                Blackboard_PushEvent(EVENT_BUTTON_UP, btn->floor);
                
                /* 清除按钮状态，避免重复触发 */
                buttons[i].pressed = false;
            }
            else if (btn->type == BUTTON_TYPE_DOWN) {
                printf("\r\n[BUTTON] Floor %d DOWN call\r\n", btn->floor);
                
                /* USART2调试 */
                char btn_msg[50];
                sprintf(btn_msg, "[U2-BTN] F%d DOWN pressed\r\n", btn->floor);
                USART2_SendString(btn_msg);
                
                /* 事件驱动：推送事件，由FSM处理 */
                Blackboard_PushEvent(EVENT_BUTTON_DOWN, btn->floor);
                
                /* 清除按钮状态，避免重复触发 */
                buttons[i].pressed = false;
            }
        }
    }
}

/**
  * @brief  处理RS485接收
  */
void ProcessRS485(void) {
    static uint32_t total_rx_count = 0;
    static uint32_t cabin_rx_count = 0;
    
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
        total_rx_count++;
        
        /* 更新OLED显示的计数器 */
        UpdateOLEDRxCount(total_rx_count, cabin_rx_count);
        /* 光电传感器触发命令 */
        if (rx_buffer[0] == CMD_PHOTO_SENSOR && rx_len >= 2) {
            uint8_t floor = rx_buffer[1];
            
            printf("\r\n========== PHOTO SENSOR TRIGGERED ==========\r\n");
            printf("[RS485 RX] Photo sensor floor %d\r\n", floor);
            printf("[DEBUG] Current state: %s, Target floor: %d\r\n", 
                   Blackboard_GetStateName(g_blackboard.state), g_blackboard.target_floor);
            
            /* 事件驱动：推送光电传感器事件 */
            Blackboard_PushEvent(EVENT_PHOTO_SENSOR, floor);
            printf("[DEBUG] Photo event pushed, will be processed by FSM\r\n");
            printf("=============================================\r\n");
        }
        /* 轿厢内呼命令 */
        else if (rx_buffer[0] == CMD_CABIN_CALL && rx_len >= 2) {
            uint8_t floor = rx_buffer[1];
            cabin_rx_count++;
            
            printf("\r\n========== CABIN CALL RECEIVED ==========\r\n");
            printf("[RS485 RX] Cabin call floor %d (total cabin calls: %lu)\r\n", floor, cabin_rx_count);
            printf("[DEBUG] Current state: %s, Current floor: %d\r\n", 
                   Blackboard_GetStateName(g_blackboard.state), g_blackboard.current_floor);
            
            /* 事件驱动：推送内呼事件 */
            Blackboard_PushEvent(EVENT_CABIN_CALL, floor);
            printf("[DEBUG] Event pushed to queue\r\n");
            printf("==========================================\r\n");
            
            /* 更新计数并立即显示 */
            UpdateOLEDRxCount(total_rx_count, cabin_rx_count);
            UpdateOLEDDisplay();
        }
        /* 门状态反馈 - TIME BASED模式下不使用 */
        // else if (rx_buffer[0] == CMD_DOOR_STATUS && rx_len >= 2) {
        //     uint8_t status = rx_buffer[1];
        //     /* Slave发送: 0=CLOSED, 1=OPENING, 2=OPEN, 3=CLOSING */
        //     const char* door_state_str[] = {"CLOSED", "OPENING", "OPEN", "CLOSING"};
        //     if (status <= 3) {
        //         g_blackboard.door_state = (DoorState_t)status;
        //         printf("[RS485 RX] Door state: %s\r\n", door_state_str[status]);
        //     }
        // }
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
                    
                    /* 如果已经在目标位置附近（300步内），直接清除呼叫 */
                    if (abs(stepper.current_position - target_pos) < 300) {
                        printf("[MOTOR] Already near floor %d, clearing calls\r\n", target_floor);
                        Blackboard_ClearCall(target_floor);
                        /* 不需要移动 */
                    } else {
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
                }
                break;
                
            case MOTOR_CMD_STOP:
                printf("[MOTOR] Stop\r\n");
                StepperMotor_Stop(&stepper);
                
                /* 电机停止时清除目标楼层的呼叫，避免光电未触发导致的死循环 */
                if (g_blackboard.target_floor > 0 && g_blackboard.target_floor <= MAX_FLOORS) {
                    printf("[MOTOR] Clearing calls at target floor %d\r\n", g_blackboard.target_floor);
                    Blackboard_ClearCall(g_blackboard.target_floor);
                }
                
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

/* 全局计数器用于OLED显示 */
static uint32_t g_total_rx_count = 0;
static uint32_t g_cabin_rx_count = 0;

/**
  * @brief  更新OLED接收计数
  */
void UpdateOLEDRxCount(uint32_t total, uint32_t cabin) {
    g_total_rx_count = total;
    g_cabin_rx_count = cabin;
}

/**
  * @brief  更新OLED显示
  */
void UpdateOLEDDisplay(void) {
    char line1[20], line2[32], line4[20];
    
    Display_Clear();
    
    /* 第1行：楼层和状态 */
    sprintf(line1, "F%d %s", g_blackboard.current_floor, 
            g_blackboard.state == STATE_IDLE ? "IDLE" : 
            g_blackboard.state == STATE_MOVING ? "MOVE" : "DOOR");
    Display_Text(0, 0, line1);
    
    /* 第2行：内呼显示和接收计数 */
    char cabin_str[20] = "C:";
    int has_cabin = 0;
    for (uint8_t f = 1; f <= MAX_FLOORS; f++) {
        if (g_blackboard.cabin_calls[f]) {
            char temp[3];
            sprintf(temp, "%d", f);
            strcat(cabin_str, temp);
            has_cabin = 1;
        }
    }
    if (!has_cabin) {
        strcat(cabin_str, "-");
    }
    sprintf(line2, "%s RX:%lu", cabin_str, g_cabin_rx_count);
    Display_Text(0, 16, line2);
    
    /* 第3行：外呼显示 - 更详细 */
    char hall_str[32] = "H:";
    int has_hall = 0;
    for (uint8_t f = 1; f <= MAX_FLOORS; f++) {
        if (g_blackboard.up_calls[f]) {
            char temp[4];
            sprintf(temp, "%d^", f);  // 上箭头表示上行
            strcat(hall_str, temp);
            has_hall = 1;
        }
        if (g_blackboard.down_calls[f]) {
            char temp[4];
            sprintf(temp, "%dv", f);  // 下箭头表示下行
            strcat(hall_str, temp);
            has_hall = 1;
        }
    }
    if (!has_hall) {
        strcat(hall_str, "-");
    }
    Display_Text(0, 32, hall_str);
    
    /* 第4行：RS485调试信息 */
    sprintf(line4, "485RX:%lu", g_total_rx_count);
    Display_Text(0, 48, line4);
    
    Display_Update();
}

/**
  * @brief  UART TX DMA完成回调
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    extern void rs485_tx_complete_callback(void);
    if (huart == &huart5) {
        rs485_tx_complete_callback();
    }
}

/**
  * @brief  UART RX DMA完成回调
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // DMA循环模式下不需要处理
}

/**
  * @brief  发送字符串到USART2调试端口
  * @param  str: 要发送的字符串
  */
void USART2_SendString(const char* str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
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

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  
  /* Configure button pins: PC0, PC1, PC2, PC3 as interrupt mode (PC5已弃用) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿触发中断（按下时）
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* Configure PA6 for Floor 1 UP button as interrupt mode (替代原PC5) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿触发中断（按下时）
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Enable and set EXTI line Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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