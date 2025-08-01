/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Slave MCU
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "local_blackboard.h"
#include "rs485_slave_adapter.h"
#include "photo_sensor.h"
#include "../Modules/keyboard/keyboard.h"
#include "../Modules/servo/servo_control.h"
#include "../Modules/LED/LED.h"
#include <stdio.h>
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
// 局部FSM处理函数
static void ProcessLocalEvents(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SlaveMCU_Init(void);
static void SlaveMCU_MainLoop(void);
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
  MX_USART2_UART_Init();  // RS485通信
  MX_USART3_UART_Init();  // 舵机通信
  /* USER CODE BEGIN 2 */
  
  // 启动UART接收中断（初始化第一次接收）
  
  // 初始化Slave MCU所有模块
  SlaveMCU_Init();
  
  printf("Slave MCU Started - Elevator Control System\n");
  printf("Modules: Photo Sensor, Servo, Keyboard, RS485\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 主循环处理
    SlaveMCU_MainLoop();
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

/**
  * @brief  初始化Slave MCU所有模块
  * @retval None
  */
static void SlaveMCU_Init(void) {
    // 初始化本地黑板
    LocalBlackboard_Init();
    
    // 初始化RS485通信（从机模式）
    RS485_SlaveAdapter_Init(&g_local_bb);
    
    // 初始化光电传感器
    PhotoSensor_Init();
    
    // 初始化舵机控制
    ServoControl_Init();
    
    // 初始化键盘
    Keyboard_Init();
    
    // 初始化LED（状态指示）
    LED_Init();
    
    // 设置初始状态
    LocalBlackboard_SetState(ELEVATOR_IDLE);
    LocalBlackboard_SetDoorState(DOOR_CLOSED);
    LocalBlackboard_SetCurrentFloor(1);  // 默认1楼
    
    // 启动UART接收中断
    static uint8_t rs485_rx_buffer;
    static uint8_t servo_rx_buffer;
    HAL_UART_Receive_IT(&huart2, &rs485_rx_buffer, 1);
    HAL_UART_Receive_IT(&huart3, &servo_rx_buffer, 1);
    
    // 延时确保所有模块初始化完成
    HAL_Delay(100);
}

/**
  * @brief  Slave MCU主循环
  * @retval None
  */
static void SlaveMCU_MainLoop(void) {
    // 更新时间戳
    g_local_bb.timestamp = HAL_GetTick();
    
    // 1. RS485通信处理（最高优先级）
    RS485_SlaveAdapter_Handler();
    
    // 2. 处理本地事件
    ProcessLocalEvents();
    
    // 3. 更新各模块
    PhotoSensor_Update();      // 光电传感器更新
    ServoControl_Update();     // 舵机控制更新
    Keyboard_Handler();        // 键盘扫描处理
    
    // 4. RS485状态同步检查
    RS485_Slave_CheckSync();
    RS485_Slave_UpdateFromBlackboard();
    
    // 5. LED状态指示
    static uint32_t led_update_time = 0;
    if (HAL_GetTick() - led_update_time > 500) {
        led_update_time = HAL_GetTick();
        
        // 根据状态控制LED
        if (RS485_Slave_IsConnected()) {
            LED_Toggle();  // 连接正常，闪烁
        } else {
            LED_On();      // 断开连接，常亮
        }
    }
    
    // 6. 看门狗喂狗（如果启用）
    // HAL_IWDG_Refresh(&hiwdg);
}

/**
  * @brief  处理本地事件
  * @retval None
  */
static void ProcessLocalEvents(void) {
    LocalEvent_t event;
    
    // 处理所有待处理事件
    while (LocalBlackboard_PopEvent(&event)) {
        switch (event.type) {
            case EVENT_OPEN_DOOR:
                // 开门事件
                if (!ServoControl_IsDoorMoving()) {
                    ServoControl_OpenDoor();
                }
                break;
                
            case EVENT_CLOSE_DOOR:
                // 关门事件
                if (!ServoControl_IsDoorMoving()) {
                    ServoControl_CloseDoor();
                }
                break;
                
            case EVENT_FLOOR_REACHED:
                // 到达楼层事件
                printf("Floor reached: %d\n", event.data);
                
                // 检查是否需要开门
                if (g_local_bb.target_floor == event.data) {
                    LocalBlackboard_PushEvent(EVENT_OPEN_DOOR, 0);
                }
                break;
                
            case EVENT_POSITION_ADJUST:
                // 位置调整事件
                printf("Position adjust: %d\n", event.data);
                break;
                
            case EVENT_ERROR:
                // 错误事件
                printf("Error occurred: 0x%04X\n", event.data);
                
                // 错误处理
                if (event.data == 0x400) {  // 紧急停止
                    ServoControl_StopDoor();
                    LED_On();  // 错误指示
                }
                break;
                
            case EVENT_SYNC_TIMEOUT:
                // 同步超时
                printf("RS485 sync timeout\n");
                break;
                
            default:
                break;
        }
    }
    
    // 检查门控制超时
    static uint32_t door_check_time = 0;
    if (HAL_GetTick() - door_check_time > 1000) {  // 每秒检查一次
        door_check_time = HAL_GetTick();
        
        // 如果门已开启超过5秒，自动关门
        if (g_local_bb.door == DOOR_OPEN) {
            if (HAL_GetTick() - g_local_bb.door_timeout_start > 5000) {
                LocalBlackboard_PushEvent(EVENT_CLOSE_DOOR, 0);
            }
        }
    }
}

/**
  * @brief  EXTI中断回调（光电传感器）
  * @param  GPIO_Pin: 触发中断的引脚
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == PHOTO_SENSOR_Pin) {
        PhotoSensor_ISR();
    }
}

/**
  * @brief  UART接收中断回调
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t rs485_rx_byte;
    static uint8_t servo_rx_byte;
    
    if (huart == &huart2) {  // RS485
        RS485_Slave_UART_RxCallback(rs485_rx_byte);
        // 重新启动接收
        HAL_UART_Receive_IT(&huart2, &rs485_rx_byte, 1);
    } else if (huart == &huart3) {  // 舵机
        ServoControl_UART_RxCallback(servo_rx_byte);
        // 重新启动接收
        HAL_UART_Receive_IT(&huart3, &servo_rx_byte, 1);
    }
}

/**
  * @brief  UART发送完成中断回调
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {  // RS485
        RS485_Slave_UART_TxCompleteCallback();
    }
}

/**
  * @brief  UART错误中断回调
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {  // RS485
        RS485_Slave_UART_ErrorCallback();
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
#ifdef USE_FULL_ASSERT
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
