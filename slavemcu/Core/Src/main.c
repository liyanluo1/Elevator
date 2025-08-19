/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Slave MCU - 完整电梯从控系统（键盘+光电+门控）
  ******************************************************************************
  * @attention
  * 
  * 系统功能：
  * 1. 键盘输入处理（轿厢内呼）
  * 2. 光电传感器检测（楼层到达）
  * 3. 门控系统（舵机控制）
  * 4. RS485通信（与Master MCU通信）
  * 5. Local Blackboard事件管理
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Modules/keyboard/keyboard.h"
#include "../Modules/photo_sensor/photo_sensor.h"
#include "../Modules/servo/servo.h"
#include "../Modules/servo/door_control.h"

/* UART句柄声明 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#include "../Modules/Local_BB/local_blackboard.h"
#include "../Modules/RS485/rs485.h"
#include "../Modules/RS485/rs485_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FLOORS 3
#define PHOTO_SENSOR_DEBOUNCE_MS 200   // 光电防抖时间
#define KEYBOARD_DEBOUNCE_MS     50    // 按键防抖时间
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 系统状态 */
static struct {
    uint8_t current_floor;       // 当前楼层
    uint8_t direction;           // 0=停止, 1=上行, 2=下行
    uint8_t target_floor;        // 目标楼层
    bool door_is_open;           // 门状态
    uint32_t keyboard_count;     // 键盘按键计数
    uint32_t photo_count;        // 光电触发计数
    uint32_t rs485_rx_count;     // RS485接收计数
    uint32_t rs485_tx_count;     // RS485发送计数
    uint32_t door_cmd_count;     // 门控命令计数
    uint32_t last_photo_time;    // 上次光电触发时间
} system_state = {
    .current_floor = 1,
    .direction = DIR_STOP,
    .target_floor = 1,
    .door_is_open = false,
    .last_photo_time = 0
};

/* 光电传感器状态 */
volatile bool sensor_triggered = false;
static photo_sensor_state_t last_sensor_state = PHOTO_SENSOR_CLEAR;

/* 门控状态 */
static DoorControl_t door_controller;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PhotoSensor_TriggerCallback(void);
void ProcessKeyboard(void);
void ProcessPhotoSensor(void);
void ProcessRS485(void);
void ProcessDoorControl(void);
void SendPhotoSensorEvent(uint8_t floor);
void SendCabinCall(uint8_t floor);
void SendDoorStatus(bool is_open);
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  printf("\r\n=== SLAVE MCU - ELEVATOR CONTROL SYSTEM ===\r\n");
  printf("Full Control with Door Module Enabled\r\n");
  printf("Features: Cabin Calls + Photo Sensor + RS485 + Door Control\r\n\r\n");
  
  /* 初始化RS485通信 */
  rs485_init();
  printf("[RS485] Initialized on USART2\r\n");
  
  /* 初始化键盘 - 中断模式 */
  Keyboard_Init();
  printf("[KEYBOARD] Initialized (PA11 EXTI)\r\n");
  printf("  S16 (PA4)  -> Floor 1 (Cabin Call)\r\n");
  printf("  S15 (PA8)  -> Floor 2 (Cabin Call)\r\n");
  printf("  S14 (PA5)  -> Floor 3 (Cabin Call)\r\n");
  printf("  S13 (PA12) -> Not Used\r\n\r\n");
  
  /* 初始化光电传感器 - 中断模式 */
  PhotoSensor_Init();
  photo_sensor_state_t initial_state = PhotoSensor_GetState();
  last_sensor_state = initial_state;
  printf("[PHOTO] Initialized (PB5 EXTI) - State: %s\r\n", 
         initial_state == PHOTO_SENSOR_BLOCKED ? "BLOCKED" : "CLEAR");
  
  /* 验证初始状态 */
  if (initial_state != PHOTO_SENSOR_BLOCKED) {
      printf("[WARNING] Photo sensor should be BLOCKED at floor 1!\r\n");
      printf("         Please check elevator position.\r\n");
  }
  
  /* 初始化舵机门控系统 */
  DoorControl_Init(&door_controller);
  
  /* 初始化Local Blackboard - 中央事件管理 */
  LocalBB_Init();
  printf("[LocalBB] Initialized - Event queue ready\r\n");
  
  printf("\r\nSystem ready. Waiting for commands...\r\n\r\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_status_time = 0;
  uint32_t last_photo_time = 0;  // 光电防抖
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 测试模式已禁用，使用按键测试 */
    
    /* 处理键盘输入 */
    ProcessKeyboard();
    
    /* 处理光电传感器 */
    ProcessPhotoSensor();
    
    /* 处理RS485接收 */
    ProcessRS485();
    
    /* 处理门控（每50ms更新） */
    static uint32_t last_door_update = 0;
    if (current_time - last_door_update >= 50) {
        last_door_update = current_time;
        ProcessDoorControl();
    }
    
    /* 处理LocalBB事件队列 */
    LocalBB_Process();
    
    /* 每5秒打印一次状态 */
    if (current_time - last_status_time >= 5000) {
        last_status_time = current_time;
        printf("[STATUS] Floor=%d, Dir=%s, Door=%s\r\n",
               system_state.current_floor,
               system_state.direction == DIR_UP ? "UP" : 
               system_state.direction == DIR_DOWN ? "DN" : "STOP",
               system_state.door_is_open ? "OPEN" : "CLOSED");
        printf("  Keyboard=%lu, Photo=%lu, RS485_RX=%lu, TX=%lu, DoorCmd=%lu\r\n",
               system_state.keyboard_count,
               system_state.photo_count,
               system_state.rs485_rx_count,
               system_state.rs485_tx_count,
               system_state.door_cmd_count);
        LocalBB_PrintStatus();
    }
    
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
  * @brief  处理键盘输入
  */
void ProcessKeyboard(void) {
    Keyboard_Handler();
    
    uint8_t key;
    if (Keyboard_PopKey(&key)) {
        system_state.keyboard_count++;
        
        printf("\r\n========================================\r\n");
        printf("[KEYBOARD #%lu] Key detected: 0x%02X\r\n", system_state.keyboard_count, key);
        
        switch(key) {
            case KEY_S16:  // Floor 1
                printf("S16 -> Floor 1 call\r\n");
                SendCabinCall(1);
                LocalBB_AddCabinCall(1);
                break;
                
            case KEY_S15:  // Floor 2
                printf("S15 -> Floor 2 call\r\n");
                SendCabinCall(2);
                LocalBB_AddCabinCall(2);
                break;
                
            case KEY_S14:  // Floor 3
                printf("S14 -> Floor 3 call\r\n");
                SendCabinCall(3);
                LocalBB_AddCabinCall(3);
                break;
                
            case KEY_S13:  // Not used
                printf("S13 pressed (not used)\r\n");
                break;
                
            default:
                printf("Unknown key (0x%02X)\r\n", key);
                break;
        }
        printf("========================================\r\n");
    }
}

/**
  * @brief  处理光电传感器 - 增强版带防抖和智能楼层检测
  */
void ProcessPhotoSensor(void) {
    uint32_t current_time = HAL_GetTick();
    
    if (sensor_triggered) {
        sensor_triggered = false;
        
        /* 防抖处理 */
        if (current_time - system_state.last_photo_time < PHOTO_SENSOR_DEBOUNCE_MS) {
            return;  // 忽略抖动
        }
        system_state.last_photo_time = current_time;
        
        photo_sensor_state_t current_state = PhotoSensor_GetState();
        
        /* 只在状态从CLEAR变为BLOCKED时触发（到达楼层） */
        if (current_state == PHOTO_SENSOR_BLOCKED && 
            last_sensor_state == PHOTO_SENSOR_CLEAR) {
            
            system_state.photo_count++;
            
            /* 智能楼层检测 - 基于方向和当前楼层 */
            uint8_t detected_floor = system_state.current_floor;
            
            if (system_state.direction == DIR_UP) {
                /* 上行：检测到的是下一个楼层 */
                detected_floor = system_state.current_floor + 1;
                if (detected_floor > MAX_FLOORS) {
                    detected_floor = MAX_FLOORS;
                }
            }
            else if (system_state.direction == DIR_DOWN) {
                /* 下行：检测到的是下一个楼层 */
                detected_floor = system_state.current_floor - 1;
                if (detected_floor < 1) {
                    detected_floor = 1;
                }
            }
            /* 如果静止，保持当前楼层不变 */
            
            printf("[PHOTO #%lu] Floor %d detected (Dir=%s, Prev=%d)\r\n",
                   system_state.photo_count, detected_floor,
                   system_state.direction == DIR_UP ? "UP" :
                   system_state.direction == DIR_DOWN ? "DN" : "STOP",
                   system_state.current_floor);
            
            /* 立即发送光电传感器事件给Master */
            SendPhotoSensorEvent(detected_floor);
            
            /* 更新当前楼层 */
            system_state.current_floor = detected_floor;
            
            /* 如果到达目标楼层，更新状态 */
            if (detected_floor == system_state.target_floor) {
                printf("[PHOTO] Target floor %d reached!\r\n", detected_floor);
                /* Master会处理停止，Slave只报告 */
            }
            
            /* 通过LocalBB处理 */
            LocalBB_AddPhotoSensor();
        }
        
        last_sensor_state = current_state;
    }
}

/**
  * @brief  处理RS485接收
  */
void ProcessRS485(void) {
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    
    if (rx_len > 0) {
        system_state.rs485_rx_count++;
        
        /* 方向设置命令 */
        if (rx_buffer[0] == CMD_DIRECTION_SET && rx_len >= 4) {
            uint8_t dir = rx_buffer[1];
            uint8_t cur = rx_buffer[2];
            uint8_t target = rx_buffer[3];
            
            system_state.direction = dir;
            system_state.current_floor = cur;
            system_state.target_floor = target;
            
            printf("[RS485 RX] Direction=%s, F%d->F%d\r\n",
                   dir == DIR_UP ? "UP" : dir == DIR_DOWN ? "DN" : "STOP",
                   cur, target);
            
            /* 通过LocalBB处理 */
            LocalBB_SetDirection(dir, cur, target);
        }
        /* 门控制命令 */
        else if (rx_buffer[0] == CMD_DOOR_OPEN) {
            printf("[RS485 RX] Door OPEN command\r\n");
            LocalBB_AddDoorCommand(true);
            system_state.door_cmd_count++;
        }
        else if (rx_buffer[0] == CMD_DOOR_CLOSE) {
            printf("[RS485 RX] Door CLOSE command\r\n");
            LocalBB_AddDoorCommand(false);
            system_state.door_cmd_count++;
        }
        /* 状态请求 */
        else if (rx_buffer[0] == CMD_STATUS_REQUEST) {
            /* 发送状态响应 */
            uint8_t tx_buffer[4];
            tx_buffer[0] = CMD_STATUS_RESPONSE;
            tx_buffer[1] = system_state.current_floor;
            tx_buffer[2] = system_state.direction;
            tx_buffer[3] = system_state.door_is_open ? 1 : 0;
            rs485_send_packet_dma(tx_buffer, 4);
            system_state.rs485_tx_count++;
        }
    }
}

/**
  * @brief  处理门控制
  */
void ProcessDoorControl(void) {
    /* 更新门控制器状态 */
    DoorControl_Update(&door_controller);
    
    /* 处理来自LocalBB的门控命令 */
    LocalBB_DoorCommand_t door_cmd = LocalBB_GetDoorCommand();
    if (door_cmd != DOOR_CMD_NONE) {
        if (door_cmd == DOOR_CMD_OPEN) {
            printf("[DOOR] Command: OPEN\r\n");
            DoorControl_Open(&door_controller);
            system_state.door_is_open = true;
            SendDoorStatus(true);
        } else if (door_cmd == DOOR_CMD_CLOSE) {
            printf("[DOOR] Command: CLOSE\r\n");
            DoorControl_Close(&door_controller);
            system_state.door_is_open = false;
            SendDoorStatus(false);
        }
        LocalBB_ClearDoorCommand();
    }
    
    /* 检查门状态变化 */
    DoorState_t door_state = DoorControl_GetState(&door_controller);
    static DoorState_t last_door_state = DOOR_STATE_CLOSED;
    
    if (door_state != last_door_state) {
        last_door_state = door_state;
        
        switch (door_state) {
            case DOOR_STATE_OPEN:
                printf("[DOOR] Fully opened\r\n");
                system_state.door_is_open = true;
                SendDoorStatus(true);
                break;
                
            case DOOR_STATE_CLOSED:
                printf("[DOOR] Fully closed\r\n");
                system_state.door_is_open = false;
                SendDoorStatus(false);
                break;
                
            case DOOR_STATE_OPENING:
            case DOOR_STATE_CLOSING:
                // 过渡状态，不需要特殊处理
                break;
        }
    }
}

/**
  * @brief  发送光电传感器事件
  */
void SendPhotoSensorEvent(uint8_t floor) {
    uint8_t tx_buffer[4];
    tx_buffer[0] = CMD_PHOTO_SENSOR;
    tx_buffer[1] = floor;
    tx_buffer[2] = 0;
    tx_buffer[3] = 0;
    
    rs485_send_packet_dma(tx_buffer, 4);
    system_state.rs485_tx_count++;
    printf("[RS485 TX] Photo sensor floor %d\r\n", floor);
}

/**
  * @brief  发送轿厢内呼
  */
void SendCabinCall(uint8_t floor) {
    uint8_t tx_buffer[4];
    tx_buffer[0] = CMD_CABIN_CALL;
    tx_buffer[1] = floor;
    tx_buffer[2] = 0;
    tx_buffer[3] = 0;
    
    printf("[RS485 TX] Sending cabin call for floor %d\r\n", floor);
    printf("  Buffer: [0x%02X, 0x%02X, 0x%02X, 0x%02X]\r\n", 
           tx_buffer[0], tx_buffer[1], tx_buffer[2], tx_buffer[3]);
    
    rs485_status_t status = rs485_send_packet_dma(tx_buffer, 4);
    if (status == RS485_OK) {
        system_state.rs485_tx_count++;
        printf("[RS485 TX] Cabin call sent successfully (count=%lu)\r\n", system_state.rs485_tx_count);
    } else if (status == RS485_BUSY) {
        printf("[RS485 TX] ERROR: RS485 busy!\r\n");
    } else {
        printf("[RS485 TX] ERROR: Send failed (status=%d)\r\n", status);
    }
}

/**
  * @brief  发送门状态
  */
void SendDoorStatus(bool is_open) {
    uint8_t tx_buffer[4];
    tx_buffer[0] = CMD_DOOR_STATUS;
    tx_buffer[1] = is_open ? 1 : 0;
    tx_buffer[2] = 0;
    tx_buffer[3] = 0;
    
    rs485_send_packet_dma(tx_buffer, 4);
    system_state.rs485_tx_count++;
    printf("[RS485 TX] Door status %s\r\n", is_open ? "OPEN" : "CLOSED");
}

/**
  * @brief  光电传感器触发回调
  * @note   在中断中被调用
  */
void PhotoSensor_TriggerCallback(void)
{
    sensor_triggered = true;
}

/**
  * @brief  HAL GPIO EXTI回调 - 统一处理所有GPIO中断
  * @param  GPIO_Pin: 触发中断的引脚
  * @note   这个函数覆盖HAL库的weak函数
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_11) {
        // PA11 中断 - 键盘按下
        Keyboard_IRQHandler();
    }
    else if (GPIO_Pin == GPIO_PIN_5) {
        // PB5 中断 - 光电传感器
        PhotoSensor_IRQHandler();
    }
}

/**
  * @brief  UART TX DMA完成回调
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    extern void rs485_tx_complete_callback(void);
    if (huart == &huart2) {
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
