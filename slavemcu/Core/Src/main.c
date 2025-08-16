/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Door control state machine test
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
#include "../Modules/photo_sensor/photo_sensor.h"
#include "../Modules/RS485/rs485.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 命令定义 - 从rs485_protocol.h */
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

/* USER CODE BEGIN PV */
/* 光电传感器在轿厢上，每层有突起物标记 */
uint8_t current_floor = 1;      // 当前楼层
uint8_t moving_direction = 0;   // 0=停止, 1=上行, 2=下行
uint32_t last_trigger_time = 0;
bool initial_trigger = true;    // 初始触发状态
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SendPhotoSensorEvent(uint8_t floor);
static void RS485_PacketCallback(uint8_t *data, uint16_t length);
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
  
  printf("\r\n=== Elevator Photo Sensor - Slave MCU ===\r\n");
  printf("Fixed sequence support: 1->2->3->2->1\r\n\r\n");
  
  /* 初始化RS485通信 */
  printf("Initializing RS485...\r\n");
  rs485_init();
  rs485_set_packet_callback(RS485_PacketCallback);
  
  /* 初始化光电传感器 */
  printf("Initializing photo sensor...\r\n");
  PhotoSensor_Init();
  
  printf("System ready. Floor 1, sensor %s\r\n\r\n",
         PhotoSensor_GetState() == PHOTO_SENSOR_BLOCKED ? "TRIGGERED" : "CLEAR");
  
  /* 如果初始就触发，发送给主控 */
  if (PhotoSensor_GetState() == PHOTO_SENSOR_BLOCKED) {
    SendPhotoSensorEvent(1);
    initial_trigger = false;  // 已处理初始触发
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  photo_sensor_state_t last_sensor_state = PhotoSensor_GetState();
  uint32_t status_print_time = 0;
  
  printf("Monitoring photo sensor...\r\n\r\n");
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 检测光电传感器状态 */
    photo_sensor_state_t sensor_state = PhotoSensor_GetState();
    
    /* 检测光电传感器从未触发到触发的边沿 */
    if (sensor_state == PHOTO_SENSOR_BLOCKED && last_sensor_state == PHOTO_SENSOR_CLEAR) {
      /* 防抖：确保两次触发间隔大于500ms */
      if (current_time - last_trigger_time > 500) {
        last_trigger_time = current_time;
        
        /* 如果是初始触发（已在初始化处理），跳过 */
        if (initial_trigger) {
          initial_trigger = false;
          printf("Initial trigger at floor 1 (already sent)\r\n");
        } else {
          /* 根据方向更新楼层 */
          if (moving_direction == 1) {  // 上行
            if (current_floor < 3) {
              current_floor++;
            }
          } else if (moving_direction == 2) {  // 下行
            if (current_floor > 1) {
              current_floor--;
            }
          }
          /* 如果方向=0（停止），保持当前楼层 */
          
          printf("*** Photo sensor triggered! Floor=%d, Dir=%s ***\r\n", 
                 current_floor,
                 moving_direction == 1 ? "UP" : 
                 moving_direction == 2 ? "DOWN" : "STOP");
          
          /* 发送楼层信息给主控 */
          SendPhotoSensorEvent(current_floor);
        }
      }
    }
    last_sensor_state = sensor_state;
    
    /* 处理RS485接收 - 主要是方向命令 */
    uint8_t rx_buffer[64];
    uint16_t rx_len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
    if (rx_len > 0) {
      /* 使用原有的回调处理 */
      RS485_PacketCallback(rx_buffer, rx_len);
    }
    
    /* 定期打印状态（每5秒） */
    if (current_time - status_print_time >= 5000) {
      status_print_time = current_time;
      printf("Status: Floor=%d, Dir=%s, Sensor=%s\r\n", 
             current_floor,
             moving_direction == 1 ? "UP" : 
             moving_direction == 2 ? "DOWN" : "STOP",
             sensor_state == PHOTO_SENSOR_BLOCKED ? "BLOCKED" : "CLEAR");
    }
    
    /* 暂时注释掉光电传感器逻辑 */
    #if 0
    if (sensor_state == PHOTO_SENSOR_BLOCKED && last_sensor_state == PHOTO_SENSOR_CLEAR) {
      // 防抖：确保两次触发间隔大于500ms
      if (current_time - last_trigger_time > 500) {
        last_trigger_time = current_time;
        
        // 如果是初始触发（已在初始化处理），跳过
        if (initial_trigger) {
          initial_trigger = false;
          printf("Initial trigger at floor 1 (already sent)\r\n");
        } else {
          // 根据方向更新楼层
          if (moving_direction == 1) {  // 上行
            if (current_floor < 3) {
              current_floor++;
            }
          } else if (moving_direction == 2) {  // 下行
            if (current_floor > 1) {
              current_floor--;
            }
          }
          // 如果方向=0（停止），保持当前楼层
          
          printf("*** Photo sensor triggered! Floor=%d, Dir=%s ***\r\n", 
                 current_floor,
                 moving_direction == 1 ? "UP" : 
                 moving_direction == 2 ? "DOWN" : "STOP");
          
          // 发送楼层信息给主控
          SendPhotoSensorEvent(current_floor);
        }
      }
    }
    last_sensor_state = sensor_state;
    #endif
    
    /* 定期打印状态（每5秒） */
    #if 0
    if (current_time - status_print_time >= 5000) {
      status_print_time = current_time;
      printf("Status: Floor=%d, Dir=%s, Sensor=%s\r\n", 
             current_floor,
             moving_direction == 1 ? "UP" : 
             moving_direction == 2 ? "DOWN" : "STOP",
             sensor_state == PHOTO_SENSOR_BLOCKED ? "BLOCKED" : "CLEAR");
    }
    #endif
    
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

// UART发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* Call RS485 TX complete callback */
        rs485_tx_complete_callback();
    }
}

// UART接收完成回调 (not used when RS485 module is active)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* Handled by RS485 module with DMA circular mode */
    }
}

// UART错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        printf("UART2 Error: 0x%08lX\r\n", huart->ErrorCode);
        // RS485模块会自动处理错误恢复
    }
}

/**
  * @brief  发送光电传感器触发事件到主控
  * @param  floor: 检测到的楼层 (1-3)
  */
static void SendPhotoSensorEvent(uint8_t floor)
{
  uint8_t tx_buffer[4];
  
  /* 构建简单数据包 */
  tx_buffer[0] = CMD_PHOTO_SENSOR;  // 命令类型
  tx_buffer[1] = floor;              // 楼层号
  tx_buffer[2] = 0;                  // 保留
  tx_buffer[3] = 0;                  // 保留
  
  /* 通过RS485发送 */
  rs485_status_t status = rs485_send_packet_dma(tx_buffer, 4);
  
  if (status == RS485_OK) {
    printf("Sent to master: Floor %d\r\n", floor);
  } else {
    printf("Failed to send to master\r\n");
  }
}

/**
  * @brief  光电传感器触发回调（可选）
  */
void PhotoSensor_TriggerCallback(void)
{
  /* 中断回调，实际处理在主循环中进行 */
}

/**
  * @brief  RS485数据包回调
  */
static void RS485_PacketCallback(uint8_t *data, uint16_t length)
{
  if (length < 1) return;
  
  uint8_t cmd = data[0];
  
  switch (cmd) {
    case CMD_DIRECTION_SET:
      if (length >= 2) {
        moving_direction = data[1];  // 0=停止, 1=上行, 2=下行
        printf("Direction set to: %s\r\n", 
               moving_direction == 1 ? "UP" : 
               moving_direction == 2 ? "DOWN" : "STOP");
      }
      break;
      
    default:
      printf("Unknown command: 0x%02X\r\n", cmd);
      break;
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