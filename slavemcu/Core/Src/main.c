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
#include "../Modules/servo/servo.h"
#include "../Modules/servo/door_control.h"
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
#define SERVO_ID 1  // 舵机ID
DoorControl_t door;  // 门控对象
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* 初始化舵机底层驱动 */
  servo_init(&huart3);
  HAL_Delay(100);
  
  /* 启用扭矩（速度已在servo_init中设置为最大） */
  servo_set_torque_enable(SERVO_ID, 1);
  HAL_Delay(100);
  
  printf("\r\n========================================\r\n");
  printf("    DOOR CONTROL STATE MACHINE TEST\r\n");
  printf("========================================\r\n");
  printf("Mapping:\r\n");
  printf("  - CLOSED = 222 degrees (position 2526)\r\n");
  printf("  - OPEN   = 0 degrees (position 0)\r\n");
  printf("----------------------------------------\r\n\r\n");
  
  /* 读取并显示当前位置 */
  uint16_t initial_pos = servo_get_position(SERVO_ID);
  printf("Current servo position: %u\r\n", initial_pos);
  printf("Current angle: %.1f degrees\r\n", initial_pos * 0.087890625);
  printf("\r\n");
  
  /* 初始化门控系统 */
  DoorControl_Init(&door, SERVO_ID);
  HAL_Delay(1000);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t test_phase = 0;
  uint32_t phase_start_time = HAL_GetTick();
  uint32_t status_print_time = 0;
  
  printf("\r\n=== Starting state machine test ===\r\n\r\n");
  
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    /* 更新门控状态 */
    DoorControl_Update(&door);
    
    /* 每秒打印一次状态 */
    if (current_time - status_print_time >= 1000) {
        status_print_time = current_time;
        DoorControl_PrintStatus(&door);
    }
    
    /* 测试序列：每个动作等待5秒 */
    if (current_time - phase_start_time >= 5000) {
        phase_start_time = current_time;
        test_phase++;
        
        printf("\r\n--- Test Phase %lu ---\r\n", test_phase);
        
        switch (test_phase % 4) {
            case 1:
                printf("ACTION: Opening door (moving to 0°)\r\n");
                DoorControl_Open(&door);
                break;
                
            case 2:
                printf("ACTION: Waiting at OPEN position\r\n");
                printf("You should see the door at 0 degrees\r\n");
                break;
                
            case 3:
                printf("ACTION: Closing door (moving to 222°)\r\n");
                DoorControl_Close(&door);
                break;
                
            case 0:
                printf("ACTION: Waiting at CLOSED position\r\n");
                printf("You should see the door at 222 degrees\r\n");
                break;
        }
    }
    
    /* 小延时 */
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