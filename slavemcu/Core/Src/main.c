/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Servo Max Speed Test Program
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Modules/servo/servo.h"
#include "../Modules/LED/LED.h"
#include <stdio.h>
#include <stdlib.h>  // For abs() function
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
uint8_t servo_id = 1;  // Default servo ID
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Servo_MaxSpeed_Test_Init(void);
static void Servo_MaxSpeed_Test_Loop(void);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize LED first
  LED_Init();
  LED_Flash(3);  // Flash 3 times to indicate system start
  
  printf("\n\n========================================\n");
  printf("SERVO MAX SPEED TEST PROGRAM\n");
  printf("LED Pin: PA1\n");
  printf("========================================\n");
  
  // Initialize servo for max speed test
  Servo_MaxSpeed_Test_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Servo_MaxSpeed_Test_Loop();
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

/**
  * @brief  Initialize servo for max speed testing
  * @retval None
  */
static void Servo_MaxSpeed_Test_Init(void) 
{
    printf("\nInitializing servo for max speed test...\n");
    
    // Initialize servo communication
    servo_init(&huart3);
    HAL_Delay(100);
    
    // Ping servo to check connection
    printf("Pinging servo ID %d...\n", servo_id);
    uint8_t ping_result = servo_ping(servo_id);
    if (ping_result == 1) {
        printf("Servo ID %d responded successfully!\n", servo_id);
        LED_Flash(1);  // Flash once for successful connection
    } else {
        printf("WARNING: No response from servo ID %d, continuing anyway...\n", servo_id);
        // Fast flash 5 times for connection warning
        for(int i = 0; i < 5; i++) {
            LED_Toggle();
            HAL_Delay(100);
        }
        LED_Off();
    }
    
    // Set angle limits to 0 for unlimited multi-turn control
    printf("Setting angle limits to 0 for multi-turn mode...\n");
    uint8_t min_limit[2] = {0x00, 0x00};
    uint8_t max_limit[2] = {0x00, 0x00};
    servo_write_reg(servo_id, 0x09, min_limit, 2);
    HAL_Delay(50);
    servo_write_reg(servo_id, 0x0B, max_limit, 2);
    HAL_Delay(50);
    
    // Set operation mode to position servo mode (0x21 = 0)
    printf("Setting operation mode to position servo (mode 0)...\n");
    uint8_t mode = 0;
    servo_write_reg(servo_id, 0x21, &mode, 1);
    HAL_Delay(50);
    
    // Enable torque (address 0x28)
    printf("Enabling servo torque...\n");
    servo_set_torque_enable(servo_id, 1);
    HAL_Delay(100);
    
    // Set maximum speed (32766 steps/s)
    printf("Setting servo to MAXIMUM speed (32766 steps/s = 479 RPM)...\n");
    servo_set_speed(servo_id, 32766);
    HAL_Delay(100);
    
    // Set acceleration (address 0x29)
    printf("Setting acceleration to 1000 steps/s^2...\n");
    uint8_t accel = 10;
    servo_write_reg(servo_id, 0x29, &accel, 1);
    HAL_Delay(50);
    
    // Move to center position (0) first
    printf("Moving to center position (0)...\n");
    servo_set_position(servo_id, 0);
    HAL_Delay(3000);
    
    printf("\n========================================\n");
    printf("SERVO SPECIFICATIONS:\n");
    printf("- Resolution: 4096 steps per 360 degrees\n");
    printf("- Min resolution: 0.0879 degrees/step\n");
    printf("- Max angle: 360 degrees (multi-turn supported)\n");
    printf("- Max speed set: 32766 steps/s (479 RPM)\n");
    printf("- Test rotation: 10 full turns = 40960 steps\n");
    printf("========================================\n\n");
}

/**
  * @brief  Main test loop for max speed rotation
  * @retval None
  */
static void Servo_MaxSpeed_Test_Loop(void) 
{
    static uint32_t last_test_time = 0;
    static uint8_t test_state = 0;
    static uint32_t rotation_start_time = 0;
    static int16_t start_position = 0;
    static uint32_t status_check_time = 0;
    
    // During rotation, check status every 500ms
    if ((test_state == 1 || test_state == 3 || test_state == 5) && 
        (HAL_GetTick() - status_check_time >= 500)) {
        
        // Blink LED during return to center (state 5)
        if (test_state == 5) {
            LED_Toggle();
        }
        
        uint16_t current_pos = servo_get_position(servo_id);
        uint8_t is_moving = servo_is_moving(servo_id);
        uint32_t elapsed = HAL_GetTick() - rotation_start_time;
        
        printf("  [%lu.%lu s] Pos: %d steps (%.1f deg), Moving: %s\n", 
               elapsed/1000, (elapsed%1000)/100,
               (int16_t)current_pos, 
               (int16_t)current_pos * 0.0879,
               is_moving ? "YES" : "NO");
        
        status_check_time = HAL_GetTick();
        
        // If stopped moving after 1 second, proceed to next test
        if (!is_moving && elapsed > 1000) {
            int16_t total_steps = (int16_t)current_pos - start_position;
            float total_degrees = total_steps * 0.0879;
            float total_rotations = total_degrees / 360.0;
            
            printf("  >>> Rotation completed!\n");
            printf("  >>> Total: %d steps = %.1f degrees = %.2f rotations\n", 
                   total_steps, total_degrees, total_rotations);
            printf("  >>> Duration: %.2f seconds\n", elapsed/1000.0);
            printf("  >>> Average speed: %.1f steps/s\n\n", 
                   (float)abs(total_steps) / (elapsed/1000.0));
            
            LED_Flash(2);  // Flash twice to indicate test completion
            HAL_Delay(2000);
            test_state++;
            last_test_time = HAL_GetTick();
        }
        return;
    }
    
    // Wait between major tests
    if (test_state != 0 && HAL_GetTick() - last_test_time < 2000) {
        return;
    }
    
    switch (test_state) {
        case 0:
            // Test 1: Maximum speed forward rotation
            printf("\n========================================\n");
            printf("[TEST 1] MAX SPEED FORWARD ROTATION\n");
            printf("Target: +10 full rotations (40960 steps)\n");
            printf("Expected time: ~1.25 seconds\n");
            printf("========================================\n");
            
            start_position = servo_get_position(servo_id);
            printf("Starting position: %d steps\n", start_position);
            printf("Target position: %d steps\n", start_position + 40960);
            printf("Executing...\n");
            LED_On();  // LED on during forward rotation
            
            servo_set_position(servo_id, start_position + 40960);
            rotation_start_time = HAL_GetTick();
            test_state = 1;
            break;
            
        case 1:
            // Monitoring forward rotation (handled above)
            break;
            
        case 2:
            // Test 2: Maximum speed reverse rotation
            printf("\n========================================\n");
            printf("[TEST 2] MAX SPEED REVERSE ROTATION\n");
            printf("Target: -10 full rotations (-40960 steps)\n");
            printf("Expected time: ~1.25 seconds\n");
            printf("========================================\n");
            
            start_position = servo_get_position(servo_id);
            printf("Starting position: %d steps\n", start_position);
            printf("Target position: %d steps\n", start_position - 40960);
            printf("Executing...\n");
            LED_Off();  // LED off during reverse rotation
            
            servo_set_position(servo_id, start_position - 40960);
            rotation_start_time = HAL_GetTick();
            test_state = 3;
            break;
            
        case 3:
            // Monitoring reverse rotation (handled above)
            break;
            
        case 4:
            // Test 3: Return to zero
            printf("\n========================================\n");
            printf("[TEST 3] RETURN TO CENTER (0)\n");
            printf("========================================\n");
            
            start_position = servo_get_position(servo_id);
            printf("Current position: %d steps\n", start_position);
            printf("Returning to position 0...\n");
            
            // LED will blink during return (handled in status check above)
            servo_set_position(servo_id, 0);
            rotation_start_time = HAL_GetTick();
            test_state = 5;
            break;
            
        case 5:
            // Monitoring return to center (handled above)
            break;
            
        case 6:
            // Test complete, wait and restart
            LED_Off();  // Ensure LED is off
            printf("\n========================================\n");
            printf("TEST CYCLE COMPLETE!\n");
            printf("Waiting 5 seconds before restart...\n");
            printf("========================================\n\n");
            
            HAL_Delay(5000);
            test_state = 0;
            break;
            
        default:
            test_state = 0;
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
