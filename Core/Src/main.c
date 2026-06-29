/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_sdk.h"
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
#define GYRO_COUNT  2  // Configure number of active gyroscopes here (1 to 5)

typedef struct {
    float acc[3];
    float gyro[3];
    float angle[3];
    float temp;
    uint32_t timestamp;
} GyroData_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buf[64];
    GyroData_t data;
    volatile uint8_t updated;
} GyroDevice_t;

GyroDevice_t g_gyros[GYRO_COUNT];

// List of available UART handles in order of index (unused are set to NULL)
UART_HandleTypeDef *const g_huart_list[5] = {
    &huart2,  // Gyro 1 (USART2)
    &huart3,  // Gyro 2 (USART3)
    NULL,     // Gyro 3 (UART4 - Not enabled yet)
    NULL,     // Gyro 4 (UART5 - Not enabled yet)
    NULL      // Gyro 5 (USART6 - Not enabled yet)
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect printf to USART1 under GCC (VS Code)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 0xFFFF);
    return len;
}

// Custom Wit-Motion packet parser
void ParseGyroBuffer(GyroDevice_t *device, uint16_t len) {
    uint16_t i = 0;
    while (i + 11 <= len) {
        if (device->rx_buf[i] != 0x55) {
            i++;
            continue;
        }
        uint8_t sum = 0;
        for (uint16_t j = 0; j < 10; j++) {
            sum += device->rx_buf[i + j];
        }
        if (sum == device->rx_buf[i + 10]) {
            uint8_t *pkg = &(device->rx_buf[i]);
            uint8_t pkg_type = pkg[1];
            int16_t d0 = (int16_t)((pkg[3] << 8) | pkg[2]);
            int16_t d1 = (int16_t)((pkg[5] << 8) | pkg[4]);
            int16_t d2 = (int16_t)((pkg[7] << 8) | pkg[6]);
            int16_t d3 = (int16_t)((pkg[9] << 8) | pkg[8]);
            
            switch (pkg_type) {
                case 0x51: // Acceleration
                    device->data.acc[0] = d0 / 32768.0f * 16.0f;
                    device->data.acc[1] = d1 / 32768.0f * 16.0f;
                    device->data.acc[2] = d2 / 32768.0f * 16.0f;
                    device->data.temp   = d3 / 100.0f;
                    break;
                case 0x52: // Angular Velocity (Gyro)
                    device->data.gyro[0] = d0 / 32768.0f * 2000.0f;
                    device->data.gyro[1] = d1 / 32768.0f * 2000.0f;
                    device->data.gyro[2] = d2 / 32768.0f * 2000.0f;
                    break;
                case 0x53: // Angle
                    device->data.angle[0] = d0 / 32768.0f * 180.0f;
                    device->data.angle[1] = d1 / 32768.0f * 180.0f;
                    device->data.angle[2] = d2 / 32768.0f * 180.0f;
                    device->updated = 1;
                    break;
                default:
                    break;
            }
            i += 11;
        } else {
            i++;
        }
    }
}

// Callback for receive idle / half-cplt / cplt events
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (int i = 0; i < GYRO_COUNT; i++) {
        if (g_gyros[i].huart != NULL && huart->Instance == g_gyros[i].huart->Instance) {
            // Capture hardware timestamp
            g_gyros[i].data.timestamp = HAL_GetTick();
            
            // Parse buffer
            ParseGyroBuffer(&g_gyros[i], Size);
            
            // Restart DMA Idle reception
            HAL_UARTEx_ReceiveToIdle_DMA(g_gyros[i].huart, g_gyros[i].rx_buf, sizeof(g_gyros[i].rx_buf));
            break;
        }
    }
}

// Configurable Initialization helper
void Gyro_System_Init(void) {
    for (int i = 0; i < GYRO_COUNT; i++) {
        g_gyros[i].huart = g_huart_list[i];
        g_gyros[i].updated = 0;
        memset(&(g_gyros[i].data), 0, sizeof(GyroData_t));
        
        if (g_gyros[i].huart != NULL) {
            // Start DMA receive to idle
            HAL_UARTEx_ReceiveToIdle_DMA(g_gyros[i].huart, g_gyros[i].rx_buf, sizeof(g_gyros[i].rx_buf));
        }
    }
}

// DMA Watchdog to self-recover from hangs
void CheckAndRecoverDMA(void) {
    static uint32_t last_check = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_check > 500) {
        last_check = now;
        for (int i = 0; i < GYRO_COUNT; i++) {
            if (g_gyros[i].huart != NULL) {
                // If the UART state is READY, it means the DMA was stopped due to an error/hang
                if (g_gyros[i].huart->RxState == HAL_UART_STATE_READY) {
                    HAL_UART_AbortReceive(g_gyros[i].huart);
                    HAL_UARTEx_ReceiveToIdle_DMA(g_gyros[i].huart, g_gyros[i].rx_buf, sizeof(g_gyros[i].rx_buf));
                    printf("Warning: DMA channel %d recovered!\r\n", i);
                }
            }
        }
    }
}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Gyro_System_Init();
  printf("Multi-Gyroscope (%d channels) System Started!\r\n", GYRO_COUNT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CheckAndRecoverDMA();

    // Check and print data for each gyroscope dynamically
    for (int i = 0; i < GYRO_COUNT; i++) {
        if (g_gyros[i].updated) {
            g_gyros[i].updated = 0;
            printf("Gyro[%d] [T:%lu] Acc: %.3f %.3f %.3f | Gyro: %.3f %.3f %.3f | Angle: %.2f %.2f %.2f\r\n",
                   i, g_gyros[i].data.timestamp,
                   g_gyros[i].data.acc[0], g_gyros[i].data.acc[1], g_gyros[i].data.acc[2],
                   g_gyros[i].data.gyro[0], g_gyros[i].data.gyro[1], g_gyros[i].data.gyro[2],
                   g_gyros[i].data.angle[0], g_gyros[i].data.angle[1], g_gyros[i].data.angle[2]);
        }
    }
    HAL_Delay(1); // Sleep briefly
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/* USER CODE BEGIN 4 */

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
