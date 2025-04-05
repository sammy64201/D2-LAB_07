/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "fatfs_sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//SPI_HandleTypeDef hspi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];
#define MAX_FILES 20
#define MAX_FILENAME_LEN 50
char filenames[MAX_FILES][MAX_FILENAME_LEN];  // Para guardar los nombres
uint8_t file_count = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void transmit_uart(char *string){
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 200);
}
void read_uart_line(char *input, uint16_t max_len) {
    char ch;
    uint16_t i = 0;

    while (i < max_len - 1) {
        HAL_UART_Receive(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

        if (ch == '\r' || ch == '\n') break;

        input[i++] = ch;
        HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY); // eco
    }

    input[i] = '\0';
    transmit_uart("\r\n");
}

void read_and_print_file(char *filename) {
    FIL fil;
    FRESULT res;

    res = f_open(&fil, filename, FA_READ);
    if (res != FR_OK) {
        transmit_uart("No se pudo abrir el archivo\r\n");
        return;
    }

    transmit_uart("Contenido del archivo:\r\n");
    char line[100];
    while (f_gets(line, sizeof(line), &fil)) {
        char formatted[110];
        snprintf(formatted, sizeof(formatted), "%s\r\n", line);  // Asegura \r\n
        transmit_uart(formatted);
    }

    f_close(&fil);
    transmit_uart("\r\nArchivo cerrado.\r\n");
}
void menu_sd(void) {
    fres = f_mount(&fs, "", 1);
    if (fres == FR_OK) {
        transmit_uart("\r\n[SD] Tarjeta montada correctamente\r\n");

        DIR dir;
        FILINFO fno;

        if (f_opendir(&dir, "/") == FR_OK) {
            transmit_uart("Archivos en la SD:\r\n");

            file_count = 0;

            while (file_count < MAX_FILES && f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
                if (!(fno.fattrib & AM_DIR)) {
                    snprintf(filenames[file_count], MAX_FILENAME_LEN, "%s", fno.fname);
                    char info[120];
                    snprintf(info, sizeof(info), "%d. %s (%lu bytes)\r\n", file_count + 1, fno.fname, (uint32_t)fno.fsize);
                    transmit_uart(info);
                    file_count++;
                }
            }

            f_closedir(&dir);

            if (file_count == 0) {
                transmit_uart("La SD no contiene archivos\r\n");
            } else {
                transmit_uart("\r\nIngrese el número del archivo que desea leer:\r\n");

                char input[10];
                read_uart_line(input, sizeof(input));
                uint8_t index = atoi(input);

                if (index > 0 && index <= file_count) {
                    read_and_print_file(filenames[index - 1]);
                } else {
                    transmit_uart("Número inválido\r\n");
                }
            }
        } else {
            transmit_uart("Error al abrir el directorio\r\n");
        }

        // Desmontar SD al final del menú
        fres = f_mount(NULL, "", 1);
        if (fres == FR_OK) {
            transmit_uart("SD desmontada correctamente\r\n");
        } else {
            transmit_uart("Error al desmontar la SD\r\n");
        }

    } else {
        transmit_uart("No se pudo montar la SD\r\n");
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(5000);
  fres = f_mount(&fs, "", 1);
  if (fres == FR_OK) {
      transmit_uart("LA SD SE MONTO :)\r\n");

      DIR dir;
      FILINFO fno;

      if (f_opendir(&dir, "/") == FR_OK) {
          transmit_uart("ARCHIVOS DE LA SD:\r\n");

          file_count = 0;

          while (file_count < MAX_FILES && f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
              if (!(fno.fattrib & AM_DIR)) {
                  snprintf(filenames[file_count], MAX_FILENAME_LEN, "%s", fno.fname);

                  char info[120];
                  snprintf(info, sizeof(info), "%d. %s (%lu bytes)\r\n", file_count + 1, fno.fname, (uint32_t)fno.fsize);
                  transmit_uart(info);

                  file_count++;
              }
          }

          f_closedir(&dir);

          if (file_count == 0) {
              transmit_uart("LA SD ESTA VACIA.\r\n");
          } else {
              transmit_uart("\r\nINGRESA EL NUMERO DEL ARCHIVO QUE QUERES VER:\r\n");

              char input[10];
              read_uart_line(input, sizeof(input));
              uint8_t index = atoi(input);

              if (index > 0 && index <= file_count) {
                  read_and_print_file(filenames[index - 1]);
              } else {
                  transmit_uart("NO ENCONTRE ESE ARCHIVO.\r\n");
              }
          }

          // DESMONTAR
          fres = f_mount(NULL, "", 1);
          if (fres == FR_OK) {
              transmit_uart("SD DESMONTADA :).\r\n");
          } else {
              transmit_uart("NO SE DESMONTO LA SD CORRECTAMENTE :(\r\n");
          }

      } else {
          transmit_uart("No se pudo abrir el directorio\r\n");
      }

  } else {
      transmit_uart("Error al montar la SD\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  transmit_uart("\r\nPresiona 'm' para abrir el menú de la SD:\r\n");

	      char input;
	      HAL_UART_Receive(&huart2, (uint8_t*)&input, 1, HAL_MAX_DELAY);

	      if (input == 'm' || input == 'M') {
	          menu_sd();
	      }// Ejecutar menú de lectura SD
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Pin */
  GPIO_InitStruct.Pin = SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
