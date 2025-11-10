/* USER CODE BEGIN Header */
/**
    ******************************************************************************
    * @file           : main.c
    * @brief          : Programa principal (lectura de teclado matricial)
    *
    * @details
    * Este archivo define el punto de inicio del firmware y la inicialización 
    * general para un sistema de control de sala basado en un STM32L4. El sistema 
    * lee un teclado matricial 4x4 mediante interrupciones GPIO, almacena las teclas 
    * presionadas en un buffer circular y envía los eventos detectados por UART. 
    * Cada vez que se registra una tecla válida, el LED de la placa cambia de estado.
    *
    * Módulos principales:
    * - `keypad_driver.h` : funciones para escaneo e inicialización del teclado
    * - `ring_buffer.h`   : implementación del buffer circular de teclas
    * - `led_driver.h`    : control del LED indicador
    * - `USART2`          : interfaz UART usada para debug/logs a 115200 baudios
    *
    * Secuencia general de ejecución:
    * 1. Inicialización del HAL y configuración del reloj principal
    * 2. Configuración de periféricos (GPIO, UART)
    * 3. Inicialización de módulos (LED, teclado, buffer circular)
    * 4. Bucle infinito: lectura del buffer, impresión de la tecla y cambio de LED
    *
    * Interrupciones:
    * - Las columnas del teclado se configuran como líneas EXTI por flanco de 
    *   bajada. La rutina `HAL_GPIO_EXTI_Callback` identifica la tecla presionada 
    *   mediante `keypad_scan()` y la guarda en `keypad_rb`.
    *
    * @note
    * Los nombres de pines/puertos como `KEYPAD_R1_GPIO_Port` o `LD2_Pin` 
    * provienen del proyecto CubeMX y deben coincidir con la configuración del `.ioc`.
    ******************************************************************************
    */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "keypad_driver.h"
#include "ring_buffer.h"
#include "led_driver.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/**
 * @brief Control del LED integrado en la placa
 *
 * Se usa como indicador visual, alternando su estado cada vez que se procesa 
 * un evento de tecla.
 */
led_handle_t led1 = { .port = GPIOA, .pin = GPIO_PIN_5 };

keypad_handle_t keypad = {
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port},
    .row_pins  = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port},
    .col_pins  = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}
};

#define KEYPAD_BUFFER_LEN 16
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN];
/**
 * @brief Buffer circular para almacenamiento de teclas
 *
 * Este buffer usa `keypad_buffer` como área de memoria (tamaño `KEYPAD_BUFFER_LEN`).
 * Las teclas detectadas desde la rutina EXTI se agregan aquí, permitiendo que 
 * el bucle principal las procese sin cargar la interrupción.
 */
ring_buffer_t keypad_rb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// === INTERRUPCIÓN DEL TECLADO ============================================
/**
 * @brief Callback de interrupción EXTI para GPIO
 *
 * Esta función del HAL se ejecuta cuando ocurre un evento EXTI en las columnas
 * del teclado (flanco de bajada). Llama a `keypad_scan()` para identificar la 
 * tecla presionada y, si es válida, la almacena en el buffer circular `keypad_rb`
 * para ser procesada luego por el bucle principal.
 *
 * @param GPIO_Pin Pin que generó la interrupción EXTI.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    char key = keypad_scan(&keypad, GPIO_Pin);
    if (key != '\0')
    {
        ring_buffer_write(&keypad_rb, (uint8_t)key);
    }
}

/* USER CODE END 0 */

/**
  * @brief  Función principal de la aplicación.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Configuración de la MCU --------------------------------------------------*/

  /* Reinicia todos los periféricos, inicializa la Flash y el Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configuración del reloj del sistema */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Inicialización de periféricos */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Bucle principal */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Configuración del reloj del sistema
  * @retval Ninguno
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configura el voltaje de salida del regulador interno principal */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Inicializa los osciladores RCC según los parámetros definidos */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Inicializa los relojes del CPU, AHB y APB */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Inicialización de USART2
  * @param Ninguno
  * @retval Ninguno
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief Inicialización de los pines GPIO
  * @param Ninguno
  * @retval Ninguno
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* Activar reloj de los puertos GPIO */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configurar el nivel inicial de salida de algunos pines */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|KEYPAD_R1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, KEYPAD_R4_Pin|KEYPAD_R3_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(KEYPAD_R2_GPIO_Port, KEYPAD_R2_Pin, GPIO_PIN_RESET);

  /* Configuración del pin B1 como interrupción por flanco descendente */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Pines de salida: LED, PA7 y R1 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|KEYPAD_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Pines de salida: R4 y R3 */
  GPIO_InitStruct.Pin = KEYPAD_R4_Pin|KEYPAD_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Pines de entrada con interrupción: C1 y C2 */
  GPIO_InitStruct.Pin = KEYPAD_C1_Pin|KEYPAD_C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Pin de entrada con interrupción: C4 */
  GPIO_InitStruct.Pin = KEYPAD_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C4_GPIO_Port, &GPIO_InitStruct);

  /* Pin de entrada con interrupción: C3 */
  GPIO_InitStruct.Pin = KEYPAD_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C3_GPIO_Port, &GPIO_InitStruct);

  /* Pin de salida: R2 */
  GPIO_InitStruct.Pin = KEYPAD_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KEYPAD_R2_GPIO_Port, &GPIO_InitStruct);

  /* Configuración de interrupciones EXTI */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Función que se ejecuta ante un error.
  * @retval Ninguno
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Implementación personalizada para manejar errores del HAL */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Muestra el nombre del archivo fuente y la línea donde falló assert_param.
  * @param  file: nombre del archivo fuente
  * @param  line: número de línea donde ocurrió el error
  * @retval Ninguno
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Puede añadirse una rutina personalizada, ej: printf("Error en %s, línea %d\r\n", file, line); */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
