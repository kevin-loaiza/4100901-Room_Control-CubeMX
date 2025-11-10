#include "keypad_driver.h"
#include "main.h"
#include <stdbool.h>

/* ===== Descripción general =====
 * Funciones principales:
 * - keypad_init: configura las líneas de las filas del teclado en estado inactivo (LOW).
 * - keypad_scan: realiza un barrido físico de las filas para detectar qué tecla
 *   (fila, columna) fue presionada. Implementa antirrebote y evita repeticiones de lectura.
 * - keypad_scan_index: función auxiliar que llama a keypad_scan usando el índice
 *   de la columna (0..3) en vez del número de pin.
 *
 * Consideraciones de diseño:
 * - Las columnas están conectadas con resistencias pull-up y generan una interrupción
 *   por flanco descendente cuando se oprime una tecla.
 * - La rutina de interrupción solo guarda el índice de columna; el escaneo de filas
 *   se ejecuta fuera de la ISR, en el hilo principal, para evitar sobrecargar la interrupción.
 */

static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

/**
 * @brief Configura las filas del teclado en estado inactivo (nivel bajo).
 * @param keypad Puntero al manejador que contiene los puertos y pines del teclado.
 *
 * Descripción: se mantienen las filas en nivel bajo para que, cuando una tecla
 * conecte la columna (con pull-up), se produzca un flanco descendente que active
 * la interrupción EXTI correspondiente. Esta disposición reduce el consumo y
 * permite detectar la transición de forma confiable.
 */
void keypad_init(keypad_handle_t* keypad) {
  /* Coloca todas las filas en nivel bajo (LOW), de modo que al presionar una tecla
     la columna, que está en pull-up, vea una transición descendente (falling edge)
     y dispare la interrupción. */
  for (int r = 0; r < KEYPAD_ROWS; ++r) {
    HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_RESET);
  }
}

// Se agrega antirrebote por software y prevención de lecturas duplicadas.
// - Estabilidad: la señal debe mantenerse en LOW durante KP_DEBOUNCE_MS para aceptar la lectura.
// - Supresión: tras detectar una tecla, se ignoran nuevas detecciones de esa misma
//   celda por un tiempo KP_SUPPRESS_MS.
#define KP_DEBOUNCE_MS 20
#define KP_SUPPRESS_MS 50

/**
 * @brief Realiza el escaneo de filas para identificar la tecla oprimida.
 * @param keypad Puntero al manejador del teclado.
 * @param col_pin Pin de la columna que generó la interrupción EXTI (GPIO_Pin).
 * @retval Carácter ASCII de la tecla detectada o '\0' si no se detectó ninguna.
 *
 * Funcionamiento:
 * - Activa todas las filas y luego baja una por una mientras se verifica si
 *   la columna respectiva está en LOW.
 * - Implementa antirrebote por muestreo (KP_DEBOUNCE_MS) y evita lecturas
 *   repetidas en un intervalo corto (KP_SUPPRESS_MS).
 * - Espera a que la tecla se suelte antes de devolverla, asegurando un
 *   comportamiento coherente con la versión original.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
  char key_pressed = '\0';

  /* Identificar qué columna originó la interrupción EXTI */
  int col_index = -1;
  for (int c = 0; c < KEYPAD_COLS; ++c) {
    if (keypad->col_pins[c] == col_pin) {
      col_index = c;
      break;
    }
  }
  if (col_index < 0) {
    return '\0';
  }

  /* Configurar todas las filas en HIGH antes del barrido */
  for (int r = 0; r < KEYPAD_ROWS; ++r) {
    HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET);
  }

  /* Registro estático para guardar el tiempo del último evento por tecla */
  static uint32_t last_event[KEYPAD_ROWS][KEYPAD_COLS] = { {0} };

  /* Barrido de filas: se baja una fila a la vez y se comprueba el estado de la columna */
  for (int r = 0; r < KEYPAD_ROWS; ++r) {
    /* Llevar la fila actual a LOW */
    HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_RESET);

    /* Pequeña espera para estabilizar las señales */
    HAL_Delay(1);

    GPIO_PinState state = HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]);
    if (state == GPIO_PIN_RESET) {
      /* Comprobar estabilidad: la señal debe mantenerse LOW durante KP_DEBOUNCE_MS */
      uint32_t start = HAL_GetTick();
      bool stable = true;
      while ((HAL_GetTick() - start) < KP_DEBOUNCE_MS) {
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) != GPIO_PIN_RESET) {
          stable = false;
          break;
        }
        HAL_Delay(1);
      }
      if (!stable) {
        /* Restaurar la fila y seguir con la siguiente */
        HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET);
        continue;
      }

      /* Evitar lecturas consecutivas demasiado próximas en el tiempo */
      uint32_t now = HAL_GetTick();
      if ((now - last_event[r][col_index]) < KP_SUPPRESS_MS) {
        /* Ignorar si la tecla fue leída recientemente */
        HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET);
        continue;
      }

      /* Guardar el tiempo del evento y registrar la tecla presionada */
      last_event[r][col_index] = now;
      key_pressed = keypad_map[r][col_index];

      /* Esperar a que se libere la tecla para evitar lecturas repetidas
         y mantener el mismo comportamiento del diseño base. */
      while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
        HAL_Delay(5);
      }

      /* Devolver la fila actual a HIGH antes de salir */
      HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET);
      break;
    }

    /* Si no se detecta pulsación en esta fila, restaurarla a HIGH y continuar */
    HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET);
  }

  /* Volver al estado inactivo: filas en LOW */
  keypad_init(keypad);
  return key_pressed;
}

/**
 * @brief Función auxiliar que llama a `keypad_scan` a partir del índice de columna.
 * @param keypad Puntero al manejador del teclado.
 * @param col_index Índice de columna (0..KEYPAD_COLS-1).
 * @retval Carácter detectado o '\0' si el índice no es válido.
 */
char keypad_scan_index(keypad_handle_t* keypad, uint8_t col_index) {
  if (col_index >= KEYPAD_COLS) return '\0';
  return keypad_scan(keypad, keypad->col_pins[col_index]);
}
