#include "led_driver.h"

/* Módulo de inicialización y control de LEDs
 *
 * Este conjunto de funciones ofrece una capa básica para manejar
 * una salida GPIO asociada a un LED. Su objetivo es mantener el
 * código simple, reutilizable y fácil de integrar en la aplicación principal.
 */

/**
 * @brief Coloca el pin del LED en estado bajo (apagado) al iniciar.
 * @param led Puntero a la estructura que contiene el puerto y el pin del LED.
 *
 * Nota: no se configura el GPIO desde aquí; se asume que la función
 *       MX_GPIO_Init ya estableció el modo adecuado del pin.
 */
void led_init(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief Activa el LED (pone el pin en nivel alto).
 * @param led Puntero a la estructura del LED.
 */
void led_on(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

/**
 * @brief Desactiva el LED (coloca el pin en nivel bajo).
 * @param led Puntero a la estructura del LED.
 */
void led_off(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief Cambia el estado actual del LED (ON ↔ OFF).
 * @param led Puntero a la estructura del LED.
 */
void led_toggle(led_handle_t *led) {
    HAL_GPIO_TogglePin(led->port, led->pin);
}
