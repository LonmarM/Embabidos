/***************************** NOTA *****************************
 * Las terminales de la 34 a la 39 no tienen resistencias Pull-Up
 * ni Pull-Down. Solo pueden ser configuradas como entradas y no 
 * pueden ser usadas como salidas.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"

// Definimos los pines de entrada y salida para ESP-IDF
#define PIN_IN   GPIO_NUM_4
#define PIN_OUT  GPIO_NUM_2

void app_main(void)
{
    // --- Configurar pin de salida ---
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << PIN_OUT),     // Indica qué pin se configura
        .mode          = GPIO_MODE_OUTPUT,      // Modo salida
        .pull_up_en    = GPIO_PULLUP_DISABLE,   // Sin pull-up
        .pull_down_en  = GPIO_PULLDOWN_ENABLE, // Sin pull-down
        .intr_type     = GPIO_INTR_DISABLE      // Sin interrupciones
    };
    // Aplicamos la configuración para el pin de salida
    gpio_config(&io_conf);

    // --- Configurar pin de entrada ---
    io_conf.pin_bit_mask  = (1ULL << PIN_IN);
    io_conf.mode          = GPIO_MODE_INPUT;    
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;  // Ajusta si necesitas resistencia interna pull-up
    io_conf.pull_down_en  = GPIO_PULLDOWN_ENABLE;// O pull_down según tu hardware
    io_conf.intr_type     = GPIO_INTR_DISABLE;
    // Aplicamos la configuración para el pin de entrada
    gpio_config(&io_conf);

    // Bucle principal, equivalente al 'loop()' en Arduino
    while (true) {
        // Leemos el estado del pin de entrada (0 o 1)
        int pin_state = gpio_get_level(PIN_IN);
        if(pin_state){
            gpio_set_level(PIN_OUT, 0);
        }
        else{
            gpio_set_level(PIN_OUT, 1);
        }

            
        // Pequeño retardo para no “saturar” la CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
