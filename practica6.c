/***
  Sistemas Embebidos
  Práctica 6 - ESP32 Dual Core
  Parte 2
***/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/timer.h"



// Definición de los pines de los LEDs
#define OUTPUT_PIN   GPIO_NUM_2
#define PIN_PWM1        13          // GPIO donde sale PWM 1
#define PIN_PWM2        14          // GPIO donde sale PWM 2
#define LEDC_CH1        LEDC_CHANNEL_0  
#define LEDC_CH2        LEDC_CHANNEL_1
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE // o LEDC_LOW_SPEED_MODE según requieras
#define FREQ            1000        // Frecuencia de PWM
#define RESOLUCION_BITS LEDC_TIMER_10_BIT
#define CU1             100         // Ciclo útil (0..1023) → ~10% para 10 bits
#define CU2             900         // Ciclo útil (0..1023) → ~90% para 10 bits
volatile bool toggle = true;

// Función de interrupción por el timer
// Retornamos pdFALSE para no forzar un cambio de contexto tras la ISR
static bool IRAM_ATTR onTimerISR(void *args)
{
    toggle = !toggle; 
    return pdFALSE; 
}

// Declaración de los manejadores de las tareas
TaskHandle_t Task1;
TaskHandle_t Task2;

// Función de la primera tarea (Task1)
void Task1code(void *pvParameters) {
    // 1. Configurar el timer de LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = RESOLUCION_BITS,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Configurar canal CH1 (PWM1 - GPIO 13)
    ledc_channel_config_t ledc_channel1 = {
        .gpio_num     = PIN_PWM1,
        .speed_mode   = LEDC_MODE,
        .channel      = LEDC_CH1,
        .intr_type    = LEDC_INTR_DISABLE,
        .timer_sel    = LEDC_TIMER,
        .duty         = CU1,
        .hpoint       = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    // 3. Configurar canal CH2 (PWM2 - GPIO 14)
    ledc_channel_config_t ledc_channel2 = {
        .gpio_num     = PIN_PWM2,
        .speed_mode   = LEDC_MODE,
        .channel      = LEDC_CH2,
        .intr_type    = LEDC_INTR_DISABLE,
        .timer_sel    = LEDC_TIMER,
        .duty         = CU2,
        .hpoint       = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
    // Bucle infinito para parpadear el LED1
    while (true) {

        vTaskDelay(pdMS_TO_TICKS(50)); // Espera 1 segundo
    }
}

// Función de la segunda tarea (Task2)
void Task2code(void *pvParameters) {
// --- 1. Configuramos el pin de salida (GPIO_NUM_2) ---
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OUTPUT_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // --- 2. Configuramos el timer (Timer Group 0, Timer 0) ---
    //    - divider=80 => cada tick es 1 µs si APB es 80 MHz
    //    - auto_reload=true => al llegar a la alarma, se reinicia solo
    timer_config_t config = {
        .divider     = 80,                // preescaler
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_EN,
        .auto_reload = true
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Ponemos el contador a 0
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    // Dispara la interrupción al llegar a 1,000,000 ticks = 1 segundo (1 tick = 1 µs)
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000000);

    // Habilitamos interrupciones en Timer0 del grupo 0
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    // Registramos la función ISR
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, onTimerISR, NULL, 0);

    // Iniciamos el contador
    timer_start(TIMER_GROUP_0, TIMER_0);

    // Bucle infinito para parpadear el LED2
    while (true) {
        // Asignamos 'toggle' al pin (encendido/apagado)
        gpio_set_level(OUTPUT_PIN, toggle);

        // Pequeña pausa para ceder CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    // Inicializar UART para impresión
    printf("Iniciando el programa...\n");


    // Crear las tareas y asignarlas a los núcleos
    xTaskCreatePinnedToCore(Task1code, "Task1", 2048, NULL, 1, &Task1, 0); // Asignada al núcleo 0
    vTaskDelay(pdMS_TO_TICKS(500)); // Breve retardo entre la creación de tareas

    xTaskCreatePinnedToCore(Task2code, "Task2", 2048, NULL, 1, &Task2, 1); // Asignada al núcleo 1
    vTaskDelay(pdMS_TO_TICKS(500)); // Breve retardo entre la creación de tareas
}
