#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Serial";

#define LED_GPIO 2
#define BLINK_PERIOD_MS 500
#define MONITOR_PERIOD_MS 1000

volatile uint32_t nivel_led = 0;

void LED_task()
{
    while (1)
    {
        gpio_set_level(LED_GPIO, nivel_led);
        nivel_led = !nivel_led;
        vTaskDelay(BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
// Tarea del consumidor
void Serial_task()
{
    int contador = 0;
    while (1)
    {
        contador++;
        ESP_LOGI(TAG, "Mensaje %d de FreeRTOS", contador);
        vTaskDelay(MONITOR_PERIOD_MS / portTICK_PERIOD_MS);
    }
}
void app_main()
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreate(LED_task, "LED_task", 2048, NULL, 1, NULL);
    xTaskCreate(Serial_task, "Serial_task", 2048, NULL, 1, NULL);
}
