#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_2
#define DEVICE_NAME "ESP32_BT_FreeRTOS"

static const char *TAG = "BT_APP";
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static uint32_t spp_handle = 0;
static char command[32] = {0};

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP iniciado, habilitando Bluetooth clásico...");
        esp_bt_dev_set_device_name(DEVICE_NAME);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "ESP32_SPP_SERVER");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "Cliente conectado");
        spp_handle = param->srv_open.handle;
        break;

    case ESP_SPP_DATA_IND_EVT:
        if (param->data_ind.len < sizeof(command))
        {
            memcpy(command, param->data_ind.data, param->data_ind.len);
            command[param->data_ind.len] = '\0';
            // Eliminar saltos de línea y espacios
            for (int i = strlen(command) - 1; i >= 0; i--)
            {
                if (command[i] == '\n' || command[i] == '\r' || command[i] == ' ')
                    command[i] = '\0';
                else
                    break;
            }
            ESP_LOGI(TAG, "Comando recibido: %s", command);
        }
        break;

    default:
        break;
    }
}

void task_led(void *pvParameters)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1)
    {
        if (strcmp(command, "ON") == 0)
        {
            gpio_set_level(LED_PIN, 1);
        }
        else if (strcmp(command, "OFF") == 0)
        {
            gpio_set_level(LED_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // Inicializar Bluetooth clásico
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret)
        ESP_LOGW(TAG, "No se pudo liberar memoria BLE: %s", esp_err_to_name(ret));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "Fallo al inicializar controlador BT: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret)
    {
        ESP_LOGE(TAG, "Fallo al habilitar controlador BT: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "Fallo al inicializar bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "Fallo al habilitar bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_spp_register_callback(spp_callback);
    if (ret)
    {
        ESP_LOGE(TAG, "Fallo al registrar callback SPP: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_spp_init(esp_spp_mode);
    if (ret)
    {
        ESP_LOGE(TAG, "Fallo al inicializar SPP: %s", esp_err_to_name(ret));
        return;
    }

    // Crear tarea del LED
    xTaskCreate(task_led, "task_led", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "Bluetooth listo. Conecta con 'Serial Bluetooth Terminal'.");
}
