#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"

// Configuración WiFi
#define WIFI_SSID "Hector Wifi"
#define WIFI_PASS "daniel1112"
#define MAX_RETRY 10

// Configuración del LED
#define LED_PIN GPIO_NUM_2

// Variables globales
static const char *TAG = "WEBSERVER_LED";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static volatile bool led_state = false;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Prototipo de funciones
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void wifi_init_sta(void);
static esp_err_t led_on_handler(httpd_req_t *req);
static esp_err_t led_off_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);
static httpd_handle_t start_webserver(void);
static void led_control_task(void *pvParameters);
static void webserver_task(void *pvParameters);

// Manejador de eventos WiFi
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Reintentando conexión WiFi...");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Fallo al conectar al WiFi");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP obtenida:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Inicialización WiFi
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Inicialización WiFi completada.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Conectado al AP SSID:%s", WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Fallo al conectar al SSID:%s", WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "Evento inesperado");
    }
}

// Handler para la ruta raíz "/"
static esp_err_t root_handler(httpd_req_t *req)
{
    const char *resp_str = "<!DOCTYPE html>"
                           "<html><head><title>ESP32 LED Control</title>"
                           "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                           "<style>"
                           "body{font-family:Arial;text-align:center;margin-top:50px;background:#f0f0f0;}"
                           "h1{color:#333;}"
                           ".button{display:inline-block;padding:15px 30px;font-size:20px;margin:10px;"
                           "cursor:pointer;text-decoration:none;border-radius:5px;border:none;}"
                           ".on{background-color:#4CAF50;color:white;}"
                           ".off{background-color:#f44336;color:white;}"
                           ".status{margin-top:30px;font-size:18px;padding:10px;}"
                           "</style></head><body>"
                           "<h1>Control de LED ESP32</h1>"
                           "<a href='/ON' class='button on'>Encender LED</a>"
                           "<a href='/OFF' class='button off'>Apagar LED</a>"
                           "<div class='status'>"
                           "<p>Usa las rutas:</p>"
                           "<p>/ON para encender</p>"
                           "<p>/OFF para apagar</p>"
                           "</div></body></html>";

    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler para encender el LED /ON
static esp_err_t led_on_handler(httpd_req_t *req)
{
    led_state = true;
    ESP_LOGI(TAG, "LED encendido via HTTP");

    const char *resp_str = "<!DOCTYPE html>"
                           "<html><head><title>LED ON</title>"
                           "<meta http-equiv='refresh' content='2;url=/'>"
                           "<style>body{font-family:Arial;text-align:center;margin-top:50px;"
                           "background:#4CAF50;color:white;}"
                           "h1{font-size:48px;}</style></head><body>"
                           "<h1>LED ENCENDIDO</h1>"
                           "<p>Redirigiendo...</p></body></html>";

    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler para apagar el LED /OFF
static esp_err_t led_off_handler(httpd_req_t *req)
{
    led_state = false;
    ESP_LOGI(TAG, "LED apagado via HTTP");

    const char *resp_str = "<!DOCTYPE html>"
                           "<html><head><title>LED OFF</title>"
                           "<meta http-equiv='refresh' content='2;url=/'>"
                           "<style>body{font-family:Arial;text-align:center;margin-top:50px;"
                           "background:#f44336;color:white;}"
                           "h1{font-size:48px;}</style></head><body>"
                           "<h1>LED APAGADO</h1>"
                           "<p>Redirigiendo...</p></body></html>";

    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Iniciar servidor web
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Iniciando servidor en puerto: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registrando URI handlers");

        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &root);

        httpd_uri_t led_on = {
            .uri = "/ON",
            .method = HTTP_GET,
            .handler = led_on_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &led_on);

        httpd_uri_t led_off = {
            .uri = "/OFF",
            .method = HTTP_GET,
            .handler = led_off_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &led_off);

        return server;
    }

    ESP_LOGI(TAG, "Error al iniciar servidor!");
    return NULL;
}

// Tarea 1: Control del LED
static void led_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Tarea de control de LED iniciada");

    // Configurar GPIO del LED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (1)
    {
        // Actualizar estado del LED según la variable global
        gpio_set_level(LED_PIN, led_state ? 1 : 0);

        // Log periódico del estado
        ESP_LOGI(TAG, "Estado LED: %s", led_state ? "ON" : "OFF");

        // Delay de 1 segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Tarea 2: Gestión del servidor web
static void webserver_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Tarea del servidor web iniciada");

    // Iniciar servidor
    httpd_handle_t server = start_webserver();

    if (server)
    {
        ESP_LOGI(TAG, "Servidor web activo y esperando solicitudes");
    }

    // Mantener la tarea viva
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Servidor web activo - Tareas FreeRTOS corriendo");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando aplicación ESP32 Web Server + FreeRTOS");

    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar WiFi
    ESP_LOGI(TAG, "Inicializando WiFi...");
    wifi_init_sta();

    // Crear tareas de FreeRTOS
    xTaskCreate(led_control_task, "led_control", 2048, NULL, 5, NULL);
    xTaskCreate(webserver_task, "webserver", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema iniciado. Ambas tareas FreeRTOS activas.");
}
