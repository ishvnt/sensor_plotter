/* WebSocket Echo Server Example

  This example code is in the Public Domain (or CC0 licensed, at your option.)

  Unless required by applicable law or agreed to in writing, this
  software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
  CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "cJSON.h"

#define UART_BAUD_RATE 115200
#define UART_RX_PIN 16
#define UART_TX_PIN 17
#define UART_PORT_NUM 2
#define BUF_SIZE (1024)

static const char *TAG = "sensor_plotter";

uint8_t msg[2000];

cJSON* json_root;
cJSON* json_imu;
cJSON* json_acceleration;
cJSON* json_gyroscope;
cJSON* json_joystick;

TaskHandle_t uart_task_handle;
TaskHandle_t ws_task_handle;

static void init_json_structure(void)
{
    json_root = cJSON_CreateObject();
    json_imu = cJSON_CreateObject();
    json_acceleration = cJSON_CreateObject();
    json_gyroscope = cJSON_CreateObject();
    json_joystick = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_acceleration, "acc_x", 0.00);
    cJSON_AddNumberToObject(json_acceleration, "acc_y", 0.00);
    cJSON_AddNumberToObject(json_acceleration, "acc_z", 0.00);
    cJSON_AddItemToObject(json_imu, "acceleration", json_acceleration);
    cJSON_AddNumberToObject(json_gyroscope, "gyro_x", 0.00);
    cJSON_AddNumberToObject(json_gyroscope, "gyro_y", 0.00);
    cJSON_AddNumberToObject(json_gyroscope, "gyro_z", 0.00);
    cJSON_AddItemToObject(json_imu, "gyroscope", json_gyroscope);
    cJSON_AddItemToObject(json_root, "imu", json_imu);
    
    cJSON_AddNumberToObject(json_joystick, "joy_x", 0);
    cJSON_AddNumberToObject(json_joystick, "joy_y", 0);
    cJSON_AddItemToObject(json_root, "joystick", json_joystick);
}

static void parse_imu_data(uint8_t *str)
{
    char *token;
    int len = strlen((const char *)str);
    char data[len + 1];
    memcpy(data, str, len);
    data[len] = '\0';
    char *start = strstr((const char *)data, "IMU: ");
    if(start)
    {
        start += 5;
        token = strtok(start, ", ");
        int i = 0;
        for(i = 0; i < 6 && token != NULL; i++, token = strtok(NULL, ", "))
        {
            float cur_data = strtof(token, NULL);
            sprintf((char*) msg, "%f", cur_data);
            switch (i)
            {
            case 0:
                cJSON_ReplaceItemInObject(json_acceleration, "acc_x", cJSON_CreateRaw((const char*) msg));
                break;
            case 1:
                cJSON_ReplaceItemInObject(json_acceleration, "acc_y", cJSON_CreateRaw((const char*) msg));
                break;
            case 2:
                cJSON_ReplaceItemInObject(json_acceleration, "acc_z", cJSON_CreateRaw((const char*) msg));
                cJSON_ReplaceItemInObject(json_imu, "acceleration", json_acceleration);
                break;
            case 3:
                cJSON_ReplaceItemInObject(json_gyroscope, "gyro_x", cJSON_CreateRaw((const char*) msg));
                break;
            case 4:
                cJSON_ReplaceItemInObject(json_gyroscope, "gyro_y", cJSON_CreateRaw((const char*) msg));
                break;
            case 5:
                cJSON_ReplaceItemInObject(json_gyroscope, "gyro_z", cJSON_CreateRaw((const char*) msg));
                cJSON_ReplaceItemInObject(json_imu, "gyroscope", json_gyroscope);
                cJSON_ReplaceItemInObject(json_root, "imu", json_imu);
                break;
            default:
                break;
            }
        }
    }
}
static void parse_joy_data(uint8_t *str)
{
    char *token;
    int len = strlen((const char *)str);
    char data[len + 1];
    memcpy(data, str, len);
    data[len] = '\0';
    char *start = strstr((const char *)data, "JOY: ");
    if(start)
    {
        start += 5;
        token = strtok(start, ", ");
        int i = 0;
        for(i = 0; i < 2 && token != NULL; i++, token = strtok(NULL, ", "))
        {
            uint16_t cur_data = (uint16_t)strtol(token, NULL, 10);
            sprintf((char*) msg, "%u", cur_data);
            switch(i)
            {
            case 0:
                cJSON_ReplaceItemInObject(json_joystick, "joy_x", cJSON_CreateRaw((const char*) msg));
                break;
            case 1:
                cJSON_ReplaceItemInObject(json_joystick, "joy_y", cJSON_CreateRaw((const char*) msg));
                cJSON_ReplaceItemInObject(json_root, "joystick", json_joystick);
                break;
            default:
                break;
            }
        }
    }
}

static void uart_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    init_json_structure();
    while (1)
    {
        uart_read_bytes(UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        parse_imu_data(data);
        parse_joy_data(data);
        cJSON_PrintPreallocated(json_root, (char*) msg, 2000, 1);
        ESP_LOGI(TAG, "%s", msg);
        if (ws_task_handle)
            xTaskNotify(ws_task_handle, 0, eNoAction);
    }
}

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
};

static void ws_task(void *params)
{
    struct async_resp_arg *arg = (struct async_resp_arg *)params;
    httpd_handle_t handle = arg->hd;
    int socket = arg->fd;
    while (1)
    {
        httpd_ws_frame_t frame = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = msg,
            .len = strlen((const char *)msg),
            .final = true};
        esp_err_t ret = httpd_ws_send_data(handle, socket, &frame);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Send failed: %d", ret);
        }
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    }
    free(arg);
    ESP_LOGI(TAG, "Task finished and memory freed.");
    vTaskDelete(NULL);
}

static esp_err_t req_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    if (req->method == HTTP_GET)
    {
        struct async_resp_arg *params = calloc(1, sizeof(struct async_resp_arg));
        if (params == NULL)
        {
            ESP_LOGE(TAG, "Calloc failed\n");
            return ESP_ERR_NO_MEM;
        }
        params->fd = httpd_req_to_sockfd(req);
        params->hd = req->handle;
        xTaskCreate(ws_task, "broadcast", 4096, params, 5, &ws_task_handle);
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    return ret;
}

static const httpd_uri_t ws = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = req_handler,
    .user_ctx = NULL,
    .is_websocket = true};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Registering the ws handler
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK)
        {
            *server = NULL;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void app_main(void)
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    server = start_webserver();
    if (server == NULL)
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
    else
    {
        ESP_LOGI(TAG, "HTTP server started successfully");
    }
    xTaskCreate(uart_task, "UART_Task", 3072, NULL, 10, &uart_task_handle);
}
