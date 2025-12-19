#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "driver/uart.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "../components/BME280/bme280.h"
#include "../components/BME280/bme280_support.c"
#include "hal/uart_types.h"
#include "portmacro.h"
#include "sdkconfig.h"

#define TAG_I2C    "I2C"
#define TAG_BME280 "BME280"
#define TAG_SMV30 "SMV30"
#define TAG_FONA "FONA"
#define TAG_ESPNOW "ESPNOW"
#define TAG_SLEEP "SLEEP"

#define I2C_BME280_ADDRESS  0x76 
#define BME_OK 0

#define SGP30_SENSOR_ADDR                   0x58
#define SGP30_INIT_AIR_QUALITY_CMD          {0x20, 0x03}
#define SGP30_INIT_AIR_QUALITY_DURATION     10 /* 10 ms max duration */
#define SGP_30_MEASURE_AIR_QUALITY_CMD      {0x20, 0x08}
#define SGP_30_MEASURE_AIR_QUALITY_DURATION 15 /* 12 ms max duration */

#define RX_BUF_SIZE 512
#define TX_BUF_SIZE 512
#define FONA_OK "\r\nOK\r\n"

#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#define SECONDS_IN_US 1000000
#define US2SECONDS(x) (x / SECONDS_IN_US)

static i2c_master_dev_handle_t bme_handle;
static i2c_master_dev_handle_t sgp30_handle;
#if CONFIG_MASTER_ROLE
static QueueHandle_t msg_queue;
static QueueHandle_t sleep_queue;
#endif
static TaskHandle_t main_handle;

struct app_data {
    uint8_t mac[ESP_NOW_ETH_ALEN];
    float temperature;
    float humidity;
    uint16_t co2;
} __attribute__((packed));


static uint64_t time_from_boot()
{
    return US2SECONDS(esp_timer_get_time());
}


static int64_t get_remaining_receiving_time()
{
    return CONFIG_RECEIVE_INTERVAL * 1000000 - esp_timer_get_time();
}

static void i2c_master_init(i2c_master_bus_handle_t *i2c_bus)
{
    i2c_master_bus_config_t bus_config = {
        .sda_io_num = CONFIG_I2C_SDA,
        .scl_io_num = CONFIG_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, i2c_bus));
    ESP_LOGI(TAG_I2C, "I2C initialised");
}

static s8 bme280_read(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
    esp_err_t err = i2c_master_transmit(bme_handle, &reg_addr, 1, CONFIG_I2C_TIMEOUT);
    if (err != ESP_OK) return -1;

    err = i2c_master_receive(bme_handle, data, len, CONFIG_I2C_TIMEOUT);
    return (err == ESP_OK) ? 0 : -1;
}


static s8 bme280_write(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, len);

    esp_err_t err = i2c_master_transmit(bme_handle, buffer, len + 1, CONFIG_I2C_TIMEOUT);

    return (err == ESP_OK) ? 0 : -1;
}

static void bme280_delay(u32 period_ms)
{
    vTaskDelay(period_ms/portTICK_PERIOD_MS);
}

static void sgp30_init_CO2()
{  
    const uint8_t INIT_CMD[2] = SGP30_INIT_AIR_QUALITY_CMD;
    ESP_ERROR_CHECK(i2c_master_transmit(sgp30_handle, INIT_CMD, sizeof(INIT_CMD), CONFIG_I2C_TIMEOUT));
    vTaskDelay(SGP30_INIT_AIR_QUALITY_DURATION/portTICK_PERIOD_MS);
}

static uint16_t sgp30_get_CO2()
{
    const uint8_t READ_CMD[2] = SGP_30_MEASURE_AIR_QUALITY_CMD;
    uint8_t buffer[3];
    
    ESP_ERROR_CHECK(i2c_master_transmit(sgp30_handle, READ_CMD, sizeof(READ_CMD), CONFIG_I2C_TIMEOUT));
    vTaskDelay(10/portTICK_PERIOD_MS);
    i2c_master_receive(sgp30_handle, buffer, sizeof(buffer), -1);

    uint16_t co2 = (buffer[0] << 8) | buffer[1];
    uint8_t crc = buffer[2];
    ESP_LOGI(TAG_SMV30, "Data obtained: C02: %"PRIu16", CRC: %"PRIu8, co2, crc);

    return co2;
}

#if CONFIG_MASTER_ROLE
static esp_err_t uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate  = CONFIG_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_driver_install(CONFIG_UART_PORT_NUM, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG_FONA, "UART driver installed");

    err = uart_param_config(CONFIG_UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG_FONA, "UART configured");

    err = uart_set_pin(CONFIG_UART_PORT_NUM, CONFIG_UART_TXD, CONFIG_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG_FONA, "UART pins configured");

    return ESP_OK;
}

static int fona_send_command(const char *cmd, const char *expected_result)
{
    uart_flush_input(CONFIG_UART_PORT_NUM);
    uart_write_bytes(CONFIG_UART_PORT_NUM, cmd, strlen(cmd));

    char resp[100];
    int len = uart_read_bytes(CONFIG_UART_PORT_NUM, resp, 100, CONFIG_UART_RX_TIMEOUT / portTICK_PERIOD_MS);
    resp[len] = '\0';
    ESP_LOGD(TAG_FONA, "Command: %s, Resp Bytes: %d, Resp: %s", cmd, len, resp);
    
    return strcmp(resp, expected_result) == 0 ? 0 : 1;
}

static int fona_start()
{
    char *ECHO_OFF = "ATE0\r\n";
    fona_send_command(ECHO_OFF, "");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (fona_send_command(ECHO_OFF, FONA_OK) != 0) {
        ESP_LOGE(TAG_FONA, "Error turning off echo");
        return 1;
    }
    ESP_LOGI(TAG_FONA, "Echo turn off");

    char *cmd = "AT\r\n"; 
    if (fona_send_command(cmd, FONA_OK) != 0) {
        ESP_LOGE(TAG_FONA, "Error starting fona");
        return 1;
    }

    return 0;
}

static int udp_open_connection()
{
    const char *CLOSE_CONNECTIONS = "AT+CIPSHUT\r\n";
    if (fona_send_command(CLOSE_CONNECTIONS, "\r\nSHUT OK\r\n") != 0) {
        ESP_LOGE(TAG_FONA, "Error shutting down connections");
        return 1;
    }

    const char *START_SINGLE_IP_CONNECTION = "AT+CIPMUX=0\r\n";
    if (fona_send_command(START_SINGLE_IP_CONNECTION, FONA_OK) != 0) {
        ESP_LOGE(TAG_FONA, "Error starting single ip connection");
        return 1;
    }

    const char *SET_APN = "AT+CSTT=\"" CONFIG_APN "\",\"\",\"\"\r\n";
    if (fona_send_command(SET_APN, FONA_OK) != 0) {
        ESP_LOGE(TAG_FONA, "Error setting APN");
        return 1;
    }

    const char *BRING_UP_WIRELESS_CONNECTION = "AT+CIICR\r\n";
    if (fona_send_command(BRING_UP_WIRELESS_CONNECTION, FONA_OK) != 0) {
        ESP_LOGE(TAG_FONA, "Error bringing up wireless connection");
        return 1;
    }

    const char* GET_LOCAL_IP = "AT+CIFSR\r\n";
    if (fona_send_command(GET_LOCAL_IP, "\r\nERROR\r\n") == 0) {
        ESP_LOGE(TAG_FONA, "Error getting local ip");
        return 1;
    }

    const char *START_UDP_CONNECTION = "AT+CIPSTART=\"UDP\",\"" CONFIG_SERVER_IP "\"," CONFIG_SERVER_PORT "\r\n";
    if (fona_send_command(START_UDP_CONNECTION, ""FONA_OK"\r\nCONNECT OK\r\n") != 0) {
        ESP_LOGE(TAG_FONA, "Error starting udp connection");
        return 1;
    }

    return 0;
}

static int udp_close_connection()
{
    const char *CLOSE_UDP_CONNECTION = "AT+CIPCLOSE\r\n";
    if (fona_send_command(CLOSE_UDP_CONNECTION, "\r\nCLOSE OK\r\n") != 0) {
        ESP_LOGE(TAG_FONA, "Error closing udp connection");
        return 1;
    }

    return 0;
}

static int udp_send_data(const uint8_t *data, size_t len)
{
    const char *SEND_UDP_DATA_FORMAT = "AT+CIPSEND=%u\r\n";
    char send_udp_data[100];
    int command_len = snprintf(send_udp_data, 100, SEND_UDP_DATA_FORMAT, len);
    assert(command_len < 100 && command_len > 0);
    if (fona_send_command(send_udp_data, "\r\n> ") != 0) {
        ESP_LOGE(TAG_FONA, "Failed to set up data transmission");
        return 1;
    }

    uart_write_bytes(CONFIG_UART_PORT_NUM, data, len);
    const char END_MESSAGE[2] = {0x1A, 0x00};
    if (fona_send_command(END_MESSAGE, "\r\nSEND OK\r\n") != 0) {
        ESP_LOGE(TAG_FONA, "Failed to send data to fona");
        return 1;
    }
    
    return 0;
}

void fona_task(void *pvParameters)
{
    ESP_ERROR_CHECK( uart_init() );

    uint8_t num_try = 0;
    while (fona_start() != 0) {
        ESP_LOGE(TAG_FONA, "Error starting fona, try number: %"PRIu8, num_try);
        num_try++;
        if (num_try == 5) {
            esp_restart();
        }
    }
    ESP_LOGI(TAG_FONA, "Fona started");

    num_try = 0;
    while (udp_open_connection() != 0) {
        ESP_LOGE(TAG_FONA, "Error setting udp, try_number: %"PRIu8, num_try);
        num_try++;
        if (num_try == 5) {
            esp_restart();
        }
    }
    ESP_LOGI(TAG_FONA, "UDP connection established");

    struct app_data msg;

    int64_t wait_time = get_remaining_receiving_time();
    while (xQueueReceive(msg_queue, &msg, (wait_time / 1000) / portTICK_PERIOD_MS) == pdPASS) {
        ESP_LOGV(TAG_FONA, "Message\n\tmac: %02x:%02x:%02x:%02x:%02x:%02x\n\ttemperature: %lf\n\thumidity: %lf\n\tco2: %"PRIu16,
                 msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5], 
                 msg.temperature,
                 msg.humidity,
                 msg.co2);

        num_try = 0;
        while (udp_send_data((uint8_t *) &msg, sizeof(msg)) != 0) {
            ESP_LOGE(TAG_FONA, "Error sending message, try_num: %"PRIu8, num_try);
            num_try++;
            if (num_try == 5) {
                esp_restart();
            }
        }
        ESP_LOGI(TAG_FONA, "Message sent");

        wait_time = get_remaining_receiving_time();
        if (wait_time < 0) {
            wait_time = 0;
        }
    }
    ESP_LOGI(TAG_FONA, "All messages sent");

    udp_close_connection();
    xTaskNotifyGive(main_handle);
}
#endif

static esp_err_t wifi_init(void)
{
    esp_err_t err;
    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err =  esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err =  esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }

    err =  esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err =  esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    err =  esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        return err;
    }

    // Long range
    err =  esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

static void copy_master_mac(uint8_t dst[ESP_NOW_ETH_ALEN])
{
    uint64_t mac = CONFIG_MASTER_MAC;
    for (int i = ESP_NOW_ETH_ALEN - 1; i >= 0 ; i--) {
        dst[i] = (uint8_t) (mac & 0xff);
        mac >>= 8;
    }
}

#if CONFIG_MASTER_ROLE
static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    const uint8_t *sender_addr = esp_now_info->src_addr;

    if (sender_addr == NULL || data == NULL || data_len != sizeof(struct app_data)) {
        ESP_LOGE(TAG_ESPNOW, "Receive cb arg error");
        return;
    }
    
    ESP_LOGI(TAG_ESPNOW, "Receive msg from "MACSTR"", MAC2STR(sender_addr));

    if (xQueueSendToFront(sleep_queue, (void *) sender_addr, 1000 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGW(TAG_ESPNOW, "Sleep queue send failed");
    }

    if (xQueueSendToFront(msg_queue, (void *) data, 1000 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGW(TAG_ESPNOW, "Message queue send failed");
    }
}
#endif
#if CONFIG_SLAVE_ROLE
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    switch (status) {
        case ESP_NOW_SEND_FAIL:
            ESP_LOGI(TAG_ESPNOW, "Failed to send data");
            break;
        case ESP_NOW_SEND_SUCCESS:
            ESP_LOGI(TAG_ESPNOW, "Data successfully send");
            xTaskNotifyGive(main_handle);
            break;
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    const uint8_t *sender_addr = esp_now_info->src_addr;

    if (sender_addr == NULL || data == NULL || data_len <= 0) {
        ESP_LOGE(TAG_ESPNOW, "Receive cb arg error");
        return;
    }

    uint8_t master_mac[ESP_NOW_ETH_ALEN];
    copy_master_mac(master_mac);
    if (memcmp(sender_addr, master_mac, ESP_NOW_ETH_ALEN)) {
        ESP_LOGI(TAG_ESPNOW, "Not a message from master");
        return;
    }

    ESP_LOGI(TAG_ESPNOW, "Receive %d bytes from "MACSTR"", data_len, MAC2STR(sender_addr));

    uint32_t *time_to_sleep = (uint32_t *) data;
    ESP_LOGI(TAG_SLEEP, "Sleeping for %" PRIu32, *time_to_sleep);

    esp_sleep_enable_timer_wakeup(*time_to_sleep * SECONDS_IN_US);
    esp_deep_sleep_start();
}
#endif

static esp_err_t espnow_start(void)
{

    esp_err_t err;
    /* Initialize ESPNOW and register sending and receiving callback function. */
    err = esp_now_init();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        return err;
    }

    #if CONFIG_SLAVE_ROLE
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        return err;
    }
    #endif
    /* Set primary master key. */

    return ESP_OK;
}

void app_main(void)
{
    main_handle = xTaskGetCurrentTaskHandle();

    i2c_master_bus_handle_t i2c_bus;
    i2c_master_init(&i2c_bus);


    i2c_device_config_t bme280_i2c_cfg = {
        .device_address = I2C_BME280_ADDRESS,
        .scl_speed_hz = CONFIG_I2C_BME280_FREQ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &bme280_i2c_cfg, &bme_handle));
    ESP_LOGI(TAG_I2C, "BME280 device added");

    struct bme280_t bme280 = {
        .bus_write = bme280_write,
        .bus_read = bme280_read,
        .dev_addr = BME280_I2C_ADDRESS2,
        .delay_msec = bme280_delay,
    };
        
    if (bme280_init(&bme280) != 0){
        ESP_LOGE(TAG_BME280, "Init error");
        esp_restart();
    }

    ESP_LOGI(TAG_BME280, "Sensor initialised");
    if (!((bme280_set_oversamp_humidity(BME280_OVERSAMP_1X) == BME_OK) && (bme280_set_oversamp_temperature(BME280_OVERSAMP_1X) == BME_OK) &&
    (bme280_set_oversamp_pressure(BME280_OVERSAMP_1X) == BME_OK) && (bme280_set_filter(BME280_FILTER_COEFF_OFF) == BME_OK))) {
        ESP_LOGE(TAG_BME280, "Settings error");
        esp_restart();
    }

    ESP_LOGI(TAG_BME280, "Sensor configured");
    s32 un_presure, un_temperature, un_humidity;
    uint8_t error_count = 0;
    while (bme280_get_forced_uncomp_pressure_temperature_humidity(&un_presure, &un_temperature, &un_humidity) != BME_OK && error_count < 10){
        ESP_LOGI(TAG_BME280, "Get data errror, error_count=%"PRIu8, error_count);
        error_count++;
    }

    if (error_count == 10) {
        ESP_LOGE(TAG_BME280, "Error reading sensor data");
        esp_restart();
    } 

    double temperature = bme280_compensate_temperature_double(un_temperature);
    double relative_humidity = bme280_compensate_humidity_double(un_humidity);
    ESP_LOGI(TAG_BME280, "Read data T=%.2f °C RH=%.2f% %", temperature, relative_humidity);

    i2c_device_config_t sgp30_i2c_cfg = {
        .device_address = SGP30_SENSOR_ADDR,
        .scl_speed_hz = CONFIG_I2C_SGP30_FREQ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &sgp30_i2c_cfg, &sgp30_handle));
    ESP_LOGI(TAG_I2C, "SGP30 device added");

    sgp30_init_CO2();
    ESP_LOGI(TAG_SMV30, "Sensor initialised");

    // Sensor calibrating
    for (uint8_t i = 0; i < 15; i++) {
        sgp30_get_CO2();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    for (uint8_t i = 0; i < 9; i++) {
        sgp30_get_CO2();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    uint16_t co2 = sgp30_get_CO2();
    ESP_LOGI(TAG_SMV30, "Co2=%"PRIu16, co2);


    struct app_data msg;
    ESP_ERROR_CHECK( esp_efuse_mac_get_default(msg.mac) );
    msg.temperature = (float) temperature;
    msg.humidity = (float) relative_humidity;
    msg.co2 = co2;

        // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK( wifi_init() );
    ESP_ERROR_CHECK( espnow_start() );

    #if CONFIG_MASTER_ROLE
    msg_queue = xQueueCreate(128, sizeof(struct app_data));
    if (msg_queue == NULL) {
        ESP_LOGE(TAG_ESPNOW, "Create msg queue fail");
        esp_restart();
    }

    if (xTaskCreate(fona_task, "Fona task", 2048, (void *) &msg_queue, tskIDLE_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG_FONA, "Error creating fona task");
        esp_restart();
    }

    xQueueSend(msg_queue, &msg, portMAX_DELAY);

    sleep_queue = xQueueCreate(128, sizeof(uint8_t[ESP_NOW_ETH_ALEN]));
    if (sleep_queue == NULL) {
        ESP_LOGE(TAG_ESPNOW, "Create sleep queue fail");
        esp_restart();
    }

    uint8_t addr[ESP_NOW_ETH_ALEN];
    int64_t wait_time = get_remaining_receiving_time();
    while (xQueueReceive(sleep_queue, addr, (wait_time / 1000) / portTICK_PERIOD_MS) == pdPASS) {
        if (!esp_now_is_peer_exist(addr)) {
            esp_now_peer_info_t peer = {0};
            peer.channel = CONFIG_ESPNOW_CHANNEL;
            peer.ifidx = ESPNOW_WIFI_IF;
            peer.encrypt = false;
            memcpy(peer.peer_addr, addr, ESP_NOW_ETH_ALEN);
            ESP_ERROR_CHECK( esp_now_add_peer(&peer) );
        }

        uint32_t time_to_sleep = CONFIG_WAKE_PERIOD - ((uint32_t) time_from_boot());
        ESP_LOGI(TAG_ESPNOW, "sending "MACSTR" to sleep for %" PRIu32, MAC2STR(addr), time_to_sleep);
        ESP_ERROR_CHECK( esp_now_send(addr, (uint8_t *) &time_to_sleep, sizeof(time_to_sleep)) );

        wait_time = get_remaining_receiving_time();
        if (wait_time < 0) {
            wait_time = 0;
        }
    }

    esp_now_deinit();
    ESP_LOGI(TAG_ESPNOW, "ESP Now deinitialised");
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG_ESPNOW, "WiFi turned down");

    int64_t time_to_wake = CONFIG_WAKE_PERIOD * 1000000 - esp_timer_get_time();
    ulTaskNotifyTake(pdFALSE, (time_to_wake / 1000) / portTICK_PERIOD_MS);

    time_to_wake = CONFIG_WAKE_PERIOD * 1000000 - esp_timer_get_time();
    if (time_to_wake < 0) {
        time_to_wake = 0;
    }
    ESP_LOGI(TAG_SLEEP, "Sleeping for %"PRIi64" s", US2SECONDS(time_to_wake));
    esp_sleep_enable_timer_wakeup(time_to_wake);
    esp_deep_sleep_start();
    #endif
    #if CONFIG_SLAVE_ROLE
    esp_now_peer_info_t master;
    memset(&master, 0, sizeof(esp_now_peer_info_t));
    master.channel = CONFIG_ESPNOW_CHANNEL;
    master.ifidx = ESPNOW_WIFI_IF;
    master.encrypt = false;
    copy_master_mac(master.peer_addr);
    ESP_ERROR_CHECK( esp_now_add_peer(&master) );

    do {
        ESP_LOGI(TAG_ESPNOW, "sending data to "MACSTR"", MAC2STR(master.peer_addr));
        ESP_ERROR_CHECK( esp_now_send(master.peer_addr, (uint8_t *) &msg, sizeof(msg)) );
    } while (ulTaskNotifyTake(pdFALSE, 1000 / portTICK_PERIOD_MS) == 0);
    #endif
}
