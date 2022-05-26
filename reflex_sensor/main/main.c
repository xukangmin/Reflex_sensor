/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow.h"
#include "driver/gpio.h"

#define ESPNOW_MAXDELAY 512


#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_1)

#define LED_GPIO          10
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<LED_GPIO)

#define ESP_INTR_FLAG_DEFAULT 0

static const char *TAG = "espnow_example";

static xQueueHandle s_espnow_queue;

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = { 0, 0 };

static void espnow_deinit(espnow_send_param_t *send_param);

static int64_t start_time, end_time, diff_time = 0;

static esp_now_peer_info_t master_info;

static int  button_triggered_status = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    espnow_event_t evt;

    evt.id = ESPNOW_BUTTON_PRESSED;

    evt.info.button_addr = (uint32_t) arg;
    
    xQueueSendFromISR(s_espnow_queue, &evt, NULL);
}

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint8_t *sensor_data_type)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *sensor_data_type = buf->sensor_data_type;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void espnow_sensor_data_prepare(espnow_send_param_t *send_param, uint32_t milliseconds)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->sensor_data_type = SENSOR_DATA_RESULT;


    // buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    ESP_LOGI(TAG, "length=%d\n", send_param->len - sizeof(espnow_data_t));

    memcpy(buf->payload, &milliseconds, sizeof(uint32_t));


    //esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    // buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    ESP_LOGI(TAG, "length=%d\n", send_param->len - sizeof(espnow_data_t));
    esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint8_t sensor_data_type = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {

                    ESP_LOGD(TAG, "Data Sent to Mater");
                    break;
                    
                    // send_param->count--;
                    // if (send_param->count == 0) {
                    //     ESP_LOGI(TAG, "Send done");
                    //     espnow_deinit(send_param);
                    //     vTaskDelete(NULL);
                    // }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &sensor_data_type);
                
                // if (ret == ESPNOW_DATA_BROADCAST) {
                //     ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                //     /* If MAC address does not exist in peer list, add it to peer list. */
                //     if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                //         esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                //         if (peer == NULL) {
                //             ESP_LOGE(TAG, "Malloc peer information fail");
                //             espnow_deinit(send_param);
                //             vTaskDelete(NULL);
                //         }
                //         memset(peer, 0, sizeof(esp_now_peer_info_t));
                //         peer->channel = CONFIG_ESPNOW_CHANNEL;
                //         peer->ifidx = ESPNOW_WIFI_IF;
                //         peer->encrypt = true;
                //         memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                //         memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                //         ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                //         free(peer);
                //     }

                //     /* Indicates that the device has received broadcast ESPNOW data. */
                //     if (send_param->state == 0) {
                //         send_param->state = 1;
                //     }

                //     /* If receive broadcast ESPNOW data which indicates that the other device has received
                //      * broadcast ESPNOW data and the local magic number is bigger than that in the received
                //      * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                //      * ESPNOW data.
                //      */
                //     if (recv_state == 1) {
                //         /* The device which has the bigger magic number sends ESPNOW data, the other one
                //          * receives ESPNOW data.
                //          */
                //         if (send_param->unicast == false && send_param->magic >= recv_magic) {
                //     	    ESP_LOGI(TAG, "Start sending unicast data");
                //     	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                //     	    /* Start sending unicast ESPNOW data. */
                //             memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                //             espnow_data_prepare(send_param);
                //             if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                //                 ESP_LOGE(TAG, "Send error");
                //                 espnow_deinit(send_param);
                //                 vTaskDelete(NULL);
                //             }
                //             else {
                //                 send_param->broadcast = false;
                //                 send_param->unicast = true;
                //             }
                //         }
                //     }
                // }
                if (ret == ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */


                    send_param->broadcast = false;
                    
                    // 
                    if (sensor_data_type == SENSOR_DATA_REGISTER)
                    {
                        // waiting for response
                        ESP_LOGI(TAG, "SETUP Master Peer "MACSTR"", MAC2STR(recv_cb->mac_addr));



                        // set out mater as peer
                        master_info.channel = CONFIG_ESPNOW_CHANNEL;
                        master_info.ifidx = ESPNOW_WIFI_IF;
                        master_info.encrypt = true;
                        memcpy(master_info.lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(master_info.peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        
                        if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
                        {
                            esp_now_add_peer(&master_info);
                        }
                    }
                    else if (sensor_data_type == SENSOR_DATA_TRIGGER)
                    {
                        // trigger LED on
                        // waiting LED 
                        ESP_LOGI(TAG, "Received Trigger Request, start light up the LED");

                        start_time = esp_timer_get_time();

                        gpio_set_level(LED_GPIO, 1);

                        button_triggered_status = 1;

                    }
                    else if (sensor_data_type == SENSOR_DATA_RECV_CONFIRM)
                    {
                        

                    }

                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                free(recv_cb->data);
                break;
            }
            case ESPNOW_BUTTON_PRESSED:
            {
                ESP_LOGI(TAG, "Button triggered");
                if (button_triggered_status == 1)
                {
                     end_time = esp_timer_get_time();

                     diff_time = end_time - start_time; // in microseconds

                    // prepare data, send to master
                
                    uint32_t diff = (uint32_t)(diff_time / 1000);

                    memcpy(send_param->dest_mac, master_info.peer_addr, ESP_NOW_ETH_ALEN);
                    espnow_sensor_data_prepare(send_param, diff);


                    ESP_LOGI(TAG, "Send diff data to master=%d", diff);
                    /* Send the next data after the previous data is sent. */
                    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                        ESP_LOGE(TAG, "Send error");
                        espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }

                }
            



                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t espnow_init(void)
{
    espnow_send_param_t *send_param;

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_espnow_queue);
    esp_now_deinit();
}

void app_main(void)
{


    //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);



    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


   //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    espnow_init();
}
