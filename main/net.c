/* Lark networking 
 * Copyright (C) 2018 Tomas Hlavacek (tomas.hlavacek@akaflieg.tu-darmstadt.de)
 *
 * This file is part of Lark.
 *
 * Lark is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Lark.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include <sys/socket.h>
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "esp_log.h"
#include "esp_err.h"

#include "sdkconfig.h"
#include "net.h"


#define TAG "lark: "

// debug
extern float dpt;


static int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}


static int show_socket_error_reason(const char *str, int socket)
{
    int err = get_socket_error_code(socket);

    if (err != 0) {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }

    return err;
}


EventGroupHandle_t tcp_event_group;
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(tcp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s\n",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(tcp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join,AID=%d\n",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        xEventGroupSetBits(tcp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave,AID=%d\n",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        xEventGroupClearBits(tcp_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}


static void wifi_init_softap()
{
    tcp_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = LARK_DEFAULT_SSID,
            .ssid_len = 0,
            .max_connection = LARK_MAX_STA_CONN,
            .password = LARK_DEFAULT_PWD,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
	    .beacon_interval = 5000
        },
    };
    if (strlen(LARK_DEFAULT_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    esp_wifi_set_protocol(0, WIFI_PROTOCOL_11B);
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s \n",
             LARK_DEFAULT_SSID, LARK_DEFAULT_PWD);
}


static int server_socket = 0;
static int connect_socket = 0;
static struct sockaddr_in server_addr;
static struct sockaddr_in client_addr;
static unsigned int socklen = sizeof(client_addr);

static esp_err_t create_tcp_server()
{
    ESP_LOGI(TAG, "server socket....port=%d\n", NMEA_PORT);
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        show_socket_error_reason("create_server", server_socket);
        return ESP_FAIL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(NMEA_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        show_socket_error_reason("bind_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    if (listen(server_socket, 5) < 0) {
        show_socket_error_reason("listen_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
    if (connect_socket < 0) {
        show_socket_error_reason("accept_server", connect_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

#define PKTSIZE 256
static void send_data(void *pvParameters) { // can be task
    char *databuff = (char *)malloc(PKTSIZE * sizeof(char));

    while (1) {
    	memset(databuff, 0, PKTSIZE * sizeof(char));
	sprintf(databuff, "%f\n", dpt);

        int len = send(connect_socket, databuff, strlen(databuff), 0);
        if (len > 0) {
	    len = 0;
        } else {
            int err = get_socket_error_code(connect_socket);
            if (err != ENOMEM) {
                show_socket_error_reason("send_data", connect_socket);
                break;
            }
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    free(databuff);
    //vTaskDelete(NULL);
}


static void close_socket()
{
    close(connect_socket);
    close(server_socket);
}


void networking_task(void *pvParameters)
{

    wifi_init_softap();

    while (1) {
        ESP_LOGI(TAG, "task tcp_conn.");
        xEventGroupWaitBits(tcp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "sta has connected to ap.");

        int socket_ret = ESP_FAIL;

        if (socket_ret == ESP_FAIL) {
            ESP_LOGI(TAG, "tcp_server will start after 3s...");
            vTaskDelay(3000 / portTICK_RATE_MS);
            ESP_LOGI(TAG, "create_tcp_server.");
            socket_ret = create_tcp_server();
        }
        if (socket_ret == ESP_FAIL) {
            ESP_LOGI(TAG, "create tcp socket error,stop.");
            continue;
        }

        /*if (tx_rx_task == NULL) {
            if (pdPASS != xTaskCreate(&send_data, "send_data", 4096, NULL, 4, &tx_rx_task)) {
                ESP_LOGE(TAG, "Send task create fail!");
            }
        } */

	send_data(NULL);

        close_socket();
    }

    vTaskDelete(NULL);
}

