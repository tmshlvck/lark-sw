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
#include "freertos/event_groups.h"

#include <sys/socket.h>
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "esp_log.h"
#include "esp_err.h"

#include "sdkconfig.h"
#include "net.h"
#include "nmea.h"


#define TAG "lark-net: "
#define STACK_SIZE 4096

extern float vario_val;


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


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
		case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
		break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			esp_wifi_connect();
		break;
		case SYSTEM_EVENT_STA_CONNECTED:
		break;
		case SYSTEM_EVENT_STA_GOT_IP:
			ESP_LOGI(TAG, "got ip:%s\n",
			ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		break;
		case SYSTEM_EVENT_AP_STACONNECTED:
			ESP_LOGI(TAG, "station:"MACSTR" join,AID=%d\n",
			MAC2STR(event->event_info.sta_connected.mac),
			event->event_info.sta_connected.aid);
		break;
		case SYSTEM_EVENT_AP_STADISCONNECTED:
			ESP_LOGI(TAG, "station:"MACSTR"leave,AID=%d\n",
			MAC2STR(event->event_info.sta_disconnected.mac),
			event->event_info.sta_disconnected.aid);
		break;
		default:
		break;
	}
	return ESP_OK;
}


static void wifi_init_softap()
{
	tcpip_adapter_init();
	/*
	tcpip_adapter_ip_info_t info = { 0, };
	IP4_ADDR(&info.ip, 192, 168, 123, 1);
	IP4_ADDR(&info.gw, 192, 168, 123, 1);
	IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
	*/
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

	ESP_LOGI(TAG, "SoftAP SSID:%s password:%s \n",
		LARK_DEFAULT_SSID, LARK_DEFAULT_PWD);
}


int client_sockets[LARK_MAX_STA_CONN];
SemaphoreHandle_t net_feed_semaphore = NULL; 
#define PKTSIZE 256
static void net_tx_task(void *pvParameters) {
	char *databuff = (char *)malloc(PKTSIZE * sizeof(char));

	while (1) {
		if (xSemaphoreTake(net_feed_semaphore, portMAX_DELAY)!= pdTRUE) {
			ESP_LOGW(TAG, "semaphore failed!\n");
			vTaskDelay(100/portTICK_PERIOD_MS);
		}

		memset(databuff, 0, PKTSIZE * sizeof(char));
		//sprintf(databuff, "%f\n", vario_val);
		Compose_Pressure_POV_fast(databuff, vario_val);

		for (int i=0; i<LARK_MAX_STA_CONN; i++) {
			if (client_sockets[i] >= 0) {
				int len = send(client_sockets[i], databuff, strlen(databuff), 0);
				if (len <= 0) {
					int err = get_socket_error_code(client_sockets[i]);
					if (err != ENOMEM)
						show_socket_error_reason("send_data", client_sockets[i]);
					close(client_sockets[i]);
					client_sockets[i]=-1;
				}
			}
		}
	}

	free(databuff);
	vTaskDelete(NULL);
}

static void net_rx_task(void *pvParameters) {
	int *socket = pvParameters;
	char *databuff = (char *)malloc(PKTSIZE * sizeof(char));

	while (1) {
		memset(databuff, 0, PKTSIZE * sizeof(char));
		int len = recv(*socket, databuff, (PKTSIZE-1), 0);
		if (len <= 0) {
			break;
		}

		printf("%s recv: %s\n", __func__, databuff);
	}

	printf("Dropping connection. Reason: Disconnect.\n");
	close(*socket);
	*socket = -1;
	vTaskDelete(NULL);
}	
	
#define RX_TASK_NAME_LEN 16

void networking_task(void *pvParameters) {
	int server_socket = 0;
	int connect_socket = 0;
	int i;
	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;
	unsigned int socklen = sizeof(client_addr);
	char rx_task_name[RX_TASK_NAME_LEN];

	wifi_init_softap();
	for (i=0; i<LARK_MAX_STA_CONN; i++) 
		client_sockets[i] = -1;

	if (pdPASS != xTaskCreate(&net_tx_task, "net_tx_task", STACK_SIZE, NULL, 4, NULL)) {
		ESP_LOGE(TAG, "Network TX task create fail!");
		vTaskDelete(NULL);
		return;
	}

	server_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket < 0) {
		show_socket_error_reason("create_server", server_socket);
		vTaskDelete(NULL);
		return;
	}
	ESP_LOGI(TAG, "server socket created on port=%d\n", NMEA_PORT);

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(NMEA_PORT);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
		show_socket_error_reason("bind_server", server_socket);
		close(server_socket);
		vTaskDelete(NULL);
		return;
	}
	if (listen(server_socket, 5) < 0) {
		show_socket_error_reason("listen_server", server_socket);
		close(server_socket);
		vTaskDelete(NULL);
		return;
	}

	while(1) {
		connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
		if (connect_socket < 0) {
			show_socket_error_reason("accept_server", connect_socket);
			close(server_socket);
			break;
		}
		for (i=0; ((i<LARK_MAX_STA_CONN)&&(client_sockets[i] >= 0)); i++);
		if (i == (LARK_MAX_STA_CONN-1)&&(client_sockets[i] > 0)) {
			ESP_LOGW(TAG, "Dropped incomming connection. Reason: Too many connections.\n");
			close(connect_socket);
			continue;
		}
		client_sockets[i] = connect_socket;

		snprintf(rx_task_name, RX_TASK_NAME_LEN, "net_rx_task_%d", i);
		if (pdPASS != xTaskCreate(&net_rx_task, rx_task_name, STACK_SIZE, &client_sockets[i], 3, NULL)) {
			ESP_LOGE(TAG, "Network RX task create fail!");
		}
	}

	close(server_socket);
	vTaskDelete(NULL);
}

