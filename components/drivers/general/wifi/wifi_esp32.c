#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE  "WIFI_UDP"
#include "debug_cf.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif
#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_utils.h"

#define UDP_SERVER_PORT         2390
#define UDP_SERVER_BUFSIZE      64

static struct sockaddr_storage source_addr;

static char WIFI_SSID[32] = "";
static char WIFI_PWD[64] = CONFIG_WIFI_PASSWORD;
static uint8_t WIFI_CH = CONFIG_WIFI_CHANNEL;
#define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static int sock;
static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;

static esp_err_t udp_server_create(void *arg);

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(c++);
    }

    return cksum;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base != WIFI_EVENT) {
        return;
    }

    switch (event_id) {
    case WIFI_EVENT_STA_START:
        /* esp_wifi_start() only brings the interface up. Association has to be
         * asked for explicitly, and this is the earliest point it is legal. */
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_CONNECTED: {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
        DEBUG_PRINT_LOCAL("associated with %.*s on channel %d",
                          event->ssid_len, event->ssid, event->channel);
        break;
    }

    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        DEBUG_PRINT_LOCAL("disconnected, reason=%d, retrying", event->reason);
        /* Back off before retrying. Reconnecting immediately from inside the
         * event handler hammers the AP and makes handshake timeouts worse
         * rather than better. */
        vTaskDelay(M2T(500));
        esp_wifi_connect();
        break;
    }

    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        DEBUG_PRINT_LOCAL("got ip " IPSTR ", CRTP UDP server on port %d",
                          IP2STR(&event->ip_info.ip), UDP_SERVER_PORT);
    }
}

bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket *in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(M2T(10));
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t *data)
{
    UDPPacket outStage = {0};
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

static esp_err_t udp_server_create(void *arg)
{
    if (isUDPInit){
        return ESP_OK;
    }

    static struct sockaddr_in dest_addr = {0};
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    DEBUG_PRINT_LOCAL("Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
    }
    DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

    isUDPInit = true;
    return ESP_OK;
}

static void udp_server_rx_task(void *pvParameters)
{
    socklen_t socklen = sizeof(source_addr);
    char rx_buffer[UDP_SERVER_BUFSIZE];
    UDPPacket inPacket = {0};

    while (true) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
        /* command step - receive  01 from Wi-Fi UDP */
        if (len < 0) {
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            continue;
        } else if(len > WIFI_RX_TX_PACKET_SIZE) {
            DEBUG_PRINT_LOCAL("Received data length = %d > %d", len, WIFI_RX_TX_PACKET_SIZE);
            continue;
        } else {
            uint8_t cksum = rx_buffer[len - 1];
            //remove cksum, do not belong to CRTP
            //check packet
            if (cksum == calculate_cksum(rx_buffer, len - 1)) {
                //copy part of the UDP packet, the size not include cksum
                inPacket.size = len - 1;
                memcpy(inPacket.data, rx_buffer, inPacket.size);
                xQueueSend(udpDataRx, &inPacket, M2T(10));
                if(!isUDPConnected) isUDPConnected = true;
            }else{
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
            }

#ifdef DEBUG_UDP
            printf("\nReceived size = %d cksum = %02X\n", inPacket.size, cksum);
            for (size_t i = 0; i < inPacket.size; i++) {
                printf("%02X ", inPacket.data[i]);
            }
            printf("\n");
#endif
        }
    }
}

static void udp_server_tx_task(void *pvParameters)
{
    UDPPacket outPacket = {0};
    while (TRUE) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        if ((xQueueReceive(udpDataTx, &outPacket, portMAX_DELAY) == pdTRUE) && isUDPConnected) {
            // append cksum to the packet
            outPacket.data[outPacket.size] = calculate_cksum(outPacket.data, outPacket.size);
            outPacket.size += 1;

            int err = sendto(sock, outPacket.data, outPacket.size, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            if (err < 0) {
                DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
                continue;
            }
#ifdef DEBUG_UDP
            printf("\nSend size = %d checksum = %02X\n", outPacket.size, outPacket.data[outPacket.size - 1]);
            for (size_t i = 0; i < outPacket.size; i++) {
                printf("%02X ", outPacket.data[i]);
            }
            printf("\n");
#endif
        }
    }
}

static void espnow_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                                       espnow_attribute_t responder_attribute,
                                       uint32_t status1,
                                       int status2,
                                       int lx_value,
                                       int ly_value,
                                       int rx_value,
                                       int ry_value,
                                       int channel_one_value,
                                       int channel_two_value)
{
    UDPPacket inPacket;
    inPacket.size = 7;
    inPacket.data[0] = 'n';
    inPacket.data[1] = 'o';
    inPacket.data[2] = 'w';
    inPacket.data[3] = lx_value & 0xFF;
    inPacket.data[4] = ly_value & 0xFF;
    inPacket.data[5] = ry_value & 0xFF;
    inPacket.data[6] = rx_value & 0xFF;
    xQueueSend(udpDataRx, &inPacket, 0);
}

static void app_espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (base != ESP_EVENT_ESPNOW) {
        return;
    }

    switch (id) {
    case ESP_EVENT_ESPNOW_CTRL_BIND: {
        espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
        DEBUG_PRINT_LOCAL("bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
        break;
    }

    case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
        espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
        DEBUG_PRINT_LOCAL("unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac), info->initiator_attribute);
        break;
    }

    default:
        break;
    }
}

void wifiInit(void)
{
    if (isInit) {
        return;
    }
    // This should probably be reduced to a CRTP packet size
    udpDataRx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(16, sizeof(UDPPacket));
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);

    espnow_storage_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    uint8_t mac[6];

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &ip_event_handler,
                    NULL,
                    NULL));

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    sprintf(WIFI_SSID, "%s_%02X%02X%02X%02X%02X%02X", CONFIG_WIFI_BASE_SSID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
      .sta = {
        .ssid = CONFIG_WIFI_STA_SSID,
        .password = CONFIG_WIFI_STA_PASSWORD,
        /* Scan every channel and take the strongest BSSID. FAST_SCAN stops at
         * the first match, which on a mesh SSID is often a distant node that
         * then fails the 4-way handshake (disconnect reason 15). */
        .scan_method = WIFI_ALL_CHANNEL_SCAN,
        .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        /* channel 0 = scan all. Pinning it here stops the drone finding an
         * AP that is on any other channel. */
        .channel = 0,
      }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Modem sleep parks the radio between DTIM beacons, which adds tens of
     * milliseconds of jitter to every control packet and slows DHCP. A flight
     * control link cannot pay that; the motors dominate power draw anyway. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* No esp_wifi_set_channel() here: in station mode the channel is whatever
     * the AP we associate to is using, and forcing it prevents association. */

#ifdef CONFIG_ENABLE_ESPNOW_CTRL
    /* esp-now sweeps the radio across every channel in the country range on
     * retransmit and while the responder is binding. That is incompatible
     * with holding a station association -- see CONFIG_ENABLE_ESPNOW_CTRL. */
    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    espnow_init(&espnow_config);
    esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, app_espnow_event_handler, NULL);
    ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
    espnow_ctrl_responder_data(espnow_ctrl_data_cb);
#endif
    if (strlen(CONFIG_WIFI_STA_SSID) == 0) {
        DEBUG_PRINT_LOCAL("WIFI_STA_SSID is empty - set it via idf.py menuconfig, "
                          "the drone cannot associate without it");
    } else {
        DEBUG_PRINT_LOCAL("wifi station init complete, joining %s", CONFIG_WIFI_STA_SSID);
    }

    if (udp_server_create(NULL) == ESP_FAIL) {
        DEBUG_PRINT_LOCAL("UDP server create socket failed");
    } else {
        DEBUG_PRINT_LOCAL("UDP server create socket succeed");
    }
    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
    isInit = true;
}