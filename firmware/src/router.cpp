#include <Arduino.h>
#include "msp.h"
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

//#define DEBUG_PRINT 0

#define SERIAL_BAUD 921600
#define SERIAL_INVERTED false

#define WIFI_AP_SSID    "RotorHazard ESP-NOW Router"
#define WIFI_AP_PSK     "ShouldNotConnectToThis"
#define WIFI_CHANNEL    6

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))


#if !defined(LED_PIN) && defined(LED_BUILTIN)
    #define LED_PIN LED_BUILTIN
#endif
#ifndef LED_INVERTED
    #define LED_INVERTED 0
#endif

#if defined(LED_PIN)
#define LED_INIT() pinMode(LED_PIN, OUTPUT)
#define LED_SET(STATE) digitalWrite(LED_PIN, STATE ^ LED_INVERTED);
#else
#define LED_INIT()
#define LED_SET(STATE)
#endif


static MSP msp_handler;
static uint8_t msp_tx_buffer[256];
static uint8_t serial_tx_buffer[256];

typedef struct {
    uint16_t freq;
    uint8_t addr[6];
    bool valid;
} peer_info_t;
static peer_info_t peers[16];

static String mac_addr_print(uint8_t const * const mac_addr)
{
    char macStr[18] = {0};
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
            mac_addr[5]);
    return String(macStr);
}

static bool add_peer(uint8_t const * const mac_addr, uint32_t const channel, uint8_t const node_id)
{
    if (ARRAY_SIZE(peers) <= node_id)
        return false;
#if DEBUG_PRINT
    Serial.printf("add_peer(mac: %02X,%02X,%02X,%02X,%02X,%02X , channel:%u, node_id:%u)\r\n",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
        channel, node_id);
#endif
#ifdef ARDUINO_ARCH_ESP32
    esp_now_peer_info_t peer_info = {
        .peer_addr = {0},
        .lmk = {0},
        .channel = (uint8_t)channel,
        .ifidx = ESP_IF_WIFI_AP,
        .encrypt = false,
        .priv = NULL};
    memcpy(peer_info.peer_addr, mac_addr, sizeof(peer_info.peer_addr));
    if (esp_now_add_peer(&peer_info) != ESP_OK)
#else
    if (esp_now_add_peer((u8 *)mac_addr, ESP_NOW_ROLE_COMBO, channel, NULL, 0) != 0)
#endif
    {
        // Error
        return false;
    }
    peers[node_id].valid = true;
    memcpy(peers[node_id].addr, mac_addr, 6);
    return true;
}

static void reset_peers(void)
{
#if DEBUG_PRINT
    Serial.println("reset_peers()");
#endif
#if 0
#ifdef ARDUINO_ARCH_ESP32
    esp_now_peer_info_t peer_info = {
        .peer_addr = {0},
        .lmk = {0},
        .channel = 0,
        .ifidx = ESP_IF_WIFI_AP,
        .encrypt = 0,
        .priv = NULL
    };
    esp_now_peer_num_t count = {0};
    if (ESP_OK == esp_now_get_peer_num(&count) && count.total_num) {
#if DEBUG_PRINT
        Serial.printf("  * remove peers (%u)... \r\n", count.total_num);
#endif
        for (uint8_t iter = 0; iter < count.total_num; iter++) {
            if (esp_now_fetch_peer(iter == 0, &peer_info) == ESP_OK) {
#if DEBUG_PRINT
                Serial.printf("  del mac: %02X,%02X,%02X,%02X,%02X,%02X\r\n",
                    peer_info.peer_addr[0], peer_info.peer_addr[1], peer_info.peer_addr[2],
                    peer_info.peer_addr[3], peer_info.peer_addr[4], peer_info.peer_addr[5]);
#endif
                //esp_now_del_peer(peer_info.peer_addr);
            }
        }
    }
#else
#endif
#endif
    uint8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(peers); iter++) {
        if (peers[iter].valid) {
            esp_now_del_peer(peers[iter].addr);
        }
    }
    memset(peers, 0, sizeof(peers));
}

int8_t find_peer_index(uint8_t const * const addr)
{
    uint8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(peers); iter++) {
        if (peers[iter].valid && memcmp(peers[iter].addr, addr, 6) == 0) {
            return iter;
        }
    }
    return -1;
}

static void espnow_laptimer_register_send(uint8_t const * addr, uint16_t const node_index, uint16_t const freq, uint8_t const type)
{
#if DEBUG_PRINT
    Serial.printf("register(mac: %02X,%02X,%02X,%02X,%02X,%02X , freq:%u, node_id:%u)\r\n",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
        freq, node_index);
#endif
    laptimer_register_resp_t command = {
        .subcommand = CMD_LAP_TIMER_REGISTER, .freq = freq, .node_index = node_index};
    size_t const size = MSP::bufferPacket(
        msp_tx_buffer, (mspPacketType_e)type, MSP_LAP_TIMER, 0, sizeof(command), (uint8_t *)&command);
    if (size)
        esp_now_send((uint8_t*)addr, msp_tx_buffer, size);
}

static void esp_now_recv_cb(uint8_t * mac_addr, uint8_t * data, uint8_t const data_len)
{
    static MSP esp_now_msp_rx;
    uint8_t iter;

    if (!data_len)
        return;

    Serial.print("ESPNOW RX ");
    Serial.print(mac_addr_print(mac_addr));

    esp_now_msp_rx.markPacketFree();

    for (iter = 0; iter < data_len; iter++) {
        if (esp_now_msp_rx.processReceivedByte(data[iter])) {
            //  MSP received, check content
            mspPacket_t & packet = esp_now_msp_rx.getPacket();
            if (packet.function == MSP_LAP_TIMER) {
                laptimer_messages_t const * const p_msg = (laptimer_messages_t *)packet.payload;
                Serial.print(" !! - Laptimer message: ");
                if (p_msg->subcommand == CMD_LAP_TIMER_REGISTER) {
                    Serial.print("LAP_TIMER_REGISTER");
                    if (packet.type == MSP_PACKET_V2_COMMAND) {
                        // Check if the client is on on current heat and send resp
                        int8_t const node_index = find_peer_index(mac_addr);
                        if (0 <= node_index) {
                            espnow_laptimer_register_send(
                                mac_addr, node_index, peers[node_index].freq, MSP_PACKET_V2_RESPONSE);
                        }
                        Serial.print(" - registeration OK");
                    } else {
                        Serial.print("MSP_RESP -> IGNORE!");
                    }
                } else {
                    Serial.print("Unsupported subcommand: ");
                    Serial.print(p_msg->subcommand);
                }
            }
            esp_now_msp_rx.markPacketFree();
        }
    }

    Serial.println();
}

static void esp_now_send_cb(
#ifdef ARDUINO_ARCH_ESP32
        uint8_t const * mac_addr, esp_now_send_status_t status
#else
        uint8_t * mac_addr, uint8_t status
#endif
                           )
{
#if DEBUG_PRINT
    Serial.printf("msg sent! mac: %02X,%02X,%02X,%02X,%02X,%02X , status:%s\r\n",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], status ? "FAIL":"OK");
#endif
}

static void espnow_send_msp(mspPacket_t & msp, uint8_t const node_id)
{
    size_t const len = MSP::bufferPacket(msp_tx_buffer, &msp);
    if (len) {
        uint8_t * p_addr = NULL;
        if (!(msp.flags & 1) && node_id < ARRAY_SIZE(peers) && peers[node_id].valid) {
            p_addr = peers[node_id].addr;
        }
        msp.flags = 0;
        esp_now_send(p_addr, msp_tx_buffer, len);
#if DEBUG_PRINT
        if (p_addr) {
            Serial.printf("espnow_send_msp() mac: %02X,%02X,%02X,%02X,%02X,%02X , node_id:%u\r\n",
                          p_addr[0], p_addr[1], p_addr[2], p_addr[3], p_addr[4], p_addr[5], node_id);
        } else {
            Serial.printf("espnow_send_msp() broadcasted\r\n");
        }
#endif
    }
}

void setup()
{
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    LED_INIT();
    LED_SET(false);

    msp_handler.markPacketFree();

    Serial.setRxBufferSize(512);
#ifdef ARDUINO_ARCH_ESP32
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, -1, -1, SERIAL_INVERTED);
#else
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, SERIAL_INVERTED);
#endif
    delay(500);

    WiFi.disconnect(true);
#ifdef ARDUINO_ARCH_ESP32
    WiFi.setTxPower(WIFI_POWER_13dBm);
#else
    WiFi.setOutputPower(13);
#endif
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PSK, WIFI_CHANNEL);

    if (esp_now_init() != 0) {
        bool led = false;
        while (1) {
            LED_SET(led);
            led ^= 1;
            delay(500);
        }
    }

#ifdef ARDUINO_ARCH_ESP8266
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
#endif
    esp_now_register_send_cb(esp_now_send_cb);
    esp_now_register_recv_cb((esp_now_recv_cb_t)esp_now_recv_cb);

    // TODO: setup wifi AP

}


void loop()
{
    static bool led_state = true;

    int temp;
    uint8_t inChar;

    while (Serial.available()) {
        temp = Serial.read();
        if (temp < 0)
            break;

        inChar = (uint8_t)temp;

        if (msp_handler.processReceivedByte(inChar)) {
            /* Handle a received MSP message */
            mspPacket_t & msp_in = msp_handler.getPacket();

            if (msp_in.type == MSP_PACKET_V2_COMMAND ||
                msp_in.type == MSP_PACKET_V2_RESPONSE) {

                LED_SET(led_state);
                led_state ^= 1;

                if (MSP_ROUTER == msp_in.function) {
                    esp_now_router_messages_t const * const p_msg = (esp_now_router_messages_t*)msp_in.payload;
                    switch (p_msg->subcommand) {
                        case SUBCMD_ROUTER_RESET: {
                            reset_peers();
                            break;
                        }
                        case SUBCMD_ROUTER_ADD: {
                            if (add_peer(p_msg->peer_add.mac, WIFI_CHANNEL, p_msg->peer_add.node_id)) {
                                espnow_laptimer_register_send(
                                    p_msg->peer_add.mac,
                                    p_msg->peer_add.node_id,
                                    p_msg->peer_add.freq,
                                    msp_in.type);
                                peers[p_msg->peer_add.node_id].freq = p_msg->peer_add.freq;
                            }
                            break;
                        }
                        case SUBCMD_ROUTER_PING: {
                            msp_in.payloadSize = sizeof(p_msg->subcommand) + 6;
                            uint8_t * p_mac = &msp_in.payload[sizeof(p_msg->subcommand)];
                            // Get MAC address
                            //esp_wifi_get_mac((wifi_interface_t)WIFI_IF_AP, p_mac);
                            WiFi.softAPmacAddress(p_mac);
                            //WiFi.macAddress(p_mac);
                            size_t const len = MSP::bufferPacket(serial_tx_buffer, &msp_in);
                            if (len) {
                                Serial.write(serial_tx_buffer, len);
                                Serial.flush();
                            }
                            break;
                        }
                    };

                } else if (MSP_LAP_TIMER == msp_in.function) {
                    laptimer_messages_t const * const p_msg = (laptimer_messages_t*)msp_in.payload;
                    uint8_t node_id = 0xff;
                    if (CMD_LAP_TIMER_LAP == p_msg->subcommand) {
                        node_id = p_msg->lap.node_index;
                    }
                    espnow_send_msp(msp_in, node_id);
                }
            }

            msp_handler.markPacketFree();
        }
    }
}
