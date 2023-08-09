#include <Arduino.h>
#include "msp.h"
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

//#define DEBUG_PRINT 1
#define ESP32_RATE_CHANGE_EN 0

#define SERIAL_BAUD 921600
#define SERIAL_INVERTED false

#define WIFI_AP_PSK     "ShouldNotConnectToThis"

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

static uint8_t wifi_channel;
static uint8_t my_mac_address[6];

static MSP msp_handler;
static uint8_t msp_tx_buffer[256];
static uint8_t serial_tx_buffer[256];

typedef struct {
    uint16_t freq;
    uint8_t addr[6];
    bool valid;
} peer_info_t;
static peer_info_t peers[16];
static peer_info_t rd_info;

String mac_addr_print(uint8_t const * const mac_addr)
{
    char macStr[18] = {0};
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
            mac_addr[5]);
    return String(macStr);
}

bool rd_mac_validate(const uint8_t * mac_addr)
{
    return (rd_info.valid && memcmp(rd_info.addr, mac_addr, 6) == 0);
}

static bool add_peer(uint8_t const * const mac_addr, uint32_t const channel, uint8_t const node_id)
{
    if (ARRAY_SIZE(peers) <= node_id && node_id != 0xff)
        return false;
    bool const peer_exists = esp_now_is_peer_exist((uint8_t*)mac_addr);
#if DEBUG_PRINT
    Serial.print("add_peer(mac: ");
    Serial.print(mac_addr_print(mac_addr));
    Serial.printf(", channel:%u, node_id:%u)\r\n", channel, node_id);
#endif
    if (!peer_exists) {
#ifdef ARDUINO_ARCH_ESP32
        esp_now_peer_info_t peer_info = {
            .peer_addr = {0},
            .lmk = {0},
            .channel = (uint8_t)channel,
            .ifidx = WIFI_IF_AP,
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
    }
    if (node_id == 0xff) {
        // RD info
        rd_info.valid = true;
        memcpy(rd_info.addr, mac_addr, 6);
#if DEBUG_PRINT
        Serial.println("RD's MAC added");
#endif
    } else {
        peers[node_id].valid = true;
        memcpy(peers[node_id].addr, mac_addr, 6);
    }
    return true;
}

static void reset_peers(void)
{
#if DEBUG_PRINT
    Serial.println("reset_peers()");
#endif
    uint8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(peers); iter++) {
        if (peers[iter].valid) {
            if (!rd_mac_validate(peers[iter].addr))
                esp_now_del_peer(peers[iter].addr);
        }
    }
    memset(peers, 0, sizeof(peers));
    if (rd_info.valid && !esp_now_is_peer_exist(rd_info.addr)) {
#if DEBUG_PRINT
        Serial.println("RD is missing... add again");
#endif
        add_peer(rd_info.addr, wifi_channel, 0xff);
    }
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
    Serial.print("Laptimer register cmd (mac: ");
    Serial.print(mac_addr_print(addr));
    Serial.printf(", freq:%u, node_id:%u)\r\n", freq, node_index);
#endif
    laptimer_register_resp_t command = {
        .subcommand = CMD_LAP_TIMER_REGISTER, .freq = freq, .node_index = (uint16_t)(node_index + 1)};
    size_t const size = MSP::bufferPacket(
        msp_tx_buffer, (mspPacketType_e)type, MSP_LAP_TIMER, 0, sizeof(command), (uint8_t *)&command);
    if (size)
        esp_now_send((uint8_t*)addr, msp_tx_buffer, size);
}

typedef struct {
    uint32_t subcommand;
    uint8_t mac_addr[6];
    char pilot[33];
} laptimer_register_ntf_t;  // Send to python


static void esp_now_recv_cb(uint8_t * mac_addr, uint8_t * data, uint8_t const data_len)
{
    static MSP esp_now_msp_rx;
    String log = "";
    uint8_t iter;

    if (!data_len)
        return;

    esp_now_msp_rx.markPacketFree();

    for (iter = 0; iter < data_len; iter++) {
        if (esp_now_msp_rx.processReceivedByte(data[iter])) {
            //  MSP received, check content
            mspPacket_t & packet = esp_now_msp_rx.getPacket();
            if (packet.function == MSP_LAP_TIMER) {
                laptimer_messages_t const * const p_msg = (laptimer_messages_t *)packet.payload;
                //log += " !! - Laptimer message: ";
                if (p_msg->subcommand == CMD_LAP_TIMER_REGISTER) {
                    log += "LAP_TIMER_REGISTER";
                    if (packet.type == MSP_PACKET_V2_COMMAND) {
                        // Check if the client is on on current heat and send resp
                        int8_t const node_index = find_peer_index(mac_addr);
                        if (0 <= node_index) {
                            espnow_laptimer_register_send(
                                mac_addr, node_index, peers[node_index].freq, MSP_PACKET_V2_RESPONSE);
                            log += " - registeration OK";
                        } else {
                            // pilot not configured yet, send info to RH
                            laptimer_register_ntf_t info;
                            info.subcommand = CMD_LAP_TIMER_REGISTER;
                            memcpy(info.mac_addr, mac_addr, sizeof(info.mac_addr));
                            memcpy(info.pilot, p_msg->register_req.pilot, sizeof(info.pilot));
                            size_t const len = MSP::bufferPacket(
                                serial_tx_buffer, MSP_PACKET_V2_COMMAND, MSP_LAP_TIMER, 0,
                                sizeof(info), (uint8_t *)&info);
                            if (len) {
                                Serial.write(serial_tx_buffer, len);
                            }
                        }
                    } else {
                        log += "MSP_LAP_TIMER - MSPv2_RESP -> IGNORE!";
                    }
                } else if (p_msg->subcommand == CMD_LAP_TIMER_START || p_msg->subcommand == CMD_LAP_TIMER_STOP) {
                    if (packet.type == MSP_PACKET_V2_COMMAND && rd_mac_validate(mac_addr)) {
                        size_t const len = MSP::bufferPacket(serial_tx_buffer, &packet);
                        if (len) {
                            Serial.write(serial_tx_buffer, len);
                        }
                    }
                } else {
                    log += "MSP_LAP_TIMER - Unsupported subcommand: ";
                    log += p_msg->subcommand;
                }
            }
            esp_now_msp_rx.markPacketFree();
        }
    }

    if (log.length())
        Serial.println(log);
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
    Serial.print("esp-now sent rdy! (mac: ");
    Serial.print(mac_addr_print(mac_addr));
    Serial.printf(", status:%s\r\n", status ? "FAIL":"OK");
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
            Serial.print("espnow_send_msp() mac: ");
            Serial.print(mac_addr_print(p_addr));
            Serial.printf(", node_id:%u\r\n", node_id);
        } else {
            Serial.printf("espnow_send_msp() broadcasted\r\n");
        }
#endif
    }
}

void wifi_ap_config(const char * ssid, uint8_t const channel)
{
    static bool wifi_setup_done = false;

    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.disconnect(true);
#ifdef ARDUINO_ARCH_ESP32
    WiFi.setTxPower(WIFI_POWER_13dBm);
#else
    WiFi.setOutputPower(13);
#endif
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid, WIFI_AP_PSK, channel);

#if defined(ARDUINO_ARCH_ESP32) && ESP32_RATE_CHANGE_EN
    //  WIFI_PHY_RATE_LORA_250K = 0x29, /**< 250 Kbps */
    //  WIFI_PHY_RATE_LORA_500K = 0x2A, /**< 500 Kbps */
    if (ESP_OK != esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_1M_L))
    {
        Serial.println("WiFi espnow rate config failed!");
    }
#endif

    wifi_channel = channel;

    if (wifi_setup_done) {
        // No need to reinit the ESP-NOW, just remove all peers
        reset_peers();
        return;
    }

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

    wifi_setup_done = true;
}

void setup()
{
    LED_INIT();
    LED_SET(false);

    msp_handler.markPacketFree();

    Serial.setRxBufferSize(512);
#ifdef ARDUINO_ARCH_ESP32
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, -1, -1, SERIAL_INVERTED);
#else
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, SERIAL_INVERTED);
#endif
    delay(100);

    // Read AP MAC address
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPmacAddress(my_mac_address);
    WiFi.mode(WIFI_OFF);
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
                            if (add_peer(p_msg->peer_add.mac, wifi_channel, p_msg->peer_add.node_id)) {
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
                            // Set MAC address
                            memcpy(p_mac, my_mac_address, sizeof(my_mac_address));
                            size_t const len = MSP::bufferPacket(serial_tx_buffer, &msp_in);
                            if (len) {
                                Serial.write(serial_tx_buffer, len);
                                Serial.flush();
                            }
                            // TODO: reset the device?
                            break;
                        }
                        case SUBCMD_ROUTER_WIFI: {
                            wifi_ap_config((char*)p_msg->set_wifi.ssid, p_msg->set_wifi.channel);
                            break;
                        }
                        case SUBCMD_ROUTER_RD: {
#if DEBUG_PRINT
                            Serial.println("SUBCMD_ROUTER_RD...");
#endif
                            if (rd_info.valid) {
                                // Remove existing one
                                esp_now_del_peer(rd_info.addr);
                                rd_info.valid = false;
                            }
                            add_peer(p_msg->set_rd.mac, wifi_channel, 0xff);
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
