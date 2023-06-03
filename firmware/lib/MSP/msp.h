#pragma once

#include <Arduino.h>

#define PACKED __attribute__((packed))

#ifndef ARDUINO_ARCH_ESP32
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#endif


uint8_t CalcCRCxor(uint8_t data, uint8_t crc);

#define MSP_PORT_INBUF_SIZE 256

#define MSP_SEQUENCE_MASK (0xF)
#define MSP_VERSION       (1U << 5)
#define MSP_STARTFLAG     (1U << 4)
#define MSP_ERRORFLAG     (1U << 5) // MSP RESP
#define MSP_ELRS_INT      (3U << 0)

typedef enum {
    MSP_IDLE,
    MSP_MSP_START,
    MSP_HEADER_M, // MSPv1
    MSP_HEADER_X, // MSPv2

    MSP_FLAGS,

    MSP_PAYLOAD_SIZE,
    MSP_PAYLOAD_FUNC,
    MSP_PAYLOAD,
    MSP_CHECKSUM,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} mspState_e;

typedef enum {
    MSP_PACKET_UNKNOWN,
    MSP_PACKET_TLM_OTA, // Used to carry info OTA
    MSP_PACKET_V1_ELRS,
    MSP_PACKET_V1_CMD,
    MSP_PACKET_V1_RESP,
    MSP_PACKET_V2_COMMAND,
    MSP_PACKET_V2_RESPONSE
} mspPacketType_e;

/* MSPv1 functions */
enum {
    MSP_VTX_CONFIG = 0x58,     // read
    MSP_VTX_SET_CONFIG = 0x59, // write
    MSP_EEPROM_WRITE = 250,
};

/* MSPv2 functions */
enum {
    MSP_ELRS_FUNC = 0x4578, // ['E','x']

    MSP_ESPNOW_BIND_FUNC = 0x454E, // ['E', 'N']

    MSP_LAP_TIMER = 0x4c54, // ['L', 'T']

    MSP_ROUTER = 0x5254, // ['R', 'T']
};

/************************* ExpressLRS Commands **************************/
enum {
    CMD_ELRS_xxx,
};

/*********************** ESP-NOW Router Commands ************************/
/* MSP_ROUTER subcommands */
enum {
    SUBCMD_ROUTER_RESET = 0x00,
    SUBCMD_ROUTER_ADD = 0x01,
    SUBCMD_ROUTER_PING = 0x02,
};

typedef struct {
    uint32_t subcommand;
} esp_now_router_ping_t;

typedef struct {
    uint32_t subcommand;
} esp_now_router_reset_t;

typedef struct {
    uint32_t subcommand;
    uint16_t node_id;
    uint16_t race_id;
    uint16_t freq;
    uint8_t mac[6];
} esp_now_router_peer_add_t;

typedef union {
    uint32_t subcommand;
    esp_now_router_ping_t ping;
    esp_now_router_reset_t reset;
    esp_now_router_peer_add_t peer_add;
} esp_now_router_messages_t;

/************************** LAPTIMER Commands ***************************/
/* MSP_LAP_TIMER subcommands */
enum {
    CMD_LAP_TIMER_REGISTER = 0x01,
    CMD_LAP_TIMER_SET_NODE,
    CMD_LAP_TIMER_START,
    CMD_LAP_TIMER_STOP,
    CMD_LAP_TIMER_LAP,
};

typedef struct {
    uint32_t subcommand;
    char pilot[33];
} PACKED laptimer_register_req_t;

typedef struct {
    uint32_t subcommand;
    uint16_t freq;
    uint16_t node_index;
} PACKED laptimer_register_resp_t;

typedef struct { // Allowed only in training mode
    uint32_t subcommand;
    laptimer_register_resp_t info;
    laptimer_register_req_t pilot;
} PACKED laptimer_set_node_t;

typedef struct {
    uint32_t subcommand;
    uint16_t node_index;
    uint16_t race_id;
} PACKED laptimer_start_t;

typedef struct {
    uint32_t subcommand;
    uint16_t node_index;
    uint16_t race_id;
} PACKED laptimer_stop_t;

typedef struct {
    uint32_t subcommand;
    uint32_t lap_time_ms;
    uint16_t node_index;
    uint16_t race_id;
    uint8_t lap_index;
} PACKED laptimer_lap_t;

typedef union {
    uint32_t subcommand;
    laptimer_register_req_t register_req;
    laptimer_register_resp_t register_resp;
    laptimer_set_node_t set_node;
    laptimer_start_t start;
    laptimer_stop_t stop;
    laptimer_lap_t lap;
} laptimer_messages_t;

/**************************   MSP Messaging   **************************/

typedef struct {
    uint8_t flags;
    uint16_t function;
    uint16_t payloadSize;
} PACKED mspHeaderV2_t;

typedef struct {
    mspPacketType_e type;
    uint8_t WORD_ALIGNED_ATTR payload[MSP_PORT_INBUF_SIZE];
    uint16_t function;
    uint16_t payloadSize;
    uint16_t payloadIterator;
    uint8_t flags;
    uint8_t sequence_nbr;
    uint8_t crc;
    bool error;

    inline uint8_t iterated(void) const
    {
        return ((type != MSP_PACKET_UNKNOWN) &&
                ((0 < payloadSize && payloadSize <= payloadIterator) || (payloadSize == 0)));
    }

    uint8_t is_free(void) const
    {
        return (type == MSP_PACKET_UNKNOWN);
    }

    void reset(void)
    {
        type = MSP_PACKET_UNKNOWN;
        flags = 0;
        function = 0;
        payloadSize = 0;
        payloadIterator = 0;
        error = false;
        sequence_nbr = 0;
        crc = 0;
    }

    inline void addByte(uint8_t const b)
    {
        if (payloadIterator >= sizeof(payload)) {
            error = true;
            return;
        }
        crc = CalcCRCxor(b, crc);
        payload[payloadIterator++] = b;
    }

    inline void add(uint8_t const b)
    {
        addByte(b);
    }

    inline void add(uint8_t const * data, size_t len)
    {
        while (len--)
            addByte(*data++);
    }

    inline void setIteratorToSize(void)
    {
        payloadSize = payloadIterator;
        payloadIterator = 0;
    }

    inline uint8_t readByte(void)
    {
        if (iterated()) {
            // We are trying to read beyond the length of the payload
            error = true;
            return 0;
        }
        return payload[payloadIterator++];
    }
} mspPacket_t;

/////////////////////////////////////////////////

class MSP
{
public:
    MSP()
    {
    }
    bool processReceivedByte(uint8_t c);
    bool mspOngoing()
    {
        return (m_inputState != MSP_IDLE);
    }
    bool mspReceived()
    {
        return (m_inputState == MSP_COMMAND_RECEIVED);
    }
    mspPacket_t & getPacket()
    {
        return m_packet;
    }
    mspPacket_t * getPacketPtr()
    {
        return &m_packet;
    }
    uint8_t error()
    {
        return m_packet.error;
    }
    void markPacketFree()
    {
        // Set input state to idle, ready to receive the next packet
        // The current packet data will be discarded internally
        m_inputState = MSP_IDLE;
    }
    static size_t bufferPacket(uint8_t * output_ptr,
                               mspPacketType_e type,
                               uint16_t function,
                               uint8_t flags,
                               uint8_t len,
                               uint8_t const * payload);
    static size_t bufferPacket(uint8_t * output_ptr, mspPacket_t * packet);

private:
    mspPacket_t m_packet;
    mspState_e m_inputState = MSP_IDLE;
    uint16_t m_offset = 0;
    uint8_t m_crc = 0, m_crc_v1 = 0;
};
