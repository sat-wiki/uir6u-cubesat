#ifndef TTC_LINK_H
#define TTC_LINK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Bench interface v1. One air packet transports one Classical CAN payload. */
enum { TTC_COMMAND = 1, TTC_TELEMETRY, TTC_HEALTH, TTC_CONTROL, TTC_CONFIG, TTC_DIAGNOSTIC, TTC_TIME };
enum { TTC_BODY = 15, TTC_AIR_BYTES = 8 + TTC_BODY, TTC_WAVE_SYMBOLS = TTC_AIR_BYTES * 8 + 8, TTC_MAX_SPS = 40 };
typedef struct {
    uint8_t kind, length;
    uint16_t sequence;
    uint8_t data[8];
} TtcPacket;
uint16_t ttc_crc(const uint8_t *data, size_t length);
bool ttc_encode(const TtcPacket *packet, uint8_t wire[TTC_AIR_BYTES]);
bool ttc_decode(const uint8_t body[TTC_BODY], TtcPacket *packet);
uint32_t ttc_can_id(const TtcPacket *packet);
bool ttc_from_can(uint32_t id, const uint8_t *data, unsigned length, TtcPacket *packet);

/* Two-step CAN time transfer uses the captured start of the sync frame.
 * Keep a separate pending event for each physical bus. */
typedef struct {
    uint32_t received_ms;
    uint16_t sequence, timestamp;
    bool pending;
} TtcTimeEvent;
bool ttc_time_observe(TtcTimeEvent *event, const TtcPacket *packet, uint16_t frame_tick,
                      uint16_t current_tick, uint32_t now_ms, uint32_t *now_us);

/* Each timing phase examines a rolling symbol integral. Adjacent complex
 * symbols recover differential BPSK without an absolute carrier phase. */
typedef struct {
    int32_t prev_i, prev_q;
    uint32_t sync;
    uint8_t body[TTC_BODY];
    unsigned bits;
    bool collecting;
} TtcPhase;
typedef struct {
    int16_t i[TTC_MAX_SPS], q[TTC_MAX_SPS];
    int32_t sum_i, sum_q;
    unsigned index, samples_per_symbol;
    TtcPhase phase[TTC_MAX_SPS];
} TtcReceiver;
void ttc_rx_init(TtcReceiver *rx, unsigned samples_per_symbol);
bool ttc_rx_sample(TtcReceiver *rx, int16_t i, int16_t q, TtcPacket *packet);
void ttc_modulate(const uint8_t wire[TTC_AIR_BYTES], int16_t *stereo, unsigned sps, int16_t amplitude);
#endif
