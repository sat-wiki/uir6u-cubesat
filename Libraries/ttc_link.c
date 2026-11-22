#include "ttc_link.h"
#include <string.h>

#define SYNC_WORD UINT32_C(0x1acffc1d)
#define CAN_PREFIX UINT32_C(0x12a00000)

uint16_t ttc_crc(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffff;
    while (length--) {
        crc ^= (uint16_t)*data++ << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}

bool ttc_encode(const TtcPacket *p, uint8_t wire[TTC_AIR_BYTES])
{
    if (!p || !wire || p->length > 8 || p->kind < TTC_COMMAND || p->kind > TTC_TIME) return false;
    const uint8_t prefix[] = {0x55,0x55,0x55,0x55,0x1a,0xcf,0xfc,0x1d};
    memcpy(wire, prefix, sizeof prefix);
    uint8_t *b = wire + sizeof prefix;
    b[0] = 1;
    b[1] = p->kind;
    b[2] = p->sequence >> 8;
    b[3] = (uint8_t)p->sequence;
    b[4] = p->length;
    memset(b + 5, 0, 8);
    memcpy(b + 5, p->data, p->length);
    uint16_t crc = ttc_crc(b, 13);
    b[13] = crc >> 8;
    b[14] = (uint8_t)crc;
    return true;
}

bool ttc_decode(const uint8_t b[TTC_BODY], TtcPacket *p)
{
    if (!p || !b || b[0] != 1 || b[1] < TTC_COMMAND || b[1] > TTC_TIME || b[4] > 8 ||
        ttc_crc(b,13) != ((unsigned)b[13] << 8 | b[14])) return false;
    for (unsigned n = b[4]; n < 8; ++n) if (b[5+n]) return false;
    p->kind = b[1];
    p->sequence = (uint16_t)((unsigned)b[2] << 8 | b[3]);
    p->length = b[4];
    memcpy(p->data, b+5, 8);
    return true;
}

uint32_t ttc_can_id(const TtcPacket *p)
{
    return CAN_PREFIX | ((uint32_t)p->kind << 16) | p->sequence;
}

bool ttc_from_can(uint32_t id, const uint8_t *data, unsigned length, TtcPacket *p)
{
    unsigned kind = (id >> 16) & 15;
    if ((id & 0xfff00000u) != CAN_PREFIX || kind < TTC_COMMAND || kind > TTC_TIME ||
        !p || !data || length > 8) return false;
    *p = (TtcPacket){.kind=(uint8_t)kind, .length=(uint8_t)length, .sequence=(uint16_t)id};
    memcpy(p->data, data, length);
    return true;
}

bool ttc_time_observe(TtcTimeEvent *event, const TtcPacket *packet, uint16_t frame_tick,
                      uint16_t current_tick, uint32_t now_ms, uint32_t *now_us)
{
    if (packet->kind != TTC_TIME) return false;
    if (packet->length == 0) {
        if (!event->pending || event->sequence != packet->sequence || now_ms - event->received_ms > 100)
            *event = (TtcTimeEvent){now_ms, packet->sequence, frame_tick, true};
        return false;
    }
    if (packet->length != 4 || !event->pending || packet->sequence != event->sequence) return false;
    event->pending = false;
    if (now_ms - event->received_ms > 100 || (uint16_t)(current_tick-event->timestamp) > 50000u)
        return false;
    const uint8_t *p = packet->data;
    uint32_t remote = (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3];
    *now_us = remote + 2u*(uint16_t)(current_tick-event->timestamp);
    return true;
}

/* Raised-cosine pulse, beta=0.25, +/-4 symbols. At 9600 baud its
 * ideal null-to-null bandwidth is 12 kHz. Table spacing is 1/40 symbol. */
static const int16_t pulse[161] = {
    32767, 32732, 32628, 32454, 32212, 31902, 31526, 31085, 30582, 30017, 29393, 28713,
    27979, 27195, 26363, 25486, 24568, 23612, 22623, 21603, 20557, 19488, 18401, 17300,
    16188, 15069, 13948, 12828, 11713, 10607, 9514, 8438, 7381, 6347, 5339, 4361,
    3415, 2503, 1629, 794, 0, -750, -1456, -2116, -2728, -3293, -3808, -4275,
    -4692, -5060, -5379, -5650, -5873, -6049, -6179, -6265, -6307, -6309, -6270, -6194,
    -6082, -5937, -5760, -5554, -5322, -5065, -4787, -4490, -4176, -3848, -3508, -3159,
    -2804, -2445, -2084, -1723, -1365, -1012, -665, -327, 0, 315, 617, 904,
    1175, 1428, 1663, 1879, 2076, 2252, 2407, 2542, 2656, 2748, 2820, 2872,
    2903, 2914, 2907, 2881, 2838, 2779, 2703, 2614, 2510, 2395, 2268, 2131,
    1985, 1832, 1673, 1509, 1341, 1170, 998, 826, 655, 486, 319, 157,
    0, -151, -296, -434, -564, -685, -797, -900, -994, -1077, -1150, -1213,
    -1266, -1308, -1340, -1363, -1375, -1379, -1373, -1358, -1335, -1304, -1266, -1221,
    -1170, -1113, -1051, -985, -915, -842, -767, -689, -611, -531, -451, -372,
    -294, -217, -142, -70, 0,
};

void ttc_modulate(const uint8_t wire[TTC_AIR_BYTES], int16_t *stereo, unsigned sps, int16_t amplitude)
{
    if (sps != 5 && sps != 40) return;
    int signs[TTC_AIR_BYTES*8];
    int level = 1;
    for (unsigned b=0; b<TTC_AIR_BYTES*8; ++b) {
        if (wire[b/8] & (0x80u >> (b%8))) level = -level;
        signs[b] = level;
    }
    for (int symbol=0; symbol<TTC_WAVE_SYMBOLS; ++symbol) {
        for (unsigned phase=0; phase<sps; ++phase) {
            int sum = 0;
            for (int tap=0; tap<=8; ++tap) {
                int bit = symbol-tap;
                int offset = (tap-4)*40 + (int)(phase*(40/sps));
                if (offset<0) offset=-offset;
                if (bit>=0 && bit<TTC_AIR_BYTES*8 && offset<=160) sum += signs[bit]*pulse[offset];
            }
            int value = (int)((int64_t)amplitude*sum/32767);
            if (value>32767) value=32767;
            if (value<-32768) value=-32768;
            *stereo++ = (int16_t)value;
            *stereo++ = 0;
        }
    }
}

void ttc_rx_init(TtcReceiver *rx, unsigned sps)
{
    memset(rx, 0, sizeof *rx);
    if (sps <= TTC_MAX_SPS) rx->samples_per_symbol = sps;
}

bool ttc_rx_sample(TtcReceiver *rx, int16_t i, int16_t q, TtcPacket *packet)
{
    if (!rx->samples_per_symbol) return false;
    unsigned n = rx->index;
    rx->sum_i += (int32_t)i - rx->i[n];
    rx->sum_q += (int32_t)q - rx->q[n];
    rx->i[n] = i;
    rx->q[n] = q;
    rx->index = (n+1 == rx->samples_per_symbol) ? 0 : n+1;

    TtcPhase *p = &rx->phase[n];
    int64_t dot = (int64_t)p->prev_i * rx->sum_i + (int64_t)p->prev_q * rx->sum_q;
    p->prev_i = rx->sum_i;
    p->prev_q = rx->sum_q;
    unsigned bit = dot < 0;
    p->sync = (p->sync << 1) | bit;
    if (!p->collecting) {
        if (p->sync == SYNC_WORD) {
            p->bits = 0;
            p->collecting = true;
        }
        return false;
    }
    unsigned byte = p->bits / 8;
    p->body[byte] = (uint8_t)((p->body[byte] << 1) | bit);
    if (++p->bits != TTC_BODY * 8) return false;
    p->collecting = false;
    return ttc_decode(p->body, packet);
}
