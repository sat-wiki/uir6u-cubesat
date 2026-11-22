#ifndef TTC_STORE_H
#define TTC_STORE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t generation, boots, reset_flags, faults;
    uint16_t period_s, gain_dac;
} TtcSettings;
bool ttc_settings_valid(const TtcSettings *s);
int ttc_majority(const TtcSettings copies[3], const bool valid[3]);
enum { TTC_NAND_CORRECTED = 1u << 14, TTC_NAND_DAMAGED = 1u << 15 };
uint16_t ttc_store_faults(void);
bool ttc_store_open(TtcSettings *settings);
bool ttc_store_save(TtcSettings *settings);
#endif
