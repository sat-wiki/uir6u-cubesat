#ifndef TTC_APP_H
#define TTC_APP_H
#include <stdbool.h>
#include <stdint.h>
void ttc_record_fault(uint32_t cause);
void ttc_app_run(void);
bool ttc_app_healthy(uint32_t now);
#endif
