#ifndef TTC_RADIO_H
#define TTC_RADIO_H
#include "ttc_link.h"
bool ttc_radio_init(void);
void ttc_radio_thread(void *argument);
bool ttc_radio_send(const TtcPacket *p);
bool ttc_radio_receive(TtcPacket *p);
void ttc_radio_trip(void);
void ttc_radio_permit(uint32_t duration_ms);
bool ttc_radio_alive(uint32_t now);
bool ttc_radio_permitted(void);
bool ttc_radio_transmitting(void);
uint32_t ttc_radio_overruns(void);
#endif
