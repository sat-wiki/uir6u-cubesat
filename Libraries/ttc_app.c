#include "ttc_app.h"
#include "ttc_radio.h"
#include "ttc_store.h"
#include "adc.h"
#include "dac.h"
#include "fdcan.h"
#include "ramecc.h"
#include "ds18b20.h"
#include "cmsis_os.h"
#include <string.h>

enum { BAD_STORE=1, BAD_RADIO=2, BAD_CAN1=4, BAD_CAN2=8, POWER_FAULT=16,
       ECC_FIXED=32, ECC_FATAL=64, BAD_TEMP=128, QUEUE_FULL=256, RAM_VOTE=512,
       BAD_ADC=1024, DMA_GAP=2048, CPU_EXCEPTION=4096, STALLED=8192 };
enum { MAX_PENDING=12 };
typedef struct { TtcPacket entries[MAX_PENDING]; unsigned head, count; } PacketQueue;
typedef struct { uint16_t sequence; uint32_t seen; bool valid; } Recent;
static Recent recent[8][8];
static unsigned recent_index[8];
static FDCAN_HandleTypeDef *const bus[] = {&hfdcan1,&hfdcan2};
static bool bus_up[2], radio_up, storage_up;
static PacketQueue downlink;
static PacketQueue can_out[2];
static uint32_t can_enqueued[2][MAX_PENDING];
static bool tx_session;
static uint32_t tx_expires;
static uint32_t faults, next_beacon, last_sample, last_vote, last_bus_check;
static uint32_t spacecraft_us, time_sync_tick, timestamp_polled;
static uint16_t last_can_timestamp;
static unsigned time_bus;
static TtcTimeEvent time_event[2];
static bool synchronized;
static uint32_t last_radio_retry;
static uint16_t serial, pdet, last_tx_pdet;
static int16_t temperature = INT16_MIN;
static volatile uint32_t heartbeat;
static volatile bool running;
static volatile uint32_t crash_words[2] __attribute__((section(".retained_fault"), aligned(8)));

void ttc_record_fault(uint32_t cause)
{
    ttc_radio_trip();
    crash_words[0] = 0x741c0000u | cause;
    crash_words[1] = ~crash_words[0];
    __DSB();
}
static TtcSettings settings[3];
static uint16_t ram_crc[3];
TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef micros_timer;
static RAMECC_HandleTypeDef *const monitors[] = {
    &hramecc1_m1,&hramecc1_m2,&hramecc1_m3,&hramecc1_m4,&hramecc1_m5,
    &hramecc2_m1,&hramecc2_m2,&hramecc2_m3,&hramecc2_m4,&hramecc2_m5,&hramecc3_m1,&hramecc3_m2
};

static void distribute(TtcSettings value)
{
    for (unsigned i=0; i<3; ++i) {
        settings[i] = value;
        ram_crc[i] = ttc_crc((const uint8_t *)&settings[i],sizeof value);
    }
}

static bool vote(void)
{
    bool good[3];
    for (unsigned i=0; i<3; ++i)
        good[i] = ttc_settings_valid(&settings[i]) &&
                  ram_crc[i] == ttc_crc((const uint8_t *)&settings[i],sizeof settings[i]);
    int best = ttc_majority(settings,good);
    if (best < 0) {
        faults |= RAM_VOTE;
        storage_up = false;
        ttc_radio_permit(0);
        distribute((TtcSettings){.period_s=10});
        return false;
    }
    for (unsigned i=0; i<3; ++i)
        if (!good[i] || memcmp(&settings[i],&settings[best],sizeof settings[i])) faults |= RAM_VOTE;
    distribute(settings[best]);
    return true;
}

static bool save(TtcSettings value)
{
    if (!storage_up || !ttc_store_save(&value)) {
        faults |= BAD_STORE;
        storage_up = false;
        ttc_radio_permit(0);
        return false;
    }
    distribute(value);
    return true;
}

static bool start_bus(unsigned n)
{
    FDCAN_FilterTypeDef filter = {.IdType=FDCAN_EXTENDED_ID, .FilterIndex=0,
        .FilterType=FDCAN_FILTER_MASK, .FilterConfig=FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1=0x12a00000, .FilterID2=0x1ff00000};
    return HAL_FDCAN_ConfigFilter(bus[n],&filter)==HAL_OK &&
        HAL_FDCAN_ConfigGlobalFilter(bus[n],FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)==HAL_OK &&
        HAL_FDCAN_ConfigTimestampCounter(bus[n],FDCAN_TIMESTAMP_PRESC_1)==HAL_OK &&
        HAL_FDCAN_EnableTimestampCounter(bus[n],FDCAN_TIMESTAMP_INTERNAL)==HAL_OK &&
        HAL_FDCAN_Start(bus[n])==HAL_OK;
}

static bool send_can(unsigned n, const TtcPacket *p)
{
    FDCAN_TxHeaderTypeDef h = {.Identifier=ttc_can_id(p), .IdType=FDCAN_EXTENDED_ID,
        .TxFrameType=FDCAN_DATA_FRAME, .DataLength=p->length, .FDFormat=FDCAN_CLASSIC_CAN,
        .ErrorStateIndicator=FDCAN_ESI_ACTIVE, .BitRateSwitch=FDCAN_BRS_OFF, .TxEventFifoControl=FDCAN_NO_TX_EVENTS};
    return bus_up[n] && HAL_FDCAN_GetTxFifoFreeLevel(bus[n]) &&
        HAL_FDCAN_AddMessageToTxFifoQ(bus[n],&h,p->data)==HAL_OK;
}

static void publish(const TtcPacket *p)
{
    for (unsigned n=0; n<2; ++n) {
        PacketQueue *q=&can_out[n];
        if (!q->count && send_can(n,p)) continue;
        if (q->count==MAX_PENDING) { faults |= QUEUE_FULL; continue; }
        unsigned tail=(q->head+q->count)%MAX_PENDING;
        q->entries[tail]=*p;
        can_enqueued[n][tail]=HAL_GetTick();
        ++q->count;
    }
}

static bool duplicate(const TtcPacket *p, uint32_t now)
{
    Recent *history = recent[p->kind];
    for (unsigned i=0; i<8; ++i)
        if (history[i].valid && history[i].sequence==p->sequence && now-history[i].seen<2000) return true;
    history[recent_index[p->kind]++ % 8] = (Recent){p->sequence,now,true};
    return false;
}

static void enqueue(const TtcPacket *p)
{
    if (downlink.count==MAX_PENDING) { faults |= QUEUE_FULL; return; }
    downlink.entries[(downlink.head+downlink.count)%MAX_PENDING] = *p;
    ++downlink.count;
}

static void handle_can(unsigned n, uint32_t now)
{
    for (unsigned i=0; i<16 && bus_up[n] && HAL_FDCAN_GetRxFifoFillLevel(bus[n],FDCAN_RX_FIFO0); ++i) {
        FDCAN_RxHeaderTypeDef h;
        uint8_t data[64]; /* HAL copies the received DLC before software validation. */
        TtcPacket p;
        if (HAL_FDCAN_GetRxMessage(bus[n],FDCAN_RX_FIFO0,&h,data)!=HAL_OK) break;
        if (h.IdType!=FDCAN_EXTENDED_ID || h.RxFrameType!=FDCAN_DATA_FRAME || h.FDFormat!=FDCAN_CLASSIC_CAN ||
            !ttc_from_can(h.Identifier,data,h.DataLength,&p)) continue;
        if (p.kind != TTC_TIME && duplicate(&p,now)) continue;
        switch (p.kind) {
        case TTC_TELEMETRY:
            if (tx_session) enqueue(&p);
            break;
        case TTC_CONTROL:
            /* EPS reports deployed and explicitly leases TX for up to 60 s.
             * Revoke discards queued payload from the previous permission. */
            if (p.length==2 && data[0]<=1 && data[1]<=60) {
                uint32_t lease = data[0] && storage_up && radio_up && !(faults & ECC_FATAL) && vote() ? data[1]*1000u : 0;
                tx_session=lease!=0;
                tx_expires=now+lease;
                ttc_radio_permit(lease);
                if (!lease) downlink.count = downlink.head = 0;
            }
            break;
        case TTC_CONFIG:
            if (p.length==4 && vote()) {
                TtcSettings next=settings[0];
                next.period_s=(uint16_t)((unsigned)data[0]<<8 | data[1]);
                next.gain_dac=(uint16_t)((unsigned)data[2]<<8 | data[3]);
                if (ttc_settings_valid(&next) &&
                    (next.period_s!=settings[0].period_s || next.gain_dac!=settings[0].gain_dac) && save(next)) {
                    HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,next.gain_dac);
                    next_beacon = now;
                }
            }
            break;
        case TTC_TIME: {
            uint16_t ticks=HAL_FDCAN_GetTimestampCounter(bus[n]);
            uint32_t stamp;
            if (ttc_time_observe(&time_event[n],&p,h.RxTimestamp,ticks,now,&stamp) && !duplicate(&p,now)) {
                time_bus=n;
                spacecraft_us=stamp;
                last_can_timestamp=ticks;
                timestamp_polled=now;
                synchronized=true;
                time_sync_tick=now;
            }
            break;
        }
        default: break;
        }
    }
}

static void service_buses(uint32_t now)
{
    if (now-last_bus_check >= 1000) {
        last_bus_check=now;
        for (unsigned n=0;n<2;++n) {
            FDCAN_ProtocolStatusTypeDef status;
            if (bus_up[n] && HAL_FDCAN_GetProtocolStatus(bus[n],&status)==HAL_OK && status.BusOff) {
                faults |= BAD_CAN1<<n;
                time_event[n].pending=false;
                if (synchronized && n==time_bus) synchronized=false;
                (void)HAL_FDCAN_Stop(bus[n]);
                bus_up[n]=false;
            }
            if (!bus_up[n]) bus_up[n]=start_bus(n);
        }
    }
    for (unsigned n=0; n<2; ++n) {
        PacketQueue *q=&can_out[n];
        while (q->count) {
            bool stale=now-can_enqueued[n][q->head]>=1000;
            if (!stale && !send_can(n,&q->entries[q->head])) break;
            if (stale) faults |= BAD_CAN1<<n;
            q->head=(q->head+1)%MAX_PENDING;
            --q->count;
        }
    }
    handle_can(0,now);
    handle_can(1,now);
}

static bool start_timer(TIM_HandleTypeDef *timer,TIM_TypeDef *instance,uint32_t period)
{
    timer->Instance=instance;
    timer->Init.Prescaler=(HAL_RCC_GetPCLK1Freq()*2u/1000000u)-1u;
    timer->Init.Period=period;
    return HAL_TIM_Base_Init(timer)==HAL_OK && HAL_TIM_Base_Start(timer)==HAL_OK;
}

static void sample_health(uint32_t now)
{
    if (HAL_ADC_Start(&hadc1)==HAL_OK && HAL_ADC_PollForConversion(&hadc1,1)==HAL_OK) {
        pdet=(uint16_t)HAL_ADC_GetValue(&hadc1);
        if (ttc_radio_transmitting()) last_tx_pdet=pdet;
    } else faults |= BAD_ADC;
    (void)HAL_ADC_Stop(&hadc1);
    if (now-last_sample < 1000) return;
    last_sample=now;
    float temp;
    if (DS18B20_Read(0,&temp)) temperature=(int16_t)(temp*16);
    else { faults |= BAD_TEMP; temperature=INT16_MIN; }
    DS18B20_StartAll();
    for (unsigned i=0; i<sizeof monitors/sizeof *monitors; ++i) {
        uint32_t flags=monitors[i]->Instance->SR & RAMECC_FLAGS_ALL;
        if (flags & RAMECC_FLAG_SINGLEERR_R) faults |= ECC_FIXED;
        if (flags & (RAMECC_FLAG_DOUBLEERR_R|RAMECC_FLAG_DOUBLEERR_W)) faults |= ECC_FATAL;
        if (flags) __HAL_RAMECC_CLEAR_FLAG(monitors[i],flags);
    }
    if (!HAL_GPIO_ReadPin(FAULT3V3_GPIO_Port,FAULT3V3_Pin)) faults |= POWER_FAULT;
    if (ttc_radio_overruns()) faults |= DMA_GAP;
}

static TtcPacket health_packet(void)
{
    return (TtcPacket){.kind=TTC_HEALTH,.sequence=serial++,.length=8,.data={
        (uint8_t)(temperature>>8),(uint8_t)temperature,(uint8_t)(last_tx_pdet>>8),(uint8_t)last_tx_pdet,
        (uint8_t)(faults>>8),(uint8_t)faults,(uint8_t)settings[0].boots,
        (uint8_t)(bus_up[0] | bus_up[1]<<1 | radio_up<<2 | storage_up<<3 | synchronized<<4 |
            (HAL_GPIO_ReadPin(ADF7021_MUXOUT_GPIO_Port,ADF7021_MUXOUT_Pin)==GPIO_PIN_SET)<<5 |
            (HAL_GPIO_ReadPin(TXPLL_LD_GPIO_Port,TXPLL_LD_Pin)==GPIO_PIN_SET)<<6 |
            ttc_radio_transmitting()<<7)}};
}

bool ttc_app_healthy(uint32_t now)
{
    if (!running) return now<3000;
    bool healthy = now-heartbeat<350 && (!radio_up || ttc_radio_alive(now)) && !(faults & ECC_FATAL);
    if (!healthy) ttc_record_fault((faults & ECC_FATAL) ? 8 : 6);
    return healthy;
}

void ttc_app_run(void)
{
    HAL_GPIO_WritePin(TCAN_SHDN1_GPIO_Port,TCAN_SHDN1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TCAN_SHDN2_GPIO_Port,TCAN_SHDN2_Pin,GPIO_PIN_RESET);
    bus_up[0]=start_bus(0);
    bus_up[1]=start_bus(1);
    if (!bus_up[0]) faults |= BAD_CAN1;
    if (!bus_up[1]) faults |= BAD_CAN2;
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    if (!start_timer(&htim3,TIM3,65535) || !start_timer(&micros_timer,TIM2,UINT32_MAX)) Error_Handler();
    TtcSettings first={.period_s=10,.gain_dac=1200};
    storage_up=ttc_store_open(&first);
    distribute(first);
    ++first.boots;
    first.reset_flags=RCC->RSR;
    if (!(first.reset_flags & (RCC_RSR_PORRSTF | RCC_RSR_BORRSTF)) &&
        (crash_words[0]>>16)==0x741c && crash_words[1]==~crash_words[0]) {
        unsigned cause=crash_words[0] & 0xffff;
        if (cause==8) first.faults |= ECC_FATAL;
        else faults |= cause==6 ? STALLED : CPU_EXCEPTION;
        first.faults |= faults;
    }
    crash_words[0]=crash_words[1]=0;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    faults |= ttc_store_faults();
    first.faults |= faults;
    if (!save(first)) { storage_up=false; distribute(first); }
    for (unsigned i=0;i<sizeof monitors/sizeof *monitors;++i) HAL_RAMECC_StartMonitor(monitors[i]);
    HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_2,DAC_ALIGN_12B_R,first.gain_dac);
    HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
    DS18B20_Init(DS18B20_Resolution_12bits);
    radio_up=ttc_radio_init();
    if (!radio_up) faults |= BAD_RADIO;
    heartbeat=HAL_GetTick();
    running=true;
    for (;;) {
        uint32_t now=HAL_GetTick();
        heartbeat=now;
        if (!radio_up && now-last_radio_retry >= 5000) {
            last_radio_retry=now;
            radio_up=ttc_radio_init();
        }
        if (tx_session && ((int32_t)(now-tx_expires)>=0 || !ttc_radio_permitted())) {
            tx_session=false;
            downlink.count=downlink.head=0;
        }
        service_buses(now);
        TtcPacket p;
        while (ttc_radio_receive(&p)) publish(&p);
        sample_health(now);
        if (synchronized) {
            if (now-time_sync_tick>10000 || now-timestamp_polled>120) synchronized=false;
            else {
                uint16_t ticks=HAL_FDCAN_GetTimestampCounter(bus[time_bus]);
                spacecraft_us+=2u*(uint16_t)(ticks-last_can_timestamp);
                last_can_timestamp=ticks;
                timestamp_polled=now;
            }
        }
        if (now-last_vote>=1000) {
            last_vote=now;
            faults |= ttc_store_faults();
            if (vote() && (faults & ~settings[0].faults)) {
                TtcSettings updated=settings[0];
                updated.faults |= faults;
                if (storage_up && !save(updated)) storage_up=false;
            }
        }
        if ((int32_t)(now-next_beacon)>=0) {
            p=health_packet();
            publish(&p);
            if (!ttc_radio_send(&p) && tx_session) enqueue(&p);
            uint32_t stamp=synchronized ? spacecraft_us : __HAL_TIM_GET_COUNTER(&micros_timer);
            TtcPacket diag={.kind=TTC_DIAGNOSTIC,.sequence=serial++,.length=8,.data={
                (uint8_t)(stamp>>24),(uint8_t)(stamp>>16),(uint8_t)(stamp>>8),(uint8_t)stamp,
                (uint8_t)(first.reset_flags>>24),(uint8_t)(first.reset_flags>>16),
                (uint8_t)(settings[0].faults>>8),(uint8_t)settings[0].faults}};
            publish(&diag);
            next_beacon=now+(uint32_t)settings[0].period_s*1000;
        }
        if (downlink.count && ttc_radio_send(&downlink.entries[downlink.head])) {
            downlink.head=(downlink.head+1)%MAX_PENDING;
            --downlink.count;
        }
        osDelay(1);
    }
}
