#include "ttc_radio.h"
#include "adf4360.h"
#include "adf7021.h"
#include "aic3104.h"
#include "ltc5599.h"
#include "i2s.h"
#include "i2c.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

enum { HALF = 120, TX_SPS = 5, WAVE_SAMPLES = TTC_WAVE_SYMBOLS * TX_SPS };
typedef struct {
    uint32_t sequence;
    int16_t iq[2*HALF];
} AudioBlock;
#define DMA_RAM __attribute__((section(".dma_buffer"), aligned(32)))
static DMA_RAM int16_t tx[4*HALF], rx[4*HALF];
static int16_t wave[2*WAVE_SAMPLES];
static QueueHandle_t blocks, commands;
static TtcReceiver slow, fast;
static volatile unsigned cursor = WAVE_SAMPLES;
static bool occupied[2];
typedef struct { uint32_t deadline, guard; } TxLease;
static volatile TxLease leases[3];
static volatile bool on_air, audio_error;
static volatile uint32_t sequence, overruns, last_dsp;
static uint32_t command_drops;
static uint32_t last_command_time;
static TtcPacket last_command;
static bool have_command;

static void pa(bool on)
{
    /* PC7 connects to the MMZ09332B Power Down pin. */
    HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    on_air = on;
}

void ttc_radio_trip(void)
{
    audio_error=true;
    if (__HAL_RCC_GPIOC_IS_CLK_ENABLED()) pa(false);
}

void ttc_radio_permit(uint32_t duration_ms)
{
    taskENTER_CRITICAL();
    uint32_t deadline=HAL_GetTick()+duration_ms;
    TxLease value=duration_ms ? (TxLease){deadline,~deadline} : (TxLease){0,0};
    for (unsigned i=0;i<3;++i) leases[i]=value;
    if (!duration_ms) {
        pa(false);
        cursor = WAVE_SAMPLES;
        occupied[0] = occupied[1] = false;
    }
    taskEXIT_CRITICAL();
}

static TxLease lease_vote(void)
{
    TxLease a=leases[0], b=leases[1], c=leases[2];
    if (a.guard==~a.deadline && ((a.deadline==b.deadline && a.guard==b.guard) ||
                                (a.deadline==c.deadline && a.guard==c.guard))) return a;
    if (b.guard==~b.deadline && b.deadline==c.deadline && b.guard==c.guard) return b;
    return (TxLease){0,0};
}

static bool allowed(void)
{
    TxLease lease=lease_vote();
    return lease.guard==~lease.deadline && (int32_t)(lease.deadline-HAL_GetTick()) > 0 && !audio_error &&
        HAL_GPIO_ReadPin(TXPLL_LD_GPIO_Port,TXPLL_LD_Pin) &&
        HAL_GPIO_ReadPin(FAULT3V3_GPIO_Port,FAULT3V3_Pin);
}

static void audio_half(unsigned free_half)
{
    BaseType_t wake = pdFALSE;
    AudioBlock b;
    b.sequence = ++sequence;
    memcpy(b.iq,rx+free_half*2*HALF,sizeof b.iq);
    if (xQueueSendFromISR(blocks,&b,&wake) != pdTRUE) ++overruns;
    bool permit = allowed();
    TxLease scrubbed=permit ? lease_vote() : (TxLease){0,0};
    for (unsigned i=0;i<3;++i) leases[i]=scrubbed;
    if (!permit) {
        cursor = WAVE_SAMPLES;
        occupied[0] = occupied[1] = false;
    }
    pa(permit && occupied[1-free_half]);
    int16_t *out = tx+free_half*2*HALF;
    unsigned left = WAVE_SAMPLES-cursor;
    unsigned count = left < HALF ? left : HALF;
    if (count) memcpy(out,wave+2*cursor,count*2*sizeof *out);
    memset(out+2*count,0,(HALF-count)*2*sizeof *out);
    occupied[free_half] = count != 0;
    cursor += count;
    portYIELD_FROM_ISR(wake);
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *h) { if (h == &hi2s1) audio_half(0); }
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *h) { if (h == &hi2s1) audio_half(1); }
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *h)
{
    if (h == &hi2s1) { audio_error = true; pa(false); }
}

bool ttc_radio_send(const TtcPacket *p)
{
    /* Single producer. ISR ignores wave until cursor is published. */
    uint8_t wire[TTC_AIR_BYTES];
    if (cursor != WAVE_SAMPLES || !allowed() || !ttc_encode(p,wire)) return false;
    ttc_modulate(wire,wave,TX_SPS,8192);
    taskENTER_CRITICAL();
    bool ok = allowed();
    if (ok) { __DMB(); cursor = 0; }
    taskEXIT_CRITICAL();
    return ok;
}

bool ttc_radio_receive(TtcPacket *p)
{
    return commands && xQueueReceive(commands,p,0) == pdTRUE;
}

bool ttc_radio_alive(uint32_t now) { return !audio_error && now-last_dsp < 250; }
bool ttc_radio_permitted(void) { return allowed(); }
bool ttc_radio_transmitting(void) { return on_air; }
uint32_t ttc_radio_overruns(void) { return overruns + command_drops; }

static void accept_symbol(TtcReceiver *decoder, int16_t i, int16_t q)
{
    TtcPacket p;
    if (!ttc_rx_sample(decoder,i,q,&p) || p.kind != TTC_COMMAND) return;
    uint32_t now = HAL_GetTick();
    if (have_command && now-last_command_time < 2000 && p.sequence == last_command.sequence &&
        p.length == last_command.length && !memcmp(p.data,last_command.data,p.length)) return;
    if (xQueueSend(commands,&p,0) == pdTRUE) {
        last_command = p;
        last_command_time = now;
        have_command = true;
    } else ++command_drops;
}

void ttc_radio_thread(void *argument)
{
    (void)argument;
    AudioBlock b;
    uint32_t previous = 0;
    for (;;) {
        if (!blocks) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        if (xQueueReceive(blocks,&b,portMAX_DELAY) != pdTRUE) continue;
        if (!slow.samples_per_symbol || b.sequence != previous+1) {
            ttc_rx_init(&slow,40);
            ttc_rx_init(&fast,5);
        }
        previous = b.sequence;
        for (unsigned n=0; n<HALF; ++n) {
            /* Complex downconversion of +12 kHz IF with a 48 kHz clock.
             * HALF is divisible by four, so phase is continuous across blocks. */
            int16_t i=b.iq[2*n]/2, q=b.iq[2*n+1]/2, x=i, y=q;
            switch (n & 3) {
                case 1: x=q; y=-i; break;
                case 2: x=-i; y=-q; break;
                case 3: x=-q; y=i; break;
            }
            accept_symbol(&slow,x,y);
            accept_symbol(&fast,x,y);
        }
        last_dsp = HAL_GetTick();
    }
}

bool ttc_radio_init(void)
{
    ADF7021_Config_t front;
    ADF4360_Config_t pll;
    LTC5599_Config_t mixer;
    AIC3104_Config_t codec;
    uint64_t actual;
    uint8_t echo;
    pa(false);
    if (!blocks) blocks = xQueueCreate(4,sizeof(AudioBlock));
    if (!commands) commands = xQueueCreate(12,sizeof(TtcPacket));
    if (!blocks || !commands) return false;

    HAL_GPIO_WritePin(ADF7021_EN_GPIO_Port, ADF7021_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    GPIO_InitTypeDef shared = {.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,
        .Mode=GPIO_MODE_AF_PP,.Alternate=GPIO_AF6_SPI3,.Speed=GPIO_SPEED_FREQ_HIGH};
    HAL_GPIO_Init(GPIOC,&shared);

    ADF7021_DefaultConfig(&front,&hspi2);
    front.xtal_hz = 25000000;
    front.r_counter = 4;
    front.xosc_enable = false;
    front.rx_freq_hz = TTC_UPLINK_HZ;
    front.rx_if_hz = 12000;
    front.afc_enable = false;

    ADF4360_DefaultConfig(&pll,&hspi3);
    /* Use the largest common PFD up to 500 kHz to keep the selected
     * channel exact with a 25 MHz reference and integer counters. */
    unsigned pfd = 500000, remainder = TTC_DOWNLINK_HZ % pfd;
    while (remainder) {
        unsigned next = pfd % remainder;
        pfd = remainder;
        remainder = next;
    }
    pll.settings.reference_divider = (uint16_t)(pll.settings.ref_in_hz / pfd);
    if (ADF4360_InitConfig(&pll) != HAL_OK ||
        ADF4360_SetFrequencyConfig(&pll,TTC_DOWNLINK_HZ,&actual) != HAL_OK || actual != TTC_DOWNLINK_HZ) return false;
    LTC5599_DefaultConfig(&mixer,&hspi3);
    if (LTC5599_WriteReg(&mixer,0,0x33) != HAL_OK || LTC5599_ReadReg(&mixer,0,&echo) != HAL_OK ||
        echo != 0x33 || LTC5599_WriteReg(&mixer,1,0xc4) != HAL_OK) return false;

    /* CLKOUT, TxRxCLK and TxRxDATA share SPI3 on PCB1_4. Keep the MCU
     * high impedance while the ADF7021 is receiving. */
    shared.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOC,&shared);
    if (ADF7021_Init(&front) != HAL_OK) return false;
    AIC3104_DefaultConfig(&codec,&hi2c1);
    codec.hi2s = &hi2s1;
    codec.clock.pll_a_reg = 0x10; /* divider Q=2, external MCLK = 256fs */
    codec.enable_adc = codec.enable_dac = codec.enable_lineout = true;
    codec.enable_hp = codec.enable_hpcom = false;
    if (AIC3104_Init(&codec) != HAL_OK || AIC3104_UpdateBits(&codec,101,1,1) != HAL_OK ||
        AIC3104_SetInputRoute(&codec,AIC3104_ADC_LEFT,AIC3104_INPUT_LINE1L,true,true) != HAL_OK ||
        AIC3104_SetInputRoute(&codec,AIC3104_ADC_RIGHT,AIC3104_INPUT_LINE1R,true,true) != HAL_OK) return false;
    /* DMA SRAM is NOLOAD. Initialise it before enabling the peripheral. */
    memset(tx,0,sizeof tx);
    memset(rx,0,sizeof rx);
    audio_error = false;
    last_dsp = HAL_GetTick();
    return HAL_I2SEx_TransmitReceive_DMA(&hi2s1,(uint16_t *)tx,(uint16_t *)rx,4*HALF) == HAL_OK;
}
