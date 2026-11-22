#include "spi.h"
#include "ttc_store.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

enum { FIRST=2024, BLOCKS=24, PAGE=2176 };
static uint8_t flash[BLOCKS][PAGE], saved[BLOCKS][PAGE], cache[PAGE], feature[256];
static unsigned row, column, reg, op, transfers, mutations, interrupt_at;
static bool selected;
static uint8_t ecc[BLOCKS];
static jmp_buf reboot;
SPI_HandleTypeDef hspi4;
uint32_t HAL_GetTick(void) { static uint32_t ticks; return ticks++; }
void HAL_GPIO_WritePin(int port,int pin,int state)
{
    (void)port; (void)pin;
    selected = state==GPIO_PIN_RESET;
    transfers=0;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,uint8_t *p,unsigned n,unsigned timeout)
{
    (void)spi; (void)timeout;
    assert(selected && n);
    if (transfers++) {
        assert(op==2 && column+n<=PAGE);
        memcpy(cache+column,p,n);
        return HAL_OK;
    }
    op=p[0];
    switch (op) {
    case 6: feature[0xc0]|=2; break;
    case 0x0f: reg=p[1]; break;
    case 0x1f: feature[p[1]]=p[2]; break;
    case 0x13: case 0xd8: case 0x10: {
        unsigned address=((unsigned)p[1]<<16 | (unsigned)p[2]<<8 | p[3]);
        assert(!(address%64) && address/64>=FIRST && address/64<FIRST+BLOCKS);
        row=address/64-FIRST;
        if (op==0x13) {
            memcpy(cache,flash[row],PAGE);
            feature[0xc0]=(feature[0xc0] & 0x0f) | (ecc[row]<<4);
            break;
        }
        assert(flash[row][2048]==0xff && (feature[0xc0]&2));
        feature[0xc0]&=~2u;
        if (op==0xd8) memset(flash[row],0xff,PAGE);
        else for (unsigned i=0;i<PAGE;++i) flash[row][i]&=cache[i];
        if (++mutations==interrupt_at) longjmp(reboot,1);
        break;
    }
    case 0x03: case 0x02:
        column=((p[1]&15u)<<8)|p[2];
        if (op==0x03) assert((p[1]>>4)==(row&1));
        else memset(cache,0xff,PAGE);
        break;
    case 0x9f: assert(n==2); break;
    default: assert(0);
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *spi,uint8_t *p,unsigned n,unsigned timeout)
{
    (void)spi; (void)timeout;
    assert(selected);
    if (op==0x9f) { p[0]=0x2c; p[1]=0x24; }
    else if (op==0x0f) p[0]=feature[reg];
    else { assert(op==3 && column+n<=PAGE); memcpy(p,cache+column,n); }
    return HAL_OK;
}

int main(void)
{
    memset(flash,0xff,sizeof flash);
    flash[0][2048]=0; /* the next block in this replica slot is on the odd plane */
    TtcSettings cfg={.period_s=10,.gain_dac=1200};
    assert(ttc_store_open(&cfg) && ttc_store_save(&cfg) && cfg.generation==1);
    memcpy(saved,flash,sizeof flash);
    for (unsigned cut=1;cut<=6;++cut) {
        memcpy(flash,saved,sizeof flash);
        TtcSettings restored={0};
        assert(ttc_store_open(&restored) && restored.period_s==10);
        restored.period_s=23;
        mutations=0;
        interrupt_at=cut;
        if (!setjmp(reboot)) { (void)ttc_store_save(&restored); assert(0); }
        interrupt_at=0;
        memset(&restored,0,sizeof restored);
        assert(ttc_store_open(&restored));
        assert(restored.period_s==(cut>=4 ? 23 : 10));
    }
    memcpy(flash,saved,sizeof flash);
    flash[4][6]^=1;
    assert(ttc_store_open(&cfg));
    assert(!memcmp(flash[1],flash[4],26));
    assert(ttc_store_faults() & TTC_NAND_DAMAGED);
    flash[4][6]^=1;
    flash[8][6]^=1;
    assert(!ttc_store_open(&cfg));
    memcpy(flash,saved,sizeof flash);
    ecc[1]=5;  /* seven to eight bit errors corrected by the chip */
    assert(ttc_store_open(&cfg) && (ttc_store_faults() & TTC_NAND_CORRECTED));
    ecc[1]=2;
    assert(ttc_store_open(&cfg) && (ttc_store_faults() & TTC_NAND_DAMAGED));
    ecc[4]=2;
    assert(!ttc_store_open(&cfg));
    puts("NAND bad blocks, ECC, six power cuts, repair and no-quorum rejection passed");
}
