#include "ttc_store.h"
#include "ttc_link.h"
#include "spi.h"
#include <string.h>

/* MT29F2G01ABAGDWB. The bench profile reserves the last 24 erase blocks.
 * Each of six replica slots has four candidate blocks for factory bad marks. */
enum { FIRST_BLOCK = 2024, PAGES_PER_BLOCK = 64, RECORD_BYTES = 26 };
static uint16_t blocks[2][3];
static int active_bank;
static bool writable;
static uint16_t flash_faults;

uint16_t ttc_store_faults(void)
{
    return flash_faults;
}

static bool command(const uint8_t *header, unsigned n, uint8_t *rx, const uint8_t *tx, unsigned length)
{
    HAL_GPIO_WritePin(NAND_CS_GPIO_Port, NAND_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi4, (uint8_t *)header, n, 10);
    if (st == HAL_OK && rx) st = HAL_SPI_Receive(&hspi4, rx, length, 10);
    if (st == HAL_OK && tx) st = HAL_SPI_Transmit(&hspi4, (uint8_t *)tx, length, 10);
    HAL_GPIO_WritePin(NAND_CS_GPIO_Port, NAND_CS_Pin, GPIO_PIN_SET);
    return st == HAL_OK;
}

static bool feature(uint8_t reg, uint8_t *v)
{
    uint8_t h[] = {0x0f,reg};
    return command(h,sizeof h,v,NULL,1);
}

static bool ready(uint8_t *status)
{
    uint32_t start = HAL_GetTick();
    do {
        if (!feature(0xc0,status)) return false;
        if (!(*status & 1)) return true;
    } while (HAL_GetTick()-start < 15);
    return false;
}

static bool row_command(uint8_t opcode, uint16_t block)
{
    uint32_t row = (uint32_t)block * PAGES_PER_BLOCK;
    uint8_t h[] = {opcode, (uint8_t)(row>>16), (uint8_t)(row>>8), (uint8_t)row};
    return command(h,sizeof h,NULL,NULL,0);
}

static bool read_page(uint16_t block, unsigned col, uint8_t *dst, unsigned size)
{
    uint8_t status;
    if (!row_command(0x13,block) || !ready(&status)) return false;
    unsigned ecc = (status >> 4) & 7;
    if (col < 2048 && ecc) {
        if (ecc != 1 && ecc != 3 && ecc != 5) {
            flash_faults |= TTC_NAND_DAMAGED;
            return false;
        }
        flash_faults |= TTC_NAND_CORRECTED;
    }
    col |= (block & 1u) << 12;
    uint8_t h[] = {0x03, (uint8_t)(col>>8), (uint8_t)col, 0};
    return command(h,sizeof h,dst,NULL,size);
}

static bool write_enable(void)
{
    uint8_t op = 0x06, status;
    return command(&op,1,NULL,NULL,0) && feature(0xc0,&status) && (status & 2);
}

static bool write_record(uint16_t block, const TtcSettings *s)
{
    uint8_t raw[RECORD_BYTES] = {'U','V','A',1}, status, check[RECORD_BYTES];
    memcpy(raw+4,s,sizeof *s);
    uint16_t crc = ttc_crc(raw,24);
    raw[24] = crc>>8;
    raw[25] = (uint8_t)crc;
    uint8_t h[] = {0x02,(uint8_t)((block & 1u)<<4),0};
    return block != UINT16_MAX && write_enable() && row_command(0xd8,block) && ready(&status) &&
        !(status & 4) && write_enable() && command(h,sizeof h,NULL,raw,sizeof raw) &&
        row_command(0x10,block) && ready(&status) && !(status & 8) &&
        read_page(block,0,check,sizeof check) && !memcmp(check,raw,sizeof raw);
}

bool ttc_store_save(TtcSettings *s)
{
    if (!writable || !ttc_settings_valid(s)) return false;
    TtcSettings next = *s;
    ++next.generation;
    unsigned bank = active_bank == 0 ? 1 : 0;
    unsigned good = 0;
    for (unsigned i=0; i<3; ++i) good += write_record(blocks[bank][i],&next);
    if (good < 3) flash_faults |= TTC_NAND_DAMAGED;
    if (good < 2) return false;
    *s = next;
    active_bank = (int)bank;
    return true;
}

bool ttc_store_open(TtcSettings *s)
{
    static const uint8_t id_cmd[] = {0x9f,0}, ecc_on[] = {0x1f,0xb0,0x10}, unlock[] = {0x1f,0xa0,0};
    uint8_t id[2], status;
    flash_faults = 0;
    writable = false;
    active_bank = -1;
    if (!ready(&status) || !command(id_cmd,2,id,NULL,2) || id[0]!=0x2c || id[1]!=0x24 ||
        !command(ecc_on,3,NULL,NULL,0) || !feature(0xb0,&status) || !(status & 0x10) ||
        !command(unlock,3,NULL,NULL,0) || !feature(0xa0,&status) || (status & 0x7c)) return false;
    TtcSettings copies[2][3] = {0};
    bool valid[2][3] = {{false}}, blank = true;
    for (unsigned bank=0; bank<2; ++bank) {
        for (unsigned slot=0; slot<3; ++slot) {
            uint16_t block = FIRST_BLOCK + (bank*3+slot)*4;
            blocks[bank][slot] = UINT16_MAX;
            for (unsigned n=0; n<4; ++n,++block) {
                uint8_t mark;
                if (!read_page(block,2048,&mark,1)) { blank = false; break; }
                if (mark != 0xff) continue;
                blocks[bank][slot] = block;
                uint8_t raw[RECORD_BYTES];
                if (!read_page(block,0,raw,sizeof raw)) { blank = false; break; }
                for (unsigned k=0; k<sizeof raw; ++k) if (raw[k]!=0xff) blank = false;
                memcpy(&copies[bank][slot],raw+4,sizeof *s);
                valid[bank][slot] = !memcmp(raw,"UVA\1",4) &&
                    ttc_crc(raw,24) == ((unsigned)raw[24]<<8 | raw[25]) &&
                    ttc_settings_valid(&copies[bank][slot]);
                break;
            }
        }
        int vote = ttc_majority(copies[bank],valid[bank]);
        if (vote >= 0 && (active_bank < 0 || (int32_t)(copies[bank][vote].generation-s->generation)>0)) {
            active_bank = (int)bank;
            *s = copies[bank][vote];
        }
    }
    if (active_bank < 0 && !blank) { flash_faults |= TTC_NAND_DAMAGED; return false; }
    writable = true;
    if (active_bank >= 0) {
        for (unsigned n=0; n<3; ++n)
            if (!valid[active_bank][n] || memcmp(s,&copies[active_bank][n],sizeof *s)) {
                flash_faults |= TTC_NAND_DAMAGED;
                (void)write_record(blocks[active_bank][n],s);
            }
    }
    return true;
}
