#include "ttc_link.h"
#include "ttc_store.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void receive(unsigned sps, int offset)
{
    TtcPacket sent = {.kind=TTC_COMMAND,.sequence=0x1234,.length=3,.data={1,2,3}}, got;
    uint8_t wire[TTC_AIR_BYTES];
    int16_t iq[TTC_WAVE_SYMBOLS*TTC_MAX_SPS*2];
    assert(ttc_encode(&sent,wire));
    assert(ttc_decode(wire+8,&got));
    assert(got.sequence==sent.sequence && !memcmp(got.data,sent.data,8));
    ttc_modulate(wire,iq,sps,6000);
    TtcReceiver rx;
    ttc_rx_init(&rx,sps);
    unsigned matches=0;
    for (int i=0; i<offset; ++i) ttc_rx_sample(&rx,0,0,&got);
    for (unsigned n=0; n<TTC_WAVE_SYMBOLS*sps; ++n) {
        double phase=1.3+6.283185307179586*95*n/48000;
        int16_t x=(int16_t)(iq[2*n]*cos(phase)+(int)(n%17)-8);
        int16_t y=(int16_t)(iq[2*n]*sin(phase)+(int)(n%23)-11);
        if (ttc_rx_sample(&rx,x,y,&got)) {
            assert(got.sequence==0x1234 && got.length==3 && !memcmp(got.data,sent.data,8));
            ++matches;
        }
    }
    assert(matches);
    wire[15]^=1;
    assert(!ttc_decode(wire+8,&got));
    uint32_t id=ttc_can_id(&sent);
    assert(id==0x12a11234 && ttc_from_can(id,sent.data,3,&got));
    assert(!ttc_from_can(id^0x100000,sent.data,3,&got));
    assert(!ttc_from_can(id,sent.data,9,&got));
}

/* Exchange the exact IQ representation used at the codec boundary with
 * the independent ground tool. This host test runs on little-endian hosts. */
static int interop(const char *action,const char *path,unsigned sps)
{
    static int16_t buffer[TTC_WAVE_SYMBOLS*TTC_MAX_SPS*2];
    FILE *file=fopen(path,!strcmp(action,"write") ? "wb" : "rb");
    assert(file);
    unsigned count=TTC_WAVE_SYMBOLS*sps;
    TtcPacket p={.kind=TTC_COMMAND,.length=3,.sequence=0x1234,.data={1,2,3}};
    if (!strcmp(action,"write")) {
        uint8_t wire[TTC_AIR_BYTES];
        assert(ttc_encode(&p,wire));
        assert(wire[21]==0x5f && wire[22]==0xae);
        ttc_modulate(wire,buffer,sps,8192);
        assert(fwrite(buffer,sizeof(int16_t)*2,count,file)==count);
    } else {
        assert(fread(buffer,sizeof(int16_t)*2,count,file)==count);
        TtcReceiver rx;
        ttc_rx_init(&rx,sps);
        unsigned found=0;
        for (unsigned n=0;n<count;++n) {
            int16_t i=buffer[2*n],q=buffer[2*n+1];
            switch (n%4) {
            case 1: { int16_t t=i; i=q; q=-t; } break;
            case 2: i=-i; q=-q; break;
            case 3: { int16_t t=i; i=-q; q=t; } break;
            }
            if (ttc_rx_sample(&rx,i,q,&p)) {
                assert(p.kind==TTC_COMMAND && p.sequence==0x1234 && p.length==3 && p.data[2]==3);
                ++found;
            }
        }
        assert(found);
    }
    fclose(file);
    return 0;
}

int main(int argc,char **argv)
{
    if (argc==4) return interop(argv[1],argv[2],!strcmp(argv[3],"1200") ? 40 : 5);
    assert(ttc_crc((const uint8_t *)"123456789",9)==0x29b1);
    for (int offset=0;offset<40;++offset) receive(40,offset);
    for (int offset=0;offset<5;++offset) receive(5,offset);
    TtcTimeEvent event={0};
    TtcPacket sync={.kind=TTC_TIME,.sequence=3};
    uint32_t stamp=0;
    assert(!ttc_time_observe(&event,&sync,65530,65532,1,&stamp));
    sync.length=4;
    sync.data[3]=10;
    ++sync.sequence;
    assert(!ttc_time_observe(&event,&sync,5,8,2,&stamp));
    --sync.sequence;
    assert(ttc_time_observe(&event,&sync,5,8,2,&stamp) && stamp==38);
    assert(!ttc_time_observe(&event,&sync,5,8,2,&stamp));
    sync.length=0;
    assert(!ttc_time_observe(&event,&sync,0,0,2,&stamp));
    sync.length=4;
    assert(!ttc_time_observe(&event,&sync,10,11,106,&stamp));
    TtcSettings c[3]={{.period_s=10},{.period_s=10},{.period_s=11}};
    bool ok[]={true,true,true};
    assert(ttc_majority(c,ok)==0);
    ok[1]=false;
    assert(ttc_majority(c,ok)==-1);
    puts("BPSK both rates, all symbol phases, residual carrier, CAN IDs, CRC and voting passed");
}
