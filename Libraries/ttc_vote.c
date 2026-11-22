#include "ttc_store.h"
#include <string.h>

_Static_assert(sizeof(TtcSettings) == 20, "Persistent format v1 is 20 bytes");
bool ttc_settings_valid(const TtcSettings *s)
{
    return s->period_s >= 1 && s->period_s <= 3600 && s->gain_dac <= 4095;
}

int ttc_majority(const TtcSettings c[3], const bool valid[3])
{
    for (int i = 0; i < 2; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (valid[i] && valid[j] && !memcmp(&c[i], &c[j], sizeof c[i])) return i;
    return -1;
}
