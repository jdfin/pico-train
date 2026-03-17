
#include <cassert>
//
#include "dcc_api.h"
using Status = DccApi::Status;
//
#include "locos.h"

extern int loco_id;


static Loco locos[] = {
    {
        .name = "ML560",
        .sn = 4195122288,
    },
    {
        .name = "SP2265",
        .sn = 4192888681,
    },
};

static constexpr int locos_max = sizeof(locos) / sizeof(locos[0]);

Loco *get_loco(uint32_t sn)
{
    for (int i = 0; i < locos_max; i++) {
        if (sn == locos[i].sn)
            return &locos[i];
    }
    return nullptr;
}
