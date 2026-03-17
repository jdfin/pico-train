#pragma once

#include <cstdint>

struct Loco {
    const char *name;
    uint32_t sn;
};

Loco *get_loco(uint32_t sn);
