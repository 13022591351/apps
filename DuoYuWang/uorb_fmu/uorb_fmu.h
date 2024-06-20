#pragma once

#include <uORB/uORB.h>

#include <stdint.h>

struct orb_gps_raw_gga_t {
    uint64_t    timestamp;
    uint8_t     gga_raw[100] { '\0' };
};

struct orb_rtcm_t {
    uint64_t    timestamp;
    uint16_t    rtcm_id;
    uint8_t     rtcm_len { 0 };
    uint8_t     rtcm[1200];
};

ORB_DECLARE(orb_gps_gga_raw);
ORB_DECLARE(orb_rtcm_unicore);
ORB_DECLARE(orb_rtcm_ntrip);
