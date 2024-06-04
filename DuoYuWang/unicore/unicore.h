#pragma once

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <wchar.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

#include <string>
#include <vector>
#include <memory>

#include <minmea/minmea.h>

#include <uorb_fmu.h>
#include <uorb_helper.h>

#define NMEA_MAX_LENGTH 512
#define RTCM_MAX_LENGTH 2560

class Unicore {
public:
    explicit Unicore(void);
    virtual ~Unicore(void);

    int uni_open(void);
    int uni_close(void);
    void uni_update(void);

private:
    int         m_uni_fd    { 0     };
    std::string m_uni_line;

    bool        m_base_flag { false };

    enum class HEAD_LOC_e : int8_t {
        NO = -1,
        NMEA = 0,
        RESP,
        RTCM_0,
        RTCM_1,
        RTCM_2,
        RTCM_3,
        END,
    };
    HEAD_LOC_e                          m_now_head      { HEAD_LOC_e::NO };
    std::vector<std::string::size_type> m_loc_vector;

    Pub_Helper<orb_gps_raw_gga_t>   *m_pub_gga_raw      { nullptr };
    Sub_Helper<orb_rtcm_t>          *m_sub_rtcm_ntrip   { nullptr };

    Pub_Helper<orb_rtcm_t>          *m_pub_rtcm_unicore { nullptr };

    void m_read_line(void);
    void m_set_base(minmea_sentence_gga &frame);
    void m_nmea_decode(void);
    void m_resp_decode(void);
    void m_rtcm_decode(std::size_t len);
    uint32_t m_crc_crc24(const uint8_t *bytes, uint16_t len);

};
