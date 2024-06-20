#pragma once

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <wchar.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

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
    int         m_uni_fd            { 0     };
    std::string m_uni_line;

    int         m_log_fd            { 0     };
    char        m_log_file_path[64] { 0     };

    bool        m_base_flag         { false };

    enum class UNI_ID_e : uint16_t {
        OBSVMB  = 12,
        GPSEPHB = 106,
        GLOEPHB = 107,
        BDSEPHB = 108,
        GALEPHB = 109,
        GPSIONB = 8,
        BDSIONB = 4,
        GALIONB = 9,
        PVTSLNB = 1021,
    };
    enum class HEAD_LOC_e : int8_t {
        NO  = -1,
        NMEA = 0,
        RESP,
        UNI,
        RTCM,
        END,
    };
    HEAD_LOC_e                       m_now_head         { HEAD_LOC_e::NO };

    Pub_Helper<orb_gps_raw_gga_t>   *m_pub_gga_raw      { nullptr };
    Sub_Helper<orb_rtcm_t>          *m_sub_rtcm_ntrip   { nullptr };

    Pub_Helper<orb_rtcm_t>          *m_pub_rtcm_unicore { nullptr };

    void m_read_line(void);
    void m_set_base(minmea_sentence_gga &frame);
    void m_log_file(minmea_sentence_zda &frame);
    void m_nmea_decode(void);
    void m_resp_decode(void);
    void m_uni_decode(std::size_t len);
    void m_rtcm_decode(std::size_t len);
    uint32_t m_crc_crc24(const uint8_t *bytes, uint16_t len);

};
