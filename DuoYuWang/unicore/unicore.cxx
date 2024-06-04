#include "unicore.h"
#include "unicore_debug.h"

// RTCM1005 RTK 基准站天线参考点坐标（ARP）
// RTCM1033 接收机与天线说明
// RTCM1125 BDS MSM5（全部伪距、载波、多普勒和 CNR 观测值）
// RTCM1075 GPS MSM5（全部伪距、载波、多普勒和 CNR 观测值）
// RTCM1085 GLONASS MSM5（全部伪距、载波、多普勒和 CNR 观测值）

#define UNI_SERIAL_DEVPATH  "/dev/ttyS2"
#define UNI_MAX_LENGTH      2560
#define UNI_CMD "\
UNLOG\r\n\
GPGGA 1\r\n\
GPGSA 1\r\n\
GPGSV 1\r\n\
GPVTG 1\r\n\
\r\n"

Unicore::Unicore(void) {

}

Unicore::~Unicore(void) {
    uni_close();
}

int Unicore::uni_open(void) {
    m_uni_fd = open(UNI_SERIAL_DEVPATH, O_RDWR | O_NONBLOCK);
    if (m_uni_fd == ERROR) {
        dbg_err("Unable to open file %s\n", UNI_SERIAL_DEVPATH);
        return ERROR;
    }

    fcntl(m_uni_fd, F_SETFL, 0);

    usleep(10 * 1000);
    fcntl(m_uni_fd, TCFLSH, 0);
    write(m_uni_fd, UNI_CMD, strlen(UNI_CMD));

    m_pub_gga_raw       = new Pub_Helper<orb_gps_raw_gga_t  >(ORB_ID(orb_gps_gga_raw));
    m_sub_rtcm_ntrip    = new Sub_Helper<orb_rtcm_t         >(ORB_ID(orb_rtcm_ntrip));
    m_pub_rtcm_unicore  = new Pub_Helper<orb_rtcm_t         >(ORB_ID(orb_rtcm_unicore));

    m_uni_line.reserve(UNI_MAX_LENGTH);
    m_loc_vector.resize(static_cast<std::size_t>(HEAD_LOC_e::END));

    return OK;
}

int Unicore::uni_close(void) {
    int ret { OK };

    if (m_uni_fd) {
        ret = close(m_uni_fd);
        m_uni_fd = 0;
    }

    if (m_pub_gga_raw) {
        delete m_pub_gga_raw;
        m_pub_gga_raw = nullptr;
    }

    if (m_sub_rtcm_ntrip) {
        delete m_sub_rtcm_ntrip;
        m_sub_rtcm_ntrip = nullptr;
    }

    if (m_pub_rtcm_unicore) {
        delete m_pub_rtcm_unicore;
        m_pub_rtcm_unicore = nullptr;
    }

    return ret;
}

void Unicore::uni_update(void) {
    if (OK == m_sub_rtcm_ntrip->subscribe()) {
        {
            decltype(m_sub_rtcm_ntrip->meta_data()) &topic = m_sub_rtcm_ntrip->meta_data();
            if (topic.rtcm_len > 0) {
                write(m_uni_fd, topic.rtcm, topic.rtcm_len);
            }
        }
    }

    m_read_line();
}

void Unicore::m_read_line(void) {
    // char data1[25]  = {
    //     0xD3, 0x00, 0x13, 0x3E, 0xD7, 0xD3, 0x02, 0x02, 0x98, 0x0E, 0xDE, 0xEF, 0x34, 0xB4,
    //     0xBD, 0x62, 0xAC, 0x09, 0x41, 0x98, 0x6F, 0x33, 0x36, 0x0B, 0x98
    // };
    // char data2[144] = {
    //     0xD3, 0x00, 0x8A, 0x43, 0x20, 0x00, 0x40, 0x7F, 0x79, 0x82, 0x00, 0x20, 0x00, 0x22,
    //     0x80, 0x65, 0x80, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x7F, 0xFF, 0xA7, 0x22,
    //     0x26, 0x26, 0x22, 0xA6, 0xA2, 0xA3, 0x20, 0xFD, 0xDC, 0x05, 0x9F, 0x5B, 0x1B, 0xC6,
    //     0x36, 0x1C, 0x86, 0x77, 0x0E, 0x32, 0x33, 0x7C, 0x61, 0x97, 0xB4, 0x0F, 0x5E, 0x7F,
    //     0xE6, 0xBF, 0xDF, 0xF8, 0x73, 0xF1, 0x3A, 0x5F, 0x88, 0xBD, 0x49, 0x6B, 0x82, 0xBC,
    //     0xA6, 0xC4, 0xCD, 0x85, 0x86, 0xFD, 0xF4, 0x1A, 0xC0, 0xFF, 0xB8, 0x38, 0x01, 0x77,
    //     0xCC, 0x78, 0x42, 0x7D, 0xEC, 0xC5, 0x40, 0x18, 0xA1, 0x81, 0x7B, 0xEC, 0x86, 0x04,
    //     0x76, 0x0F, 0xEE, 0x28, 0x53, 0x6E, 0xE0, 0x84, 0x36, 0x09, 0x22, 0x26, 0x0C, 0x72,
    //     0x80, 0xD3, 0x4C, 0xC2, 0x8E, 0x7A, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    //     0x80, 0x00, 0x57, 0x4E, 0x18, 0x59, 0x3D, 0x75, 0xE5, 0x8D, 0xD3, 0xE7, 0x86, 0x58,
    //     0x80, 0x71, 0xCE, 0x42
    // };
    // m_uni_line.append(data1, 25);
    // m_uni_line.append(data2, 144);

    int     read_len                       { 0 };
    uint8_t reab_buf[UNI_MAX_LENGTH / 2] = { 0 };

    ioctl(m_uni_fd, FIONREAD, &read_len);
    if (read_len <= 0) {
        return;
    }
    read_len = read(m_uni_fd, reab_buf, UNI_MAX_LENGTH / 2);
    if (static_cast<int>(m_uni_line.length()) > UNI_MAX_LENGTH - read_len) {
        m_uni_line.erase(read_len);
    }
    m_uni_line.append(reinterpret_cast<char *>(reab_buf), read_len);

    while (true) {
        if (m_now_head == HEAD_LOC_e::NO) {
            m_loc_vector[static_cast<uint8_t>(HEAD_LOC_e::NMEA)]   = m_uni_line.find("$G");
            m_loc_vector[static_cast<uint8_t>(HEAD_LOC_e::RESP)]   = m_uni_line.find("$c");
            m_loc_vector[static_cast<uint8_t>(HEAD_LOC_e::RTCM_0)] = m_uni_line.find("\323\000");
            m_loc_vector[static_cast<uint8_t>(HEAD_LOC_e::RTCM_1)] = m_uni_line.find("\323\001");
            m_loc_vector[static_cast<uint8_t>(HEAD_LOC_e::RTCM_2)] = m_uni_line.find("\323\002");
            m_loc_vector[static_cast<uint8_t>(HEAD_LOC_e::RTCM_3)] = m_uni_line.find("\323\003");

            for (std::vector<std::size_t>::iterator iter = m_loc_vector.begin(); iter != m_loc_vector.end(); iter++) {
                if (*iter != std::string::npos) {
                    m_now_head = static_cast<HEAD_LOC_e>(std::distance(m_loc_vector.begin(), iter));
                    if (std::string::npos != 0) {
                        m_uni_line.erase(0, *iter);
                    }
                    break;
                }
            }
            if (m_now_head == HEAD_LOC_e::NO) {
                m_uni_line.clear();
                return;
            }
        }

        switch (m_now_head) {
            case (HEAD_LOC_e::NMEA):
            case (HEAD_LOC_e::RESP): {
                std::string::size_type loc_rn = m_uni_line.find("\r\n");
                if (loc_rn == std::string::npos) {
                    return;
                }
                m_uni_line[loc_rn] = '\0';

                if (m_now_head == HEAD_LOC_e::NMEA) {
                    m_nmea_decode();
                }

                if (m_now_head == HEAD_LOC_e::RESP) {
                    m_resp_decode();
                }

                m_uni_line.erase(0, loc_rn);
                m_now_head = HEAD_LOC_e::NO;
                break;
            }
            case (HEAD_LOC_e::RTCM_0):
            case (HEAD_LOC_e::RTCM_1):
            case (HEAD_LOC_e::RTCM_2):
            case (HEAD_LOC_e::RTCM_3): {
                if (m_uni_line.length() < 6) {
                    return;
                }

                std::size_t rtcm_len = ((m_uni_line[1] << 4) & 0xF00) + m_uni_line[2] + 6;

                if (m_uni_line.length() < rtcm_len) {
                    return;
                }

                m_rtcm_decode(rtcm_len);

                m_uni_line.erase(0, rtcm_len);
                m_now_head = HEAD_LOC_e::NO;
                break;
            }
            default: {
                m_uni_line.clear();
                m_now_head = HEAD_LOC_e::NO;
                break;
            }
        }
    }
}

void Unicore::m_set_base(minmea_sentence_gga &frame) {
    if (!m_base_flag) {
        return;
    }

    if (frame.fix_quality == 2) {
        char    cmd[]   = "MODE BASE TIME 10\r\n";
        size_t  len     = strlen(cmd);

        if (len == write(m_uni_fd, cmd, len)) {
            m_base_flag = true;
        }
    }
}

void Unicore::m_nmea_decode(void) {
    switch (minmea_sentence_id(m_uni_line.c_str(), false)) {
        case (MINMEA_SENTENCE_RMC): {   // GPRMC 卫星定位信息
            minmea_sentence_rmc frame;

            if (minmea_parse_rmc(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxRMC sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_GGA): {   // GPGGA 卫星定位信息
            minmea_sentence_gga frame;

            if (minmea_parse_gga(&frame, m_uni_line.c_str())) {
                {
                    decltype(m_pub_gga_raw->meta_data()) &topic = m_pub_gga_raw->meta_data();

                    strcpy(reinterpret_cast<char *>(topic.gga_raw), m_uni_line.c_str());
                    // strcpy(reinterpret_cast<char *>(topic.gga_raw), "$GNGGA,044744.00,3122.4658,N,12025.2791,E,1,10,3.00,12.575,M,7.100,M,00,0000*5F");
                    topic.timestamp = orb_absolute_time();
                    if (OK != m_pub_gga_raw->publish()) {
                        dbg_err("ERROR! orb_gps_gga_raw Pub Failed\n");
                    }
                }

                m_set_base(frame);

                dbg_info("Fix quality....................: %d\n",  frame.fix_quality);
                dbg_info("Altitude.......................: %ld\n", frame.altitude.value);
                dbg_info("Tracked satellites.............: %d\n",  frame.satellites_tracked);
            }
            else {
                dbg_info("$xxGGA sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_GSA): {   // GPGSA 参与定位解算的卫星信息
            minmea_sentence_gsa frame;

            if (minmea_parse_gsa(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxGSA sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_GLL): {   // GPGLL 地理位置信息
            minmea_sentence_gll frame;

            if (minmea_parse_gll(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxGLL sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_GST): {   // GPGST 伪距观测误差信息
            minmea_sentence_gst frame;

            if (minmea_parse_gst(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxGST sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_GSV): {   // GPGSV 可视卫星信息
            minmea_sentence_gsv frame;

            if (minmea_parse_gsv(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxGSV sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_GBS): {   // GPGBS 卫星故障检测信息
            minmea_sentence_gbs frame;

            if (minmea_parse_gbs(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxGBS sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_VTG): {   // GPVTG 地面航向与速度信息
            minmea_sentence_vtg frame;

            if (minmea_parse_vtg(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxVTG sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_SENTENCE_ZDA): {   // GPZDA 日期和时间
            minmea_sentence_zda frame;

            if (minmea_parse_zda(&frame, m_uni_line.c_str())) {

            }
            else {
                dbg_info("$xxZDA sentence is not parsed\n");
            }

            break;
        }
        case (MINMEA_INVALID): {
            
            break;
        }
        case (MINMEA_UNKNOWN): {
            
            break;
        }
    }
}

void Unicore::m_resp_decode(void) {
    dbg_info("%s\n", m_uni_line.c_str());
}

void Unicore::m_rtcm_decode(std::size_t len) {
    uint32_t crc1 = (m_uni_line[len - 3] << 16) | (m_uni_line[len - 2] << 8) | m_uni_line[len - 1];
    uint32_t crc2 = m_crc_crc24(reinterpret_cast<const uint8_t *>(m_uni_line.c_str()), len - 3);

    if (crc1 != crc2) {
        return;
    }

    uint16_t rtcm_id = ((m_uni_line[3] << 4) & 0xFF0) + ((m_uni_line[4] >> 4) & 0x0F);

    {
        decltype(m_pub_rtcm_unicore->meta_data()) &topic = m_pub_rtcm_unicore->meta_data();

        topic.timestamp = orb_absolute_time();
        topic.rtcm_id   = rtcm_id;
        topic.rtcm_len  = len;
        memcpy(topic.rtcm, m_uni_line.c_str(), len);
        if (OK != m_pub_rtcm_unicore->publish()) {
            dbg_err("ERROR! orb_rtcm_unicore Pub Failed\n");
        }
    }
}

uint32_t Unicore::m_crc_crc24(const uint8_t *bytes, uint16_t len) {
    static constexpr uint32_t   POLYCRC24   = 0x1864CFB;
    uint32_t                    crc         = 0;

    while (len--) {
        uint8_t         b       = *bytes++;
        const uint8_t   idx     = (crc>>16) ^ b;
        uint32_t        crct    = idx<<16;
    
        for (uint8_t j = 0; j < 8; j++) {
            crct <<= 1;
            if (crct & 0x1000000) {
                crct ^= POLYCRC24;
            }
        }
        crc = ((crc << 8) & 0xFFFFFF) ^ crct;
    }

    return crc;
}
