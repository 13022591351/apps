#include "unicore.h"
#include "unicore_debug.h"

#include <nuttx/crc32.h>

/*
RTCM1005 RTK 基准站天线参考点坐标（ARP）
RTCM1033 接收机与天线说明

RTCM1075 GPS     MSM5（全部伪距、载波、多普勒和 CNR 观测值）
RTCM1125 BDS     MSM5（全部伪距、载波、多普勒和 CNR 观测值）
RTCM1095 GALILEO MSM5（全部伪距、载波、多普勒和 CNR 观测值）
RTCM1085 GLONASS MSM5（全部伪距、载波、多普勒和 CNR 观测值）

RTCM1019 GPS     星历
RTCM1042 BDS     星历
RTCM1045 GALILEO F/NAV 星历
RTCM1046 GALILEO I/NAV 星历
RTCM1020 GLONASS 星历

OBSVMB 1\r\n\
GPSEPHB 10\r\n\
BDSEPHB 10\r\n\
GALEPHB 10\r\n\
GLOEPHB 10\r\n\
GPSIONB 10\r\n\
BDSIONB 10\r\n\
GALIONB 10\r\n\
PVTSLNB 1\r\n\
*/

#define LOG_FILE_PATH       "/mnt/sd0/log/"
#define UNI_SERIAL_DEVPATH  "/dev/ttyS2"
#define UNI_MAX_LENGTH      2560
#define UNI_CMD "\
UNLOG\r\n\
\
RTCM1005 10\r\n\
RTCM1033 10\r\n\
\
RTCM1075 1\r\n\
RTCM1125 1\r\n\
RTCM1095 1\r\n\
RTCM1085 1\r\n\
\
RTCM1019 0.05\r\n\
RTCM1042 0.05\r\n\
RTCM1045 0.05\r\n\
RTCM1046 0.05\r\n\
RTCM1020 0.05\r\n\
\
GPGGA 1\r\n\
GPZDA 1\r\n\
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
    // 清空接收缓冲区
    fcntl(m_uni_fd, TCFLSH, 0);
    // 发送板卡配置指令
    write(m_uni_fd, UNI_CMD, strlen(UNI_CMD));

    m_pub_gga_raw       = new Pub_Helper<orb_gps_raw_gga_t  >(ORB_ID(orb_gps_gga_raw));
    m_sub_rtcm_ntrip    = new Sub_Helper<orb_rtcm_t         >(ORB_ID(orb_rtcm_ntrip));
    m_pub_rtcm_unicore  = new Pub_Helper<orb_rtcm_t         >(ORB_ID(orb_rtcm_unicore));

    // 设置 协议解析缓冲 长度
    m_uni_line.reserve(UNI_MAX_LENGTH);

    return OK;
}

int Unicore::uni_close(void) {
    int ret { OK };

    if (m_uni_fd) {
        ret = close(m_uni_fd);
        m_uni_fd = 0;
    }

    if (m_log_fd) {
        ret = close(m_log_fd);
        m_log_fd = 0;
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
    // 判断 Ntrip RTCM数据 订阅状态
    if (OK == m_sub_rtcm_ntrip->subscribe()) {
        {
            decltype(m_sub_rtcm_ntrip->meta_data()) &topic = m_sub_rtcm_ntrip->meta_data();

            if (topic.rtcm_len > 0) {
                // 向板卡发送 Ntrip RTCM数据
                write(m_uni_fd, topic.rtcm, topic.rtcm_len);
            }
        }
    }

    m_read_line();
}

void Unicore::m_read_line(void) {
    int     read_len                       { 0 };
    uint8_t reab_buf[UNI_MAX_LENGTH / 2] = { 0 };

    // 获取串口接收缓冲区数据长度
    ioctl(m_uni_fd, FIONREAD, &read_len);
    if (read_len <= 0) {
        return;
    }
    read_len = read(m_uni_fd, reab_buf, UNI_MAX_LENGTH / 2);
    int free_len = UNI_MAX_LENGTH - static_cast<int>(m_uni_line.length());
    // 判断 协议解析缓冲 空余长度是否大于 接收到的数据长度
    if (free_len < read_len) {
        // 从头清除大于空余长度的部分
        // 设置 当前帧头 为 无帧头
        m_uni_line.erase(read_len - free_len);
        m_now_head = HEAD_LOC_e::NO;
    }
    // 向 协议解析缓冲 追加读取到的数据
    m_uni_line.append(reinterpret_cast<char *>(reab_buf), read_len);

    while (true) {
        if (m_now_head == HEAD_LOC_e::NO) {
            // 遍历 协议解析缓冲 长度 减三
            for (int index = 0; m_uni_line.length() - index > 3; index++) {
                if (m_uni_line[index + 0] == '$' && m_uni_line[index + 1] == 'G') {
                    m_now_head = HEAD_LOC_e::NMEA;
                    m_uni_line.erase(0, index);
                    break;
                }
                else if (m_uni_line[index + 0] == '$' && m_uni_line[index + 1] == 'c') {
                    m_now_head = HEAD_LOC_e::RESP;
                    m_uni_line.erase(0, index);
                    break;
                }
                else if (m_uni_line[index + 0] == 0xAA && m_uni_line[index + 1] == 0x44 && m_uni_line[index + 2] == 0xB5) {
                    m_now_head = HEAD_LOC_e::UNI;
                    m_uni_line.erase(0, index);
                    break;
                }
                else if (m_uni_line[index + 0] == 0xD3 && (m_uni_line[index + 1] & 0xFC) == 0x00) {
                    m_now_head = HEAD_LOC_e::RTCM;
                    m_uni_line.erase(0, index);
                    break;
                }
            }
            if (m_now_head == HEAD_LOC_e::NO) {
                // 清除 协议解析缓冲 被遍历的部分
                m_uni_line.erase(0, m_uni_line.length() - 3);
            }
        }

        switch (m_now_head) {
            case (HEAD_LOC_e::NO): {
                return;
            }
            case (HEAD_LOC_e::NMEA):
            case (HEAD_LOC_e::RESP): {
                std::string::size_type loc_rn = m_uni_line.find("\r\n");

                if (loc_rn == std::string::npos) {
                    // 未找到 帧尾
                    // 退出循环 等待接收新的数据
                    return;
                }

                if (loc_rn > 100) {
                    // 如果 帧尾 位置 大于100字节
                    // 清除第一个字节
                    // 设置 当前帧头 为 无帧头
                    m_uni_line.erase(0, 1);
                    m_now_head = HEAD_LOC_e::NO;
                    break;
                }

                // 将 帧尾 替换成 结束符
                m_uni_line[loc_rn] = '\0';

                if (m_now_head == HEAD_LOC_e::NMEA) {
                    m_nmea_decode();
                }

                if (m_now_head == HEAD_LOC_e::RESP) {
                    m_resp_decode();
                }

                // 清除 当前数据帧
                // 设置 当前帧头 为 无帧头
                m_uni_line.erase(0, loc_rn);
                m_now_head = HEAD_LOC_e::NO;
                break;
            }
            case (HEAD_LOC_e::UNI): {
                if (m_uni_line.length() < 28) {
                    // 判断 协议解析缓冲 是否具备解析条件
                    return;
                }

                // 解析 UniCore协议 帧长度
                std::size_t uni_len = (((m_uni_line[7] << 8) & 0xFF00) | m_uni_line[6]) + 28;

                if (uni_len > UNI_MAX_LENGTH / 2) {
                    // 帧长度 大于 缓冲区 一半长度
                    // 清除第一个字节
                    // 设置 当前帧头 为 无帧头
                    m_uni_line.erase(0, 1);
                    m_now_head = HEAD_LOC_e::NO;
                    break;
                }

                if (m_uni_line.length() < uni_len) {
                    // 协议解析缓冲 中数据长度不足
                    // 退出循环 等待接收新的数据
                    return;
                }

                uint32_t crc1 = (m_uni_line[uni_len - 1] << 24) | (m_uni_line[uni_len - 2] << 16) | (m_uni_line[uni_len - 3] << 8) | m_uni_line[uni_len - 4];
                uint32_t crc2 = crc32(reinterpret_cast<const uint8_t *>(m_uni_line.c_str()), uni_len - 4);
                if (crc1 != crc2) {
                    // CRC校验错误 清除第一个字节
                    // 设置 当前帧头 为 无帧头
                    m_uni_line.erase(0, 1);
                    m_now_head = HEAD_LOC_e::NO;
                    break;
                }

                m_uni_decode(uni_len);

                // 清除 当前数据帧
                // 设置 当前帧头 为 无帧头
                m_uni_line.erase(0, uni_len);
                m_now_head = HEAD_LOC_e::NO;
                break;
            }
            case (HEAD_LOC_e::RTCM): {
                if (m_uni_line.length() < 6) {
                    // 判断 协议解析缓冲 是否具备解析条件
                    return;
                }

                // 解析 RTCM协议 帧长度
                std::size_t rtcm_len = ((m_uni_line[1] << 4) & 0x300) + m_uni_line[2] + 6;

                if (rtcm_len > UNI_MAX_LENGTH / 2) {
                    // 帧长度 大于 缓冲区 一半长度
                    // 清除第一个字节
                    // 设置 当前帧头 为 无帧头
                    m_uni_line.erase(0, 1);
                    m_now_head = HEAD_LOC_e::NO;
                    break;
                }

                if (m_uni_line.length() < rtcm_len) {
                    // 协议解析缓冲 中数据长度不足
                    // 退出循环 等待接收新的数据
                    return;
                }

                uint32_t crc1 = (m_uni_line[rtcm_len - 3] << 16) | (m_uni_line[rtcm_len - 2] << 8) | m_uni_line[rtcm_len - 1];
                uint32_t crc2 = m_crc_crc24(reinterpret_cast<const uint8_t *>(m_uni_line.c_str()), rtcm_len - 3);

                if (crc1 != crc2) {
                    // CRC校验错误 清除第一个字节
                    // 设置 当前帧头 为 无帧头
                    m_uni_line.erase(0, 1);
                    m_now_head = HEAD_LOC_e::NO;
                    break;
                }

                m_rtcm_decode(rtcm_len);

                // 清除 当前数据帧
                // 设置 当前帧头 为 无帧头
                m_uni_line.erase(0, rtcm_len);
                m_now_head = HEAD_LOC_e::NO;
                break;
            }
            default: {
                // 清除 协议解析缓冲
                // 设置 当前帧头 为 无帧头
                m_uni_line.clear();
                m_now_head = HEAD_LOC_e::NO;
                return;
            }
        }
    }
}

void Unicore::m_set_base(minmea_sentence_gga &frame) {
    if (!m_base_flag) {
        return;
    }

    if (frame.fix_quality == 4) {
        char cmd[]  = "MODE BASE TIME 10\r\n";
        int  len    = strlen(cmd);

        if (len == write(m_uni_fd, cmd, len)) {
            m_base_flag = true;
        }
    }
}

void Unicore::m_log_file(minmea_sentence_zda &frame) {
    if (m_log_fd) {
        fsync(m_log_fd);
        return;
    }
    else {
        if (frame.date.year > 2000) {
            char file_path[32] = { 0 };
            char file_name[32] = { 0 };

            sprintf(file_path, "%s%" PRIi16 "_%" PRIi16 "_%" PRIi16 "",
                    LOG_FILE_PATH,
                    frame.date.year,
                    frame.date.month,
                    frame.date.day);
            sprintf(file_name, "%" PRIi16 "_%" PRIi16 "_%" PRIi16 "-%" PRIi16 "_%" PRIi16 "_%" PRIi16 ".rtcm3",
                    frame.date.year,
                    frame.date.month,
                    frame.date.day,
                    frame.time.hours,
                    frame.time.minutes,
                    frame.time.seconds);
            sprintf(m_log_file_path, "%s/%s", file_path, file_name);

            // 判断根据日期创建的文件夹是否存在
            // 不存在就新建一个
            struct stat file_path_stat;
            if ((stat(file_path, &file_path_stat) == 0) && (((file_path_stat.st_mode) & S_IFMT) == S_IFDIR)) {
                dbg_info("ttimestamp: %" PRIu64 " %s Existing\n", orb_absolute_time(), file_path);
            }
            else {
                if (mkdir(file_path, 0777) != OK) {
                    dbg_info("ttimestamp: %" PRIu64 " Create Dir  %s Failed\n", orb_absolute_time(), file_path);
                    return;
                }
                else {
                    dbg_info("ttimestamp: %" PRIu64 " Create Dir  %s Success\n", orb_absolute_time(), file_path);
                }
            }

            m_log_fd = open(m_log_file_path, O_WRONLY | O_CREAT | O_APPEND);
            if (m_log_fd < 0) {
                dbg_info("ttimestamp: %" PRIu64 " Create File %s Failed\n", orb_absolute_time(), file_name);
                m_log_fd = 0;
                return;
            }
            else {
                dbg_info("ttimestamp: %" PRIu64 " Create File %s Success\n", orb_absolute_time(), file_name);
            }
        }
    }
}

void Unicore::m_nmea_decode(void) {
    switch (minmea_sentence_id(m_uni_line.c_str(), false)) {
        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;

            if (minmea_parse_gga(&frame, m_uni_line.c_str())) {
                dbg_info("ttimestamp: %" PRIu64 " %s\n", orb_absolute_time(), m_uni_line.c_str());

                {
                    decltype(m_pub_gga_raw->meta_data()) &topic = m_pub_gga_raw->meta_data();

                    // char *gga = "$GNGGA,105622.00,3149.74589713,N,11846.05175760,E,1,28,0.6,17.3871,M,2.1321,M,,*74";
                    // strcpy(reinterpret_cast<char *>(topic.gga_raw), gga);

                    strcpy(reinterpret_cast<char *>(topic.gga_raw), m_uni_line.c_str());
                    topic.timestamp = orb_absolute_time();
                    if (OK != m_pub_gga_raw->publish()) {
                        dbg_err("ERROR! orb_gps_gga_raw Pub Failed\n");
                    }
                }

                m_set_base(frame);
            }
            else {
                dbg_info("$xxGGA sentence is not parsed\n");
            }

            break;
        }
        case MINMEA_SENTENCE_ZDA: {
            struct minmea_sentence_zda frame;

            if (minmea_parse_zda(&frame, m_uni_line.c_str())) {
                dbg_info("ttimestamp: %" PRIu64 " %s\n", orb_absolute_time(), m_uni_line.c_str());

                m_log_file(frame);
            }
            else {
                dbg_info("$xxZDA sentence is not parsed\n");
            }

            break;
        }
        default :{
            break;
        }
    }
}

void Unicore::m_resp_decode(void) {
    dbg_info("ttimestamp: %" PRIu64 " %s\n", orb_absolute_time(), m_uni_line.c_str());
}

void Unicore::m_uni_decode(std::size_t len) {
    uint16_t uni_id = ((m_uni_line[5] << 8) & 0xFF00) | m_uni_line[4];

    switch (uni_id) {
        case (static_cast<uint16_t>(UNI_ID_e::OBSVMB)):  { 
            dbg_info("ttimestamp: %" PRIu64 "  OBSVMB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::GPSEPHB)): { 
            dbg_info("ttimestamp: %" PRIu64 " GPSEPHB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::GLOEPHB)): { 
            dbg_info("ttimestamp: %" PRIu64 " GLOEPHB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::BDSEPHB)): { 
            dbg_info("ttimestamp: %" PRIu64 " BDSEPHB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::GALEPHB)): { 
            dbg_info("ttimestamp: %" PRIu64 " GALEPHB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::GPSIONB)): { 
            dbg_info("ttimestamp: %" PRIu64 " GPSIONB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::BDSIONB)): { 
            dbg_info("ttimestamp: %" PRIu64 " BDSIONB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::GALIONB)): { 
            dbg_info("ttimestamp: %" PRIu64 " GALIONB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        case (static_cast<uint16_t>(UNI_ID_e::PVTSLNB)): { 
            dbg_info("ttimestamp: %" PRIu64 " PVTSLNB len: %" PRIu16 "\n", orb_absolute_time(), len);
            break;
        }
        default: {
            break;
        }
    }
}

void Unicore::m_rtcm_decode(std::size_t len) {
    uint16_t rtcm_id = ((m_uni_line[3] << 4) & 0xFF0) + ((m_uni_line[4] >> 4) & 0x0F);

    if (m_log_fd) {
        write(m_log_fd, m_uni_line.c_str(), len);
    }

    // dbg_info("ttimestamp: %" PRIu64 "  id: %" PRIu16 " len: %" PRIu16 "\n", orb_absolute_time(), rtcm_id, len);

    if (rtcm_id == 1005 ||
        rtcm_id == 1033 ||
        rtcm_id == 1075 ||
        rtcm_id == 1125 ||
        rtcm_id == 1095 ||
        rtcm_id == 1085) {
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
