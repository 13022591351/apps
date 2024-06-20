#include "ntrip_client.h"
#include "ntrip_client_debug.h"

#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <memory.h>
#include <netutils/base64.h>

NtripClient::NtripClient() {

}

NtripClient::~NtripClient() {
    ntrip_close();
}

void NtripClient::set_url(std::string url) {
    m_url           = url;
}

void NtripClient::set_port(std::string port) {
    m_port          = port;
}

void NtripClient::set_mountpoint(std::string mountpoint) {
    m_mountpoint    = mountpoint;
}

void NtripClient::set_username(std::string username) {
    m_username      = username;
}

void NtripClient::set_password(std::string password) {
    m_password      = password;
}

void NtripClient::set_useragent(std::string useragent) {
    m_useragent     = useragent;
}

std::string NtripClient::get_url(void) {
    return m_url;
}

std::string NtripClient::get_port(void) {
    return m_port;
}

std::string NtripClient::get_mountpoint(void) {
    return m_mountpoint;
}

std::string NtripClient::get_username(void) {
    return m_username;
}

std::string NtripClient::get_password(void) {
    return m_password;
}

std::string NtripClient::get_useragent(void) {
    return m_useragent;
}

int NtripClient::ntrip_open(void) {
    int ret;

    while (true) {
        ret = m_net_init();
        if (ret >= 0) {
            break;
        }
    }

    m_sub_gga_raw    = new Sub_Helper<orb_gps_raw_gga_t>(ORB_ID(orb_gps_gga_raw));
    m_pub_rtcm_ntrip = new Pub_Helper<orb_rtcm_t>(ORB_ID(orb_rtcm_ntrip));

    m_pub_rtcm_ntrip->meta_data().rtcm_len = 0;
    m_last_conn_time = orb_absolute_time();
    m_last_recv_time = orb_absolute_time();

    m_last_conn_time = orb_absolute_time();

    m_ntrip_line.resize(NTRIP_MAX_LENGTH);

    return ret;
}

int NtripClient::ntrip_close(void) {
    m_net_deinit();

    if (m_sub_gga_raw) {
        delete m_sub_gga_raw;
        m_sub_gga_raw = nullptr;
    }

    if (m_pub_rtcm_ntrip) {
        delete m_pub_rtcm_ntrip;
        m_pub_rtcm_ntrip = nullptr;
    }

    return 0;
}

void NtripClient::ntrip_update(void) {
    const orb_abstime now = orb_absolute_time();

    // 网络重连标志
    if (m_re_conn_flag) {
        // 设置 网络重连标志 为 False
        m_re_conn_flag = false;
        // 设置 Ntrip连接状态 为 False
        m_ntrip_conn_flag = false;

        m_net_deinit();
        while (true) {
            if (m_net_init() >= 0) {
                break;
            }
        }
    }

    if (m_sock_fd > 0) {
        int     read_len                         { 0 };
        uint8_t reab_buf[NTRIP_MAX_LENGTH / 2] = { 0 };

        // 获取网络接收缓冲区数据长度
        ioctl(m_sock_fd, FIONREAD, &read_len);
        if (read_len > 0) {
            // 记录网络接收到数据的时间
            m_last_recv_time = now;
            read_len = read(m_sock_fd, reab_buf, NTRIP_MAX_LENGTH / 2);

            // 判断 Ntrip连接状态
            if (m_ntrip_conn_flag) {
                int free_len = NTRIP_MAX_LENGTH - static_cast<int>(m_ntrip_line.length());

                // 判断 协议解析缓冲 空余长度是否大于 接收到的数据长度
                if (free_len < read_len) {
                    // 从头清除大于空余长度的部分
                    m_ntrip_line.erase(read_len - free_len);
                }
                // 向 协议解析缓冲 追加读取到的数据
                m_ntrip_line.append(reinterpret_cast<char *>(reab_buf), read_len);

                m_rtcm_decode();
            }
            else {
                if (strstr(reinterpret_cast<char *>(reab_buf), "ICY 200 OK")) {
                    // 收到 "ICY 200 OK" Ntrip服务器鉴权成功
                    // 设置 Ntrip连接状态 为 True
                    m_ntrip_conn_flag = true;
                }
                dbg_info((char *)reab_buf);
            }
        }

        m_ntrip_decode();

        // 判断GGA数据订阅状态
        if (OK == m_sub_gga_raw->subscribe()) {
            // 判断 Ntrip连接状态
            if (m_ntrip_conn_flag) {
                decltype(m_sub_gga_raw->meta_data()) &topic = m_sub_gga_raw->meta_data();

                // 获取GGA数据长度
                size_t len = strlen(reinterpret_cast<char *>(topic.gga_raw));

                if (len > 0) {
                    minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, reinterpret_cast<char *>(topic.gga_raw))) {
                        // 确保GGA数据已定位
                        if (frame.fix_quality) {
                            char gga_str[100] = "";
                            snprintf(gga_str, len + 3, "%s\r\n", reinterpret_cast<char *>(topic.gga_raw));

                            // 判断发送网络发送出去的数据长度是否和GGA数据长度一致
                            if (write(m_sock_fd, gga_str, len + 3) != static_cast<int>(len + 3)) {
                                // 长度不一致 设置 网络重连标志 为 True
                                m_re_conn_flag = true;
                            }
                        }
                    }
                }
            }
        }
    }
}

int NtripClient::m_net_init(void) {
    addrinfo        hints;
    FAR addrinfo   *servinfo;
    FAR addrinfo   *itr;
    int             ret;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family     = AF_INET;
    hints.ai_socktype   = SOCK_STREAM;

    dbg_info("Connecting to %s:%s...\n", m_url.c_str(), m_port.c_str());

    ret = getaddrinfo(m_url.c_str(), m_port.c_str(), &hints, &servinfo);
    if (ret != OK) {
        dbg_err("ERROR! getaddrinfo() failed: %s\n", gai_strerror(ret));
        return -1;
    }

    itr = servinfo;
    do {
        m_sock_fd = socket(itr->ai_family, itr->ai_socktype, itr->ai_protocol);
        if (m_sock_fd < 0) {
            continue;
        }

        ret = connect(m_sock_fd, itr->ai_addr, itr->ai_addrlen);
        if (ret == 0) {
            break;
        }

        close(m_sock_fd);
        m_sock_fd = -1;
    }
    while ((itr = itr->ai_next) != NULL);

    freeaddrinfo(servinfo);

    if (m_sock_fd < 0) {
        dbg_err("ERROR! Couldn't create socket\n");
        return -1;
    }

    ret = fcntl(m_sock_fd, F_GETFL, 0);
    if (ret < 0) {
        dbg_err("ERROR! fcntl() F_GETFL failed, errno: %d\n", errno);
        return -1;
    }

    ret = fcntl(m_sock_fd, F_SETFL, ret | O_NONBLOCK);
    if (ret < 0) {
        dbg_err("ERROR! fcntl() F_SETFL failed, errno: %d\n", errno);
        return -1;
    }

    return m_sock_fd;
}

int NtripClient::m_net_deinit(void) {
    if (m_sock_fd) {
        close(m_sock_fd);
        m_sock_fd = 0;
    }

    return 0;
}

std::string NtripClient::m_base64_encode(const std::string &src) {
    std::string ret;
    size_t len  = src.length();
    size_t olen = (len + 2) / 3 * 4;

    ret.resize(olen);
    base64_encode((void *)src.c_str(), len, (void *)ret.c_str(), &olen);

    return ret;
}

uint32_t NtripClient::m_crc_crc24(const uint8_t *bytes, uint16_t len) {
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

void NtripClient::m_ntrip_decode(void) {
    const orb_abstime now = orb_absolute_time();

    // 如果 上次发起鉴权 时间大于1s Ntrip Ntrip连接状态 为 False
    if (now - m_last_conn_time > 1e+6 && !m_ntrip_conn_flag) {
        // 判断GPGGA数据是否为空
        if (strlen(reinterpret_cast<char *>(m_sub_gga_raw->meta_data().gga_raw))) {
            minmea_sentence_gga frame;

            if (minmea_parse_gga(&frame, reinterpret_cast<char *>(m_sub_gga_raw->meta_data().gga_raw))) {
                // 确保GGA数据已定位
                if (frame.fix_quality) {
                    std::string req;

                    req += "GET /" + m_mountpoint + " HTTP/1.0\r\n";
                    req += "User-Agent: " + m_useragent + "\r\n";

                    req += "Authorization: Basic " + m_base64_encode(m_username + ":" + m_password) + "\r\n";

                    req += "\r\n";

                    dbg_info(req.c_str());

                    // 判断发送网络发送出去的数据长度是否和鉴权请求数据长度一致
                    if (write(m_sock_fd, req.c_str(), req.length()) != static_cast<int>(req.length())) {
                        // 长度不一致 设置 网络重连标志 为 True
                        m_re_conn_flag = true;
                    }

                    // 更新 上次发起鉴权 时间
                    m_last_conn_time = now;
                }
            }
        }
    }
}

void NtripClient::m_rtcm_decode(void) {
    const orb_abstime now = orb_absolute_time();

    bool find_head { false };

    while (m_ntrip_line.length()) {
        // 遍历 协议解析缓冲 长度 减二
        for (int index = 0; m_ntrip_line.length() - index > 2; index++) {
            if (m_ntrip_line[index + 0] == 0xD3 && (m_ntrip_line[index + 1] & 0xFC) == 0x00) {
                find_head = true;
                m_ntrip_line.erase(0, index);
                break;
            }
        }

        if (find_head) {
            if (m_ntrip_line.length() < 6) {
                // 判断 协议解析缓冲 是否具备解析条件
                return;
            }

            // 解析 RTCM协议 帧长度
            std::size_t rtcm_len = ((m_ntrip_line[1] << 4) & 0x300) + m_ntrip_line[2] + 6;

            if (rtcm_len > NTRIP_MAX_LENGTH / 2) {
                // 帧长度 大于 缓冲区 一半长度
                // 清除第一个字节
                // 设置 当前帧头 为 无帧头
                m_ntrip_line.erase(0, 1);
                find_head = false;
                break;
            }

            if (m_ntrip_line.length() < rtcm_len) {
                // 协议解析缓冲 中数据长度不足
                // 退出循环 等待接收新的数据
                return;
            }

            uint16_t rtcm_id    = ((m_ntrip_line[3] << 4) & 0xFF0) + ((m_ntrip_line[4] >> 4) & 0x0F);
            uint32_t crc1       = (m_ntrip_line[rtcm_len - 3] << 16) | (m_ntrip_line[rtcm_len - 2] << 8) | m_ntrip_line[rtcm_len - 1];
            uint32_t crc2       = m_crc_crc24(reinterpret_cast<const uint8_t *>(m_ntrip_line.c_str()), rtcm_len - 3);

            if (crc1 != crc2) {
                // CRC校验错误 清除第一个字节
                // 设置 当前帧头 为 无帧头
                m_ntrip_line.erase(0, 1);
                find_head = false;
                break;
            }

            // Ntrip服务器已连接 收到的为RTCM数据 直接发布
            {
                decltype(m_pub_rtcm_ntrip->meta_data()) &topic = m_pub_rtcm_ntrip->meta_data();

                topic.timestamp = now;
                topic.rtcm_id   = rtcm_id;
                topic.rtcm_len  = rtcm_len;
                memcpy(topic.rtcm, m_ntrip_line.c_str(), rtcm_len);

                if (OK != m_pub_rtcm_ntrip->publish()) {
                    dbg_err("ERROR! orb_rtcm_ntrip Pub Failed\n");
                }
            }

            // 清除 当前数据帧
            // 设置 当前帧头 为 无帧头
            m_ntrip_line.erase(0, rtcm_len);
            find_head = false;
            break;
        }
        else {
            m_ntrip_line.erase(0, m_ntrip_line.length() - 2);
        }
    }
}