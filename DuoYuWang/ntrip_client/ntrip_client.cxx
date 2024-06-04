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

    ret = m_net_init();
    if (ret < 0) {
        dbg_err("ERROR! Net Init Failed\n");
        return -1;
    }

    m_sub_gga_raw    = new Sub_Helper<orb_gps_raw_gga_t>(ORB_ID(orb_gps_gga_raw));
    m_pub_rtcm_ntrip = new Pub_Helper<orb_rtcm_t>(ORB_ID(orb_rtcm_ntrip));

    m_pub_rtcm_ntrip->meta_data().rtcm_len = 0;
    m_last_conn_time = orb_absolute_time();
    m_last_recv_time = orb_absolute_time();

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
    if (m_sock_fd > 0) {


        m_ntrip_decode();

        if (OK == m_sub_gga_raw->subscribe()) {
            if (m_ntrip_conn_flag) {
                size_t len = strlen(reinterpret_cast<char *>(m_sub_gga_raw->meta_data().gga_raw));

                if (len > 0) {
                    minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, reinterpret_cast<char *>(m_sub_gga_raw->meta_data().gga_raw))) {
                        if (frame.fix_quality) {
                            write(m_sock_fd, m_sub_gga_raw->meta_data().gga_raw, len);
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

void NtripClient::m_ntrip_decode(void) {
    const orb_abstime now = orb_absolute_time();

    int     read_len                     { 0 };
    uint8_t reab_buf[NTRIP_MAX_LENGTH] = { 0 };

    ioctl(m_sock_fd, FIONREAD, &read_len);
    if (read_len > 0) {
        m_last_recv_time = now;
        read_len = read(m_sock_fd, reab_buf, NTRIP_MAX_LENGTH);
        if (m_ntrip_conn_flag) {
            {
                decltype(m_pub_rtcm_ntrip->meta_data()) &topic = m_pub_rtcm_ntrip->meta_data();

                topic.timestamp = now;
                topic.rtcm_len  = read_len;
                memcpy(topic.rtcm, reab_buf, read_len);
                if (OK != m_pub_rtcm_ntrip->publish()) {
                    dbg_err("ERROR! orb_rtcm_ntrip Pub Failed\n");
                }
            }
        }
        else {
            if (strstr(reinterpret_cast<char *>(reab_buf), "ICY 200 OK")) {
                m_ntrip_conn_flag = true;
            }
        }
    }

    if (now - m_last_recv_time > 5e+6 && m_ntrip_conn_flag) {
        m_ntrip_conn_flag = false;
    }

    if (now - m_last_conn_time > 1e+6 && !m_ntrip_conn_flag) {

        if (strlen(reinterpret_cast<char *>(m_sub_gga_raw->meta_data().gga_raw))) {

            minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, reinterpret_cast<char *>(m_sub_gga_raw->meta_data().gga_raw))) {
                if (frame.fix_quality) {
                    if (m_re_conn_flag) {
                        m_re_conn_flag = false;
                        m_net_deinit();
                        m_net_init();
                    }

                    std::string req;
                    req += "GET /";
                    req += m_mountpoint;
                    req += " HTTP/1.1\r\n";
                    req += "User-Agent: ";
                    req += m_useragent;
                    req += "\r\nAccept: */*\r\n";
                    req += "Connection: close\r\n";
                    req += "Authorization: Basic ";
                    req += m_base64_encode(m_username + ":" + m_password);

                    if (write(m_sock_fd, req.c_str(), req.length()) != static_cast<int>(req.length())) {
                        m_re_conn_flag = true;
                    }

                    m_last_conn_time = now;
                }
            }
        }
    }
}
