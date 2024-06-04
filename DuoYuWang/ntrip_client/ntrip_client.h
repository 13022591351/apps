#pragma once

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <minmea/minmea.h>
#include <uorb_fmu.h>
#include <uorb_helper.h>

#include <string>

#define NTRIP_MAX_LENGTH 1200

class NtripClient {
public:
    explicit NtripClient();
    virtual ~NtripClient();

    void set_url(std::string url);
    void set_port(std::string port);
    void set_mountpoint(std::string mountpoint);
    void set_username(std::string username);
    void set_password(std::string password);
    void set_useragent(std::string useragent);

    std::string get_url(void);
    std::string get_port(void);
    std::string get_mountpoint(void);
    std::string get_username(void);
    std::string get_password(void);
    std::string get_useragent(void);

    int ntrip_open(void);
    int ntrip_close(void);
    void ntrip_update(void);

private:
    std::string m_url;
    std::string m_port;
    std::string m_mountpoint;
    std::string m_username;
    std::string m_password;
    std::string m_useragent;
    std::string m_gga;

    sockaddr_in                     m_server;
    int                             m_sock_fd           { 0       };

    int m_net_init(void);
    int m_net_deinit(void);

    bool        m_re_conn_flag                          { false   };
    bool        m_ntrip_conn_flag                       { false   };
    orb_abstime m_last_conn_time                        { 0       };
    orb_abstime m_last_recv_time                        { 0       };

    std::string m_base64_encode(const std::string &src);
    void        m_ntrip_decode(void);

    Sub_Helper<orb_gps_raw_gga_t>  *m_sub_gga_raw       { nullptr };
    Pub_Helper<orb_rtcm_t>         *m_pub_rtcm_ntrip    { nullptr };

};