#pragma once

#include <uORB/uORB.h>

#include <stdint.h>

template <class T>
class Pub_Helper {
public:
    explicit Pub_Helper(const orb_metadata *name) {
        m_name = const_cast<orb_metadata *>(name);
        m_fd   = orb_advertise_multi_queue_persist(name, &m_data, &m_instance, 1);
    }

    virtual ~Pub_Helper() {
        orb_unadvertise(m_fd);
    }

    int publish(void) {
        int ret = orb_publish(m_name, m_fd, &m_data);
        usleep(10);
        return ret;
    }

    T &meta_data(void) {
        return m_data;
    }

private:
    orb_metadata *m_name     { nullptr };
    int           m_fd       { 0       };
    T             m_data;
    int           m_instance { 0       };

};

template <class T>
class Sub_Helper {
public:
    explicit Sub_Helper(const orb_metadata *name) {
        m_name = const_cast<orb_metadata *>(name);
        m_fd   = orb_subscribe(name);
    }

    virtual ~Sub_Helper() {
        orb_unsubscribe(m_fd);
    }

    int subscribe(void) {
        bool updated { false                     };
        int  ret     { orb_check(m_fd, &updated) };

        if (ret == ERROR || !updated) {
            return ERROR;
        }

        return orb_copy(m_name, m_fd, &m_data);
    }

    T &meta_data(void) {
        return m_data;
    }

private:
    orb_metadata *m_name { nullptr };
    int           m_fd   { 0       };
    T             m_data;

};
