#include <nuttx/config.h>

#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <fcntl.h>
#include <wchar.h>

#include <nuttx/clock.h>
#include <stdio.h>

#include <memory>

#include <uorb_fmu.h>
#include <uorb_helper.h>

#include "rtcm3_proxy_debug.h"


#define RTCM_SERIAL_DEVPATH "/dev/ttyS0"

extern "C" int main(int argc, FAR char *argv[]) {
    dbg_info("Hello, RTCM3 Proxy !!\n");

    int rtcm_fd = open(RTCM_SERIAL_DEVPATH, O_RDWR | O_NONBLOCK);
    if (rtcm_fd < 0) {
        dbg_err("Unable to open file %s\n", RTCM_SERIAL_DEVPATH);
        return -1;
    }
    fcntl(rtcm_fd, F_SETFL, 0);

    Sub_Helper<orb_rtcm_t> sub_rtcm_unicore(ORB_ID(orb_rtcm_unicore));

    while (true) {
        usleep(2 * 1000);

        if (OK == sub_rtcm_unicore.subscribe()) {
            {
                decltype(sub_rtcm_unicore.meta_data()) &topic = sub_rtcm_unicore.meta_data();

                if (topic.rtcm_len > 6) {
                    const orb_abstime now = orb_absolute_time();

                    dbg_info("orb_rtcm_unicore : ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) RTCM  id: %" PRIu16 "\n",
                            topic.timestamp,
                            now - topic.timestamp,
                            topic.rtcm_id);

                    write(rtcm_fd, topic.rtcm, topic.rtcm_len);
                }
            }
        }
    }

    close(rtcm_fd);

    return 0;
}
