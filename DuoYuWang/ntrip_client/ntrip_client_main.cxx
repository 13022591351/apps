#include <nuttx/config.h>

#include <stdlib.h>
#include <time.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <stdio.h>

#include <string>

#include "ntrip_client.h"
#include "ntrip_client_debug.h"

extern "C" int main(int argc, FAR char *argv[]) {
    if (argc != 7) {
        return 0;
    }

    NtripClient ntrip_client;

    ntrip_client.set_url(argv[1]);
    ntrip_client.set_port(argv[2]);
    ntrip_client.set_mountpoint(argv[3]);
    ntrip_client.set_useragent(argv[4]);
    ntrip_client.set_username(argv[5]);
    ntrip_client.set_password(argv[6]);

    ntrip_client.ntrip_open();

    dbg_info("Hello, Ntrip Client !!\n");

    while (true) {
        ntrip_client.ntrip_update();
        usleep(2 * 1000);
    }
    
    return 0;
}
