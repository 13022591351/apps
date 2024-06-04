#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <net/if.h>
#include <netinet/in.h>

#include "netutils/netlib.h"
#include "netutils/httpd.h"

#include "cgi.h"

int main(int argc, FAR char *argv[]) {
    printf("Starting webserver\n");

    httpd_init();
    cgi_register();
    httpd_listen();

    printf("webserver_main: Exiting\n");
    fflush(stdout);

    return 0;
}
