#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include <nuttx/net/netstats.h>
#include "netutils/httpd.h"

#include "cgi.h"

void cgi_abc(FAR struct httpd_state *p_hs, char *name) {
    printf("cgi_abc\r\n");
}

void cgi_led(FAR struct httpd_state *p_hs, char *name) {
    printf("LEDControl.cgi\r\n");
}

static struct httpd_cgi_call a[] = {
    { NULL, "/abc", cgi_abc },
    { NULL, "/LEDControl.cgi", cgi_led }
};

void cgi_register(void) {
    for (int i = 0; i < nitems(a); i++) {
        httpd_cgi_register(&a[i]);
    }
}

