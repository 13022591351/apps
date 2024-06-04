#include <nuttx/config.h>

#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>

#include <nuttx/clock.h>
#include <stdio.h>

#include <memory>

#include "unicore.h"
#include "unicore_debug.h"

extern "C" int main(int argc, FAR char *argv[]) {
    dbg_info("Hello, UNICORE !!\n");

    Unicore uni;

    uni.uni_open();

    while (true) {
        uni.uni_update();
        usleep(2 * 1000);
    }

    return 0;
}
