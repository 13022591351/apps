#pragma once

#include <nuttx/config.h>

#include <stdio.h>
#include <inttypes.h>

#if defined(CONFIG_DYW_UNICORE_DBG_ERR)
    #define dbg_err(...) printf(__VA_ARGS__)
#else
    #define dbg_err(...)
#endif

#if defined(CONFIG_DYW_UNICORE_DBG_INFO)
    #define dbg_info(...) printf(__VA_ARGS__)
#else
    #define dbg_info(...)
#endif
