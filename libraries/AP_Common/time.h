#pragma once
#include <time.h>  // ✅ Ensure this is high enough to expose time_t
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#ifndef __STDC_WANT_LIB_EXT1__
#define __STDC_WANT_LIB_EXT1__ 1
#endif

// replacement for mktime()


