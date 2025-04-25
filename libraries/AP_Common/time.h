#pragma once
#include ctime  // ✅ Ensure this is high enough to expose time_t
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#ifndef __STDC_WANT_LIB_EXT1__
#define __STDC_WANT_LIB_EXT1__ 1
#endif

// replacement for mktime()
std::time_t ap_mktime(const struct tm *t);

