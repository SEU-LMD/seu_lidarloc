#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <sys/time.h>
#include <core/timer.h>

inline Adsfi::HafTime GetHafTimestamp()
{
    timeval tv;
    gettimeofday(&tv, nullptr);
    return Adsfi::HafTime{ (uint32_t)tv.tv_sec, (uint32_t)(tv.tv_usec * 1000) };
}

#endif
