/***************************************************
 CSC C85 RoboSoccer clock helper
***************************************************/

#ifndef LOGIC_CLOCK_H
#define LOGIC_CLOCK_H

#include <time.h>

// What's the damn time
// is static inline because it's declared and defined in this .h file
static inline double maintenant(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec / 1e9;
}

#endif
