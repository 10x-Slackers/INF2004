#ifndef S5_HOST_TMONITOR_H
#define S5_HOST_TMONITOR_H
#include <stdarg.h>
#include <stdio.h>
#include "s5_host.h"
static inline int tm_printf(UB *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vprintf((char *)fmt, args);
    va_end(args);
    return n;
}
#endif
