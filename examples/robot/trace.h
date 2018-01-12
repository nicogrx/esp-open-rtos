#ifndef TRACE_H
#define TRACE_H
#ifdef DEBUG
#define INFO(fmt, args...) printf(fmt, ##args)
#else
#define INFO(fmt, args...)
#endif
#endif
