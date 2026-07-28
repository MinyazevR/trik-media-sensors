#ifndef TRIK_SENSORS_LOG_H_
#define TRIK_SENSORS_LOG_H_

#include <stdio.h>

typedef enum { LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG } log_level_t;

extern log_level_t g_log_level;

#define LOG(level, fmt, ...) do { \
    if ((level) <= g_log_level) { \
        fprintf((level) <= LOG_WARN ? stderr : stdout, \
                "%c %s:%d: " fmt "\n", \
                "EWID"[level], __func__, __LINE__, ##__VA_ARGS__); \
        fflush((level) <= LOG_WARN ? stderr : stdout); \
    } \
} while (0)

static inline void log_set_level(log_level_t _level) { g_log_level = _level; }
static inline log_level_t log_get_level(void)  { return g_log_level; }

#endif
