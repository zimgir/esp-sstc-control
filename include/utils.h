#ifndef __UTILS_H__
#define __UTILS_H__

#include <assert.h>

#define _STR(s) #s
#define STR(s) _STR(s)

#define LOG(msg, ...)                      \
    do                                     \
    {                                      \
        Serial.printf(msg, ##__VA_ARGS__); \
    } while (0)

#define LOGI(msg, ...) LOG("\n\n" msg "\n\n", ##__VA_ARGS__)
#define LOGE(msg, ...) LOG("\n\nERROR: %s: " msg "\n\n", __func__, ##__VA_ARGS__)
#define LOGD(msg, ...) LOG(msg, ##__VA_ARGS__)

#define BUG(condition)                                                              \
    do                                                                              \
    {                                                                               \
        if (condition)                                                              \
        {                                                                           \
            while (1)                                                               \
            {                                                                       \
                Serial.printf("BUG in %s at %s:%d\n", __func__, __FILE__, __LINE__); \
                delay(1000);                                                        \
            }                                                                       \
        }                                                                           \
    } while (0)


#ifdef PROFILING_ENABLE
#define PROFILE_OP(operation)                                                                    \
    {                                                                                            \
        uint64_t t0, t1;                                                                         \
        t0 = (uint64_t)esp_timer_get_time();                                                     \
        operation;                                                                               \
        t1 = (uint64_t)esp_timer_get_time();                                                     \
        LOG("%s took %lld us\n", #operation, t1 > t0 ? t1 - t0 : 0xFFFFFFFFFFFFFFFFUL - t0 + t1); \
    }
#define PROFILE_OP_TIMES(operation, times)                           \
    {                                                                \
        int i;                                                       \
        uint64_t t0, t1, ct, tt;                                     \
        tt = 0;                                                      \
        for (i = 0; i < times; ++i)                                  \
        {                                                            \
            t0 = (uint64_t)esp_timer_get_time();                     \
            operation;                                               \
            t1 = (uint64_t)esp_timer_get_time();                     \
            ct = t1 > t0 ? t1 - t0 : 0xFFFFFFFFFFFFFFFFUL - t0 + t1; \
            tt += ct;                                                \
            LOG("%s took %lld us\n", #operation, ct);                 \
        }                                                            \
        LOG("%s average %lld us\n", #operation, tt / times);          \
    }
#else
#define PROFILE_OP(operation) operation
#define PROFILE_OP_TIMES(operation, times) operation
#endif

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define RAW_REG(addr) (volatile uint32_t *)(addr)
#define NUM2BIT(num) (1UL << num)

#if 0 // no static_sssert
#define _COMPILER_ASSERT_3(expr,msg) typedef char msg[(!!(expr))*2-1]
#define _COMPILER_ASSERT_2(expr,line) _COMPILER_ASSERT_3(expr,assert_failed_on_line_##line)
#define _COMPILER_ASSERT_1(expr,line) _COMPILER_ASSERT_2(expr,line)
#define COMPILER_ASSERT(expr, msg) _COMPILER_ASSERT_1(expr,__LINE__)
#else
#define COMPILER_ASSERT(expr, msg) static_assert((expr),msg)
#endif

void log_init(void);

#endif