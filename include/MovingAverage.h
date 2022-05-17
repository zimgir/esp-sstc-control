#ifndef __MOVING_AVERAGE_H__
#define __MOVING_AVERAGE_H__

#include <string.h>

template <typename T_TYPE, uint32_t T_LENGTH>
class MovingAverage
{

public:
    MovingAverage() : __first_idx(0),
                      __last_idx(0),
                      __num(0),
                      __sum(0),
                      __buffer{0}
    {
    }

    ~MovingAverage() {}

    T_TYPE value()
    {
        T_TYPE avg;
        avg = __sum / __num;
        return avg;
    }

    T_TYPE add(T_TYPE val)
    {
        T_TYPE avg;

        if (__num >= length)
        {
            __sum -= __buffer[__first_idx];
            __first_idx = (__first_idx + 1) % length;
            __sum += val;
            __buffer[__last_idx] = val;
            __last_idx = (__last_idx + 1) % length;
        }
        else
        {
            __num += 1;
            __sum += val;
            __buffer[__last_idx] = val;
            __last_idx = (__last_idx + 1) % length;
        }

        avg = __sum / __num;

        return avg;
    }

    T_TYPE reset()
    {
        T_TYPE avg;

        avg = __sum / __num;

        __first_idx = 0;
        __last_idx = 0;
        __num = 0;
        __sum = 0;

        memset(__buffer, 0, sizeof(__buffer));

        return avg;
    }

    uint32_t num()
    {
        return __num;
    }

    static const uint32_t length = T_LENGTH;

private:
    uint32_t __first_idx;
    uint32_t __last_idx;
    uint32_t __num;

    T_TYPE __sum;

    T_TYPE __buffer[length];
};

#endif