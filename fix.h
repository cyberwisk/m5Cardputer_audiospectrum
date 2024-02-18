
#include <limits.h>

template<int p = 15>
static inline int FIX_MPY(int a, int b)
{
  a *= b;
  a += 1 << (p-1);
  a >>= p;
  return a;
}

template<int p = 15>
static inline int FIX_SQU(int a)
{
  return FIX_MPY<p>(a, a);
}

/**
 * Fixed point Log2
 * 
 * https://stackoverflow.com/questions/4657468/fast-fixed-point-pow-log-exp-and-sqrt
 */
template<int p = 15>
static inline int FIX_LOG2(unsigned int x)
{
    unsigned int b = 1U << (p - 1);
    int y = 0;

    if (p < 1 || p > 31) {
        return INT_MAX; // indicates an error
    }

    if (x == 0) {
        return INT_MIN; // represents negative infinity
    }

    while (x < 1U << p) {
        x <<= 1;
        y -= 1U << p;
    }

    while (x >= 2U << p) {
        x >>= 1;
        y += 1U << p;
    }

    unsigned long z = x;

    for (int i = 0; i < p; i++) {
        z = (z * z + (1 << (p-1))) >> p;
        if (z >= 2 << p) {
            z >>= 1;
            y += b;
        }
        b >>= 1;
    }

    return y;
}
