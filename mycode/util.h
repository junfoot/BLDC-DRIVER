#ifndef __UTIL_H__
#define __UTIL_H__

#include "main.h"
#include <stdbool.h>

#define SQ(x) ((x) * (x))
#define SQRT3 1.73205080757f
static const float ONE_BY_SQRT3 = 0.57735026919f;
static const float TWO_BY_SQRT3 = 1.15470053838f;
static const float SQRT3_BY_2 = 0.86602540378f;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x)				((x < 0) ? -1 : 1)

#ifndef PI
  #define PI               3.14159265358979f
#endif

unsigned int crc32(const unsigned char *buf, int len, unsigned int init);
float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);

static inline void int_to_data(int val, uint8_t *data)
{
	data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
	int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

static inline uint32_t cpu_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();   // Disable IRQ Interrupts
    return primask;
}

static inline void cpu_exit_critical(uint32_t priority_mask)
{
    __set_PRIMASK(priority_mask);
}


#endif
