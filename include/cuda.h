#ifndef _CUDA_H_
#define _CUDA_H_

#include <stddef.h>

#define __constant__ __attribute__((constant))
#define __device__ __attribute__((device))
#define __global__ __attribute__((global))
#define __host__ __attribute__((host))
#define __shared__ __attribute__((shared))

struct uint3 {
  unsigned int x, y, z;
};

struct dim3 {
  unsigned int x, y, z;
  dim3(unsigned int vx = 1, unsigned int vy = 1, unsigned int vz = 1) : x(vx), y(vy), z(vz) {}
  dim3(uint3 v) : x(v.x), y(v.y), z(v.z) {}
  operator uint3(void) { uint3 t; t.x = x; t.y = y; t.z = z; return t; }
};

typedef struct uint3 uint3;
typedef struct dim3 dim3;

extern const dim3 threadIdx;
extern const dim3 blockIdx;
extern const uint3 blockDim;
extern const uint3 gridDim;

#endif
