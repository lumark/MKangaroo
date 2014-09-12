#pragma once

#include <cuda_runtime.h>
#include <Kangaroo/CUDA_SDK/cutil_math.h>

namespace roo
{

#define COLOR_SDF

struct __align__(8) SDF_t_Smart {
  inline __host__ __device__ SDF_t_Smart() {}
  inline __host__ __device__ SDF_t_Smart(float v) : val(v), w(1), total_fuse_num(0), val_total(v) {}
  inline __host__ __device__ SDF_t_Smart(float v, float w, float num, float v_total) :
    val(v), w(w), total_fuse_num(num), val_total(v_total) {}

  inline __host__ __device__ operator float() const {
    return val;
  }

  // clamp sdf value via "t"
  inline __host__ __device__ void Clamp(float minval, float maxval) {
    val = clamp(val, minval, maxval);
  }

  inline __host__ __device__ void LimitWeight(float max_weight) {
    w = fminf(w, max_weight);
  }

  inline __host__ __device__ void operator+=(const SDF_t_Smart& rhs)
  {
    if(rhs.w > 0)
    {
      if(total_fuse_num == 0)
      {
        val = (w * val + rhs.w * rhs.val);
        w += rhs.w;
        val /= w;
        val_total = val_total + rhs.val;
        total_fuse_num ++;
      }
      else if( rhs.val >= 1.3 * (val_total/total_fuse_num) ||
               rhs.val <= 0.7 * (val_total/total_fuse_num)
        )
      {
        // skip this value
      }
      else
      {
        val = (w * val + rhs.w * rhs.val);
        w += rhs.w;
        val /= w;
        val_total = val_total + rhs.val;
        total_fuse_num ++;
      }
      // compute mean val

    }
  }


  float val; // final SDF value
  float w;
  float total_fuse_num;
  float val_total; // mean SDF value
};

//struct __align__(8) SDF_t_Smart {
//    inline __host__ __device__ SDF_t_Smart() {}
//    inline __host__ __device__ SDF_t_Smart(float v) : val(v), w(1) {}
//    inline __host__ __device__ SDF_t_Smart(float v, float w) : val(v), w(w) {}
//    inline __host__ __device__ operator float() const {
//        return __half2float(val);
//    }
//    inline __host__ __device__ void Clamp(float minval, float maxval) {
//        val = __float2half_rn( clamp(__half2float(val), minval, maxval) );
//    }
//    inline __host__ __device__ void LimitWeight(float max_weight) {
//        w = __float2half_rn(fminf( __half2float(w), max_weight));
//    }
//    inline __host__ __device__ void operator+=(const SDF_t_Smart& rhs)
//    {
//        if(__half2float(rhs.w) > 0) {
//            val = __float2half_rn(__half2float(w) * __half2float(val) + __half2float(rhs.w) * __half2float(rhs.val) );
//            w = __float2half_rn(__half2float(w) + __half2float(rhs.w));
//            val = __float2half_rn(__half2float(val) / __half2float(w));
//        }
//    }

//    unsigned short w;
//    unsigned short val;
//};

inline __host__ __device__ SDF_t_Smart operator+(const SDF_t_Smart& lhs, const SDF_t_Smart& rhs)
{
  SDF_t_Smart res = lhs;
  res += rhs;
  return res;
}

}
