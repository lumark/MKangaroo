#pragma once

#include <cuda_runtime.h>
#include "CUDA_SDK/cutil_math.h"
#include <cmath>        // std::abs

namespace roo
{

#define COLOR_SDF

struct __align__(8) SDF_t {
    inline __host__ __device__ SDF_t() {}
    inline __host__ __device__ SDF_t(float v) : val(v), w(1) {}
    inline __host__ __device__ SDF_t(float v, float w) : val(v), w(w) {}

    inline __host__ __device__ operator float() const {
        return val;
    }
    inline __host__ __device__ void Clamp(float minval, float maxval) {
        val = clamp(val, minval, maxval);
    }
    inline __host__ __device__ void LimitWeight(float max_weight) {
        w = fminf(w, max_weight);
    }
    inline __host__ __device__ void operator+=(const SDF_t& rhs)
    {
        if(rhs.w > 0) {
            val = (w * val + rhs.w * rhs.val);
            w += rhs.w;
            val /= w;
        }
    }

    float val;
    float w;
};


struct __align__(8) SDF_t_comp {
    inline __host__ __device__ SDF_t_comp():Times(0) {}
    inline __host__ __device__ SDF_t_comp(float v) : val(v), w(1), Times(0) {}
    inline __host__ __device__ SDF_t_comp(float v, float w) : val(v), w(w), Times(0) {}

    inline __host__ __device__ operator float() const {
        return val;
    }

    inline __host__ __device__ void Clamp(float minval, float maxval) {
        val = clamp(val, minval, maxval);
    }

    inline __host__ __device__ void LimitWeight(float max_weight) {
        w = fminf(w, max_weight);
    }

    inline __host__ __device__ void operator+=(const SDF_t& rhs)
    {
        if(rhs.w > 0) {
            val = (w * val + rhs.w * rhs.val);
            w += rhs.w;
            val /= w;
            Times=Times++;
        }
    }

    inline __host__ __device__ void UpdatUnCertainty(float tran_x,float tran_y,float tran_z,
                                                     float rot_x, float rot_y, float rot_z)
    {
      TranChange = std::abs(tran_x)+std::abs(tran_y)+std::abs(tran_z);
      RotChange  = std::abs(rot_x) +std::abs(rot_y) +std::abs(rot_z);
    }

    /// TODO: do more complex compare
    inline __host__ __device__ bool CheckIfUpdate(float tran_x,float tran_y,float tran_z,
                                                  float rot_x, float rot_y, float rot_z)
    {
      float cur_tran_change = std::abs(tran_x)+std::abs(tran_y)+std::abs(tran_z);
      float cur_rot_change  = std::abs(rot_x) +std::abs(rot_y) +std::abs(rot_z);

      if(cur_tran_change<TranChange && cur_rot_change<RotChange)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    float val;
    float w;
    int   Times;// record times that we update the voxel
    float TranChange; // change in translation
    float RotChange;  // change in rotation
};

//struct __align__(8) SDF_t {
//    inline __host__ __device__ SDF_t() {}
//    inline __host__ __device__ SDF_t(float v) : val(v), w(1) {}
//    inline __host__ __device__ SDF_t(float v, float w) : val(v), w(w) {}
//    inline __host__ __device__ operator float() const {
//        return __half2float(val);
//    }
//    inline __host__ __device__ void Clamp(float minval, float maxval) {
//        val = __float2half_rn( clamp(__half2float(val), minval, maxval) );
//    }
//    inline __host__ __device__ void LimitWeight(float max_weight) {
//        w = __float2half_rn(fminf( __half2float(w), max_weight));
//    }
//    inline __host__ __device__ void operator+=(const SDF_t& rhs)
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

inline __host__ __device__ SDF_t operator+(const SDF_t& lhs, const SDF_t& rhs)
{
    SDF_t res = lhs;
    res += rhs;
    return res;
}

}
