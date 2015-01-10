#pragma once

#include <kangaroo/config.h>
#include <cuda_runtime.h>

#ifdef HAVE_THRUST
#include <thrust/device_vector.h>
#endif // HAVE_THRUST

#ifdef HAVE_NPP
#include <npp.h>
#endif // HAVE_NPP

#include "kangaroo/Image.h"
#include "kangaroo/Mat.h"
#include "kangaroo/sampling.h"
#include "kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"

namespace roo
{

// A VolumeGrid consist n numbers of single volume. Each volume is d*d*d cube.
// When init the VolumeGrid, we set the dim of it and set the dim of single volume of it.
// The initialization of VolumeGrid only init one single volume and a table of active volumes.
// The table of active volumes is consist the information if a single volume is active or not.
// When access VolumeGrid(x,y,z), we will return the volume in that single volume.
// The VolumeGrid itself works like a "volume manager".

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
struct VolumeGrid
{
  inline __host__
  void CleanUp()
  {
    Management::template Cleanup<T,Target>(ptr);
  }

  inline __host__
  void InitVolume(unsigned int n_w, unsigned int n_h, unsigned int n_d)
  {
    Target::template AllocatePitchedMem<T>(&ptr,&pitch,&img_pitch,n_w,n_h,n_d);

    w = n_w;
    h = n_h;
    d = n_d;
  }


  //////////////////////////////////////////////////////
  // VolumeGrid / set copy
  //////////////////////////////////////////////////////
  inline __host__
  void Memset(unsigned char v = 0)
  {
    cudaMemset(ptr,v,pitch*h*d);
  }

  // the following allows us to copy memory from device or host
  template<typename TargetFrom, typename ManagementFrom>
  inline __host__
  void CopyFrom(const VolumeGrid<T,TargetFrom,ManagementFrom>& img)
  {
    // If these VolumeGrids don't have the same height,
    // or have an image pitch different from their height,
    // we need to do a copy for each depth layer.
    //    assert(w == img.w);
    //    assert(h == img.h);
    //    assert(img_pitch == img.img_pitch);
    cudaMemcpy2D(ptr,pitch,img.ptr,img.pitch, std::min(img.w,w)*sizeof(T),
                 h*std::min(img.d,d), TargetCopyKind<Target,TargetFrom>() );
  }


  template <typename DT>
  inline __host__
  void MemcpyFromHost(DT* hptr, size_t hpitch )
  {
    cudaMemcpy2D( (void*)ptr, pitch, hptr, hpitch, w*sizeof(T), h*d, cudaMemcpyHostToDevice );
  }

  template <typename DT>
  inline __host__
  void MemcpyFromHost(DT* ptr )
  {
    MemcpyFromHost(ptr, w*sizeof(T) );
  }

  //////////////////////////////////////////////////////
  // Direct Pixel Access
  //////////////////////////////////////////////////////

  inline  __device__ __host__
  const T* ImagePtr(size_t z) const
  {
    return (T*)((unsigned char*)(ptr) + z*img_pitch);
  }

  inline  __device__ __host__
  T* ImagePtr(size_t z)
  {
    return (T*)((unsigned char*)(ptr) + z*img_pitch);
  }

  inline  __device__ __host__
  T* RowPtr(size_t y, size_t z)
  {
    return (T*)((unsigned char*)(ptr) + z*img_pitch + y*pitch);
  }

  inline  __device__ __host__
  const T* RowPtr(size_t y, size_t z) const
  {
    return (T*)((unsigned char*)(ptr) + z*img_pitch + y*pitch);
  }

  inline  __device__ __host__
  T& operator()(size_t x, size_t y, size_t z)
  {
    return RowPtr(y,z)[x];
  }

  inline  __device__ __host__
  const T& operator()(size_t x, size_t y, size_t z) const
  {
    return RowPtr(y,z)[x];
  }

  inline  __device__ __host__
  T& operator[](size_t ix)
  {
    return ptr[ix];
  }

  inline  __device__ __host__
  const T& operator[](size_t ix) const
  {
    return ptr[ix];
  }

  inline  __device__ __host__
  T& Get(int x, int y, int z)
  {
    return RowPtr(y,z)[x];
  }

  inline  __device__ __host__
  const T& Get(int x, int y, int z) const
  {
    return RowPtr(y,z)[x];
  }

  inline  __device__ __host__
  T& Get(int3 p)
  {
    return RowPtr(p.y,p.z)[p.x];
  }

  inline  __device__ __host__
  const T& Get(int3 p) const
  {
    return RowPtr(p.y,p.z)[p.x];
  }

  //////////////////////////////////////////////////////
  // Interpolated / bounded access
  //////////////////////////////////////////////////////

  inline  __device__ __host__
  const T GetFractionalNearestNeighbour(float3 pos) const
  {
    const float3 pf = pos * make_float3(w-1, h-1, d-1);
    return Get(pf.x+0.5, pf.y+0.5, pf.z+0.5);
  }

  inline  __device__ __host__
  float GetFractionalTrilinear(float3 pos) const
  {
    const float3 pf = pos * make_float3(w-1.f, h-1.f, d-1.f);

    const int ix = floorf(pf.x);
    const int iy = floorf(pf.y);
    const int iz = floorf(pf.z);
    const float fx = pf.x - ix;
    const float fy = pf.y - iy;
    const float fz = pf.z - iz;

    const float v0 = Get(ix,iy,iz);
    const float vx = Get(ix+1,iy,iz);
    const float vy = Get(ix,iy+1,iz);
    const float vxy = Get(ix+1,iy+1,iz);
    const float vz = Get(ix,iy,iz+1);
    const float vxz = Get(ix+1,iy,iz+1);
    const float vyz = Get(ix,iy+1,iz+1);
    const float vxyz = Get(ix+1,iy+1,iz+1);

    return lerp(
          lerp(lerp(v0,vx,fx),  lerp(vy,vxy,fx), fy),
          lerp(lerp(vz,vxz,fx), lerp(vyz,vxyz,fx), fy),
          fz
          );
  }

  // clamp - returns smallest integer not less than a scalar or each vector component.
  // lerp - returns linear interpolation of two scalars or vectors based on a weight
  inline  __device__ __host__
  float GetFractionalTrilinearClamped(float3 pos) const
  {
    // get real gird index by % of pose
    const float3 pf = pos * make_float3(w-1.f, h-1.f, d-1.f);

    // get approximate grid index
    const int ix = fmaxf(fminf(w-2, floorf(pf.x) ), 0);
    const int iy = fmaxf(fminf(h-2, floorf(pf.y) ), 0);
    const int iz = fmaxf(fminf(d-2, floorf(pf.z) ), 0);

    // get index difference
    const float fx = pf.x - ix;
    const float fy = pf.y - iy;
    const float fz = pf.z - iz;

    // get voxel value
    const float v0 = Get(ix,iy,iz);
    const float vx = Get(ix+1,iy,iz);
    const float vy = Get(ix,iy+1,iz);
    const float vxy = Get(ix+1,iy+1,iz);
    const float vz = Get(ix,iy,iz+1);
    const float vxz = Get(ix+1,iy,iz+1);
    const float vyz = Get(ix,iy+1,iz+1);
    const float vxyz = Get(ix+1,iy+1,iz+1);

    // lerp - returns linear interpolation of two scalars or vectors based on a weight
    return lerp(
          lerp(lerp(v0,vx,fx),  lerp(vy,vxy,fx), fy),
          lerp(lerp(vz,vxz,fx), lerp(vyz,vxyz,fx), fy),
          fz );
  }

  //////////////////////////////////////////////////////
  // Finite differences
  //////////////////////////////////////////////////////

  inline __device__ __host__
  float3 GetBackwardDiffDxDyDz(int x, int y, int z) const
  {
    const float v0 = Get(x, y, z);
    return make_float3(
          v0 - Get(x-1, y, z),
          v0 - Get(x, y-1, z),
          v0 - Get(x, y, z-1) );
  }

  inline __device__ __host__
  float3 GetFractionalBackwardDiffDxDyDz(float3 pos) const
  {
    // get real gird index by % of pose
    const float3 pf = pos * make_float3(w-1.f, h-1.f, d-1.f);

    const int ix = fmaxf(fminf(w-2, floorf(pf.x) ), 1);
    const int iy = fmaxf(fminf(h-2, floorf(pf.y) ), 1);
    const int iz = fmaxf(fminf(d-2, floorf(pf.z) ), 1);

    const float fx = pf.x - ix;
    const float fy = pf.y - iy;
    const float fz = pf.z - iz;

    const float3 v0 = GetBackwardDiffDxDyDz(ix,iy,iz);
    const float3 vx = GetBackwardDiffDxDyDz(ix+1,iy,iz);
    const float3 vy = GetBackwardDiffDxDyDz(ix,iy+1,iz);
    const float3 vxy = GetBackwardDiffDxDyDz(ix+1,iy+1,iz);
    const float3 vz = GetBackwardDiffDxDyDz(ix,iy,iz+1);
    const float3 vxz = GetBackwardDiffDxDyDz(ix+1,iy,iz+1);
    const float3 vyz = GetBackwardDiffDxDyDz(ix,iy+1,iz+1);
    const float3 vxyz = GetBackwardDiffDxDyDz(ix+1,iy+1,iz+1);

    return lerp(
          lerp(lerp(v0,vx,fx),  lerp(vy,vxy,fx), fy),
          lerp(lerp(vz,vxz,fx), lerp(vyz,vxyz,fx), fy),
          fz
          );

    //        const int3 p = make_int3(pf);
    //        return GetBackwardDiffDxDyDz(p);
  }

  //////////////////////////////////////////////////////
  // Obtain slices / subimages
  //////////////////////////////////////////////////////

  inline __device__ __host__
  VolumeGrid<T,Target,DontManage> SubVolumeGrid(int3 start, int3 size)
  {
    return VolumeGrid<T,Target,DontManage>(
          &Get(start), size.x, size.y, size.z,
          pitch, img_pitch
          );
  }

  inline __device__ __host__
  Image<T,Target,DontManage> ImageXY(size_t z)
  {
    assert( z < d );
    return Image<T,Target,DontManage>( ImagePtr(z), w, h, pitch);
  }

  inline __device__ __host__
  Image<T,Target,DontManage> ImageXZ(size_t y)
  {
    assert( y < h );
    return Image<T,Target,DontManage>( RowPtr(y,0), w, d, img_pitch);
  }

  //////////////////////////////////////////////////////
  // Size Accessors
  //////////////////////////////////////////////////////

  inline __device__ __host__
  uint3 Voxels() const
  {
    return make_uint3(w,h,d);
  }

  //////////////////////////////////////////////////////
  // Thrust convenience methods
  //////////////////////////////////////////////////////

#ifdef HAVE_THRUST
  inline __device__ __host__
  typename roo::ThrustType<T,Target>::Ptr begin() {
    return (typename roo::ThrustType<T,Target>::Ptr)(ptr);
  }

  inline __device__ __host__
  typename roo::ThrustType<T,Target>::Ptr end() {
    return (typename roo::ThrustType<T,Target>::Ptr)( RowPtr(h-1,d-1) + w );
  }

  inline __host__
  void Fill(T val) {
    thrust::fill(begin(), end(), val);
  }
#endif

  //    inline __device__ __host__
  //    int ActiveSDFsNum()
  //    {
  //       return ActiveSDFs.size();
  //    }

  //////////////////////////////////////////////////////
  // Member variables
  //////////////////////////////////////////////////////

  size_t pitch;
  T* ptr;
  size_t w;
  size_t h;

  size_t img_pitch;
  size_t d;

};

}
