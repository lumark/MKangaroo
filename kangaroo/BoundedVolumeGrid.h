#pragma once

#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "Sdf.h"


namespace roo
{

inline int GetAvailableGPUMemory()
{
  const unsigned bytes_per_mb = 1024*1000;
  size_t cu_mem_start, cu_mem_total;
  if(cudaMemGetInfo( &cu_mem_start, &cu_mem_total ) != cudaSuccess) {
    std::cerr << "Unable to get available memory" << std::endl;
    exit(-1);
  }

  int LeftMemory = cu_mem_start/bytes_per_mb;
  return LeftMemory;
}

// A BoundedVolumeGrid consist n numbers of single volume. Each volume is
// (m_BasicGridRes*m_BasicGridRes*m_BasicGridRes) cube.

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
class BoundedVolumeGrid
{
public:

  inline __host__
  void init(unsigned int n_w, unsigned int n_h, unsigned int n_d, unsigned int n_res, const BoundingBox& r_bbox)
  {
    // init grid sdf
    m_w    = n_w;
    m_h    = n_h;
    m_d    = n_d;
    m_bbox = r_bbox;
    m_BasicGridRes = n_res;
    m_WholeGridRes = m_w/n_res;

    printf("init! whole grid res:%d;basic grid res:%d",m_WholeGridRes,m_BasicGridRes);

    // init all basic SDFs
    for(int i=0;i!=64; i++)
    {
      m_GridVolumes[i].InitVolume(64,64,64);
    }
  }

  //////////////////////////////////////////////////////
  // Dimensions
  //////////////////////////////////////////////////////

  inline __device__ __host__
  float3 SizeUnits() const
  {
    return m_bbox.Size();
  }

  inline __device__ __host__
  float3 VoxelSizeUnits() const
  {
    return m_bbox.Size() /
        make_float3( m_w-1, m_h-1, m_d-1 );
  }

  //////////////////////////////////////////////////////
  // Return true if this BoundedVolumeGrid represents a positive
  // amount of space.
  //////////////////////////////////////////////////////

  //  inline __device__ __host__
  //  bool IsValid() const {
  //    const uint3 size = Volume<T,Target,Management>::Voxels();
  //    return size.x >= 8 && size.y >= 8 && size.z >= 8;
  //  }

  inline  __device__ __host__
  T& operator()(unsigned int x,unsigned int y, unsigned int z)
  {
    int nIndex = int(floorf(x/64)) + 4 * ( int(floorf(y/64)) + 4 * int(floorf(z/64)) );

//    if(nIndex>20)
//    {
//      printf("input=(%d,%d,%d);index=%d(%d,%d,%d);pos(%d,%d,%d)",x,y,z,nIndex,int(x/64), int(y/64), int(z/64), x%64, y%64, z%64);

//    }

    return m_GridVolumes[nIndex](x%64, y%64, z%64);
  }

  inline __device__ __host__
  int GetValidVolsNum()
  {
    int num = 0;

    //    for(int i=0;i!=m_d/m_res;i++)
    //    {
    //      for(int j=0;j!=m_d/m_res;j++)
    //      {
    //        for(int k=0;k!=m_d/m_res;k++)
    //        {
    //          if(GetSDFByIndex(i,j,k)->d == m_res)
    //          {
    //            num++;
    //          }
    //        }
    //      }
    //    }

    return num;
  }



  // input pos_w in meter
  inline  __device__ __host__
  float GetUnitsTrilinearClamped(float3 pos_w) const
  {
    // get pose of voxel in whole sdf, in %
    const float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    if(pos_w.z>=0.5 && pos_v.z>=0)
    {
      // Get the index of voxel in basic sdf
      const int nIndex_x = floorf(pos_v.x/0.25f);
      const int nIndex_y = floorf(pos_v.y/0.25f);
      const int nIndex_z = floorf(pos_v.z/0.25f);

      const int nIndex = nIndex_x + 4* (nIndex_y+ 4* nIndex_z);

      float3 pos_v_grid =make_float3( fmod(pos_v.x,0.25f) /0.25f, fmod(pos_v.y,0.25f) /0.25f, fmod(pos_v.z,0.25f) /0.25f);

      //      printf("pos_v:%f,%f,%f, index=%d; x=%f,y=%f,z=%f;",pos_v.x,pos_v.y,pos_v.z, nIndex, pos_v_grid.x, pos_v_grid.y,pos_v_grid.z);

      return m_GridVolumes[nIndex].GetFractionalTrilinearClamped(pos_v_grid);
    }
    else
    {
      return 0;
    }

  }

  inline __device__ __host__
  float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
  {
    // pose of voxel in whole sdf, in meters
    const float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    if(pos_w.z>=0.5 && pos_v.z>=0)
    {
      // Get the index of voxel in basic sdf
      const int nIndex_x = floorf(pos_v.x/0.25f);
      const int nIndex_y = floorf(pos_v.y/0.25f);
      const int nIndex_z = floorf(pos_v.z/0.25f);

      const int nIndex = nIndex_x + 4* (nIndex_y+ 4* nIndex_z);

      float3 pos_v_grid =make_float3( fmod(pos_v.x,0.25f) /0.25f, fmod(pos_v.y,0.25f) /0.25f, fmod(pos_v.z,0.25f) /0.25f);

      //      printf("pos_v:%f,%f,%f, index=%d; x=%f,y=%f,z=%f;",pos_v.x,pos_v.y,pos_v.z, nIndex, pos_v_grid.x, pos_v_grid.y,pos_v_grid.z);

      const float3 deriv = m_GridVolumes[nIndex].GetFractionalBackwardDiffDxDyDz(pos_v_grid);

      return deriv / VoxelSizeUnits();
    }
    else
    {
      return make_float3(0,0,0);
    }
  }

  inline __device__ __host__
  float3 GetUnitsOutwardNormal(float3 pos_w) const
  {
    const float3 deriv = GetUnitsBackwardDiffDxDyDz(pos_w);
    return deriv / length(deriv);
  }

  inline __device__ __host__
  float3 VoxelPositionInUnits(int x, int y, int z) const
  {
    const float3 vol_size = m_bbox.Size();

    //    printf("input(%d,%d,%d,). get(%f,%f,%f);",x,y,z,m_bbox.Min().x + vol_size.x*x/(float)(m_w-1),m_bbox.Min().y + vol_size.y*y/(float)(m_h-1),m_bbox.Min().z + vol_size.z*z/(float)(m_d-1));

    return make_float3(
          m_bbox.Min().x + vol_size.x*x/(float)(m_w-1),
          m_bbox.Min().y + vol_size.y*y/(float)(m_h-1),
          m_bbox.Min().z + vol_size.z*z/(float)(m_d-1)
          );
  }

  inline __device__ __host__
  float3 VoxelPositionInUnits(int3 p_v) const
  {
    return VoxelPositionInUnits(p_v.x,p_v.y,p_v.z);
  }


public:
  size_t                                                m_d;
  size_t                                                m_w;
  size_t                                                m_h;

  // bounding box ofbounded volume grid
  BoundingBox                                           m_bbox;

  size_t                                                m_BasicGridRes;   // resolution of a single grid
  size_t                                                m_WholeGridRes;   // how many grid we want to use in one row, usually 4, 8, 16

  // volume that save all data
  VolumeGrid<T, TargetDevice, Manage>                   m_GridVolumes[64];
};


}
