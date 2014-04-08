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
    m_WholeGridRes = m_w/m_BasicGridRes;

    //    printf("init! whole grid res:%d;basic grid res:%d",m_WholeGridRes,m_BasicGridRes);

    // init all basic SDFs
    for(int i=0;i!=m_WholeGridRes*m_WholeGridRes*m_WholeGridRes; i++)
    {
      m_GridVolumes[i].InitVolume(m_BasicGridRes,m_BasicGridRes,m_BasicGridRes);

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
    int nIndex = int(floorf(x/m_BasicGridRes)) + m_WholeGridRes * ( int(floorf(y/m_BasicGridRes)) + m_WholeGridRes * int(floorf(z/m_BasicGridRes)) );

    //        printf("fuse x:%d,y:%d,z:%d, index %d,x:%d,y:%d,z:%d, basic:%d,whole:%d;",x,y,z,nIndex, x%m_BasicGridRes, y%m_BasicGridRes, z%m_BasicGridRes,m_BasicGridRes,m_WholeGridRes);

    return m_GridVolumes[nIndex](x%m_BasicGridRes, y%m_BasicGridRes, z%m_BasicGridRes);
  }


  // input pos_w in meter
  inline  __device__ __host__
  float GetUnitsTrilinearClamped(float3 pos_w) const
  {
    /// get pose of voxel in whole sdf, in %
    float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    if(pos_v.x>=1) { pos_v.x =0.99999; }
    if(pos_v.y>=1) { pos_v.y =0.99999; }
    if(pos_v.z>=1) { pos_v.z =0.99999; }

    if(pos_v.x<0) { pos_v.x =0; }
    if(pos_v.y<0) { pos_v.y =0; }
    if(pos_v.z<0) { pos_v.z =0; }

    //    if(pos_v.x>=1) { pos_v.x =1; }
    //    if(pos_v.y>=1) { pos_v.y =1; }
    //    if(pos_v.z>=1) { pos_v.z =1; }

    //    if(pos_v.x<0) { pos_v.x =0; }
    //    if(pos_v.y<0) { pos_v.y =0; }
    //    if(pos_v.z<0) { pos_v.z =0; }

    // Get the index of voxel in basic sdf
    const int3 Index =make_int3( floorf(pos_v.x/0.25f),  floorf(pos_v.y/0.25f),  floorf(pos_v.z/0.25f)  );

    // access actual vol
    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);

    if(nIndex>=0 && nIndex<64)
    {
      /// get axis.
      float3 pos_v_grid = make_float3( fmod(pos_v.x,0.25f) /0.25f, fmod(pos_v.y,0.25f) /0.25f, fmod(pos_v.z,0.25f) /0.25f );

      //      printf("pos_v:%f,%f,%f, index=%d; x=%f,y=%f,z=%f;",pos_v.x,pos_v.y,pos_v.z, nIndex, pos_v_grid.x, pos_v_grid.y,pos_v_grid.z);

      return m_GridVolumes[nIndex].GetFractionalTrilinearClamped(pos_v_grid);
    }
    else
    {
      printf("pos_w:(%f,%f,%f),pos_v(%f,%f,%f),index(%d,x%d,y%d,z%d);",pos_w.x,pos_w.y,pos_w.z,pos_v.x,pos_v.y,pos_v.z,nIndex,Index.x,Index.y,Index.z);
      return 0;
    }
  }

  inline __device__ __host__
  float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
  {
    /// get pose of voxel in whole sdf, in %
    float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    if(pos_v.x>=1) { pos_v.x =0.99999; }
    if(pos_v.y>=1) { pos_v.y =0.99999; }
    if(pos_v.z>=1) { pos_v.z =0.99999; }

    if(pos_v.x<0) { pos_v.x =0; }
    if(pos_v.y<0) { pos_v.y =0; }
    if(pos_v.z<0) { pos_v.z =0; }

    //    if(pos_v.x>=1) { pos_v.x =1; }
    //    if(pos_v.y>=1) { pos_v.y =1; }
    //    if(pos_v.z>=1) { pos_v.z =1; }

    //    if(pos_v.x<0) { pos_v.x =0; }
    //    if(pos_v.y<0) { pos_v.y =0; }
    //    if(pos_v.z<0) { pos_v.z =0; }

    // Get the index of voxel in basic sdf
    const int3 Index =make_int3( floorf(pos_v.x/0.25f), floorf(pos_v.y/0.25f),floorf(pos_v.z/0.25f)  );

    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);


    if(nIndex>=0 && nIndex<64)
    {
      /// get axis.
      float3 pos_v_grid = make_float3( fmod(pos_v.x,0.25f) /0.25f, fmod(pos_v.y,0.25f) /0.25f, fmod(pos_v.z,0.25f) /0.25f );

      //      printf("pos_v:%f,%f,%f, index=%d; x=%f,y=%f,z=%f;",pos_v.x,pos_v.y,pos_v.z, nIndex, pos_v_grid.x, pos_v_grid.y,pos_v_grid.z);

      const float3 deriv = m_GridVolumes[nIndex].GetFractionalBackwardDiffDxDyDz(pos_v_grid);

      return deriv / VoxelSizeUnits();
    }
    else
    {
      printf("pos_w:(%f,%f,%f),pos_v(%f,%f,%f),index(%d,x%d,y%d,z%d);",pos_w.x,pos_w.y,pos_w.z,pos_v.x,pos_v.y,pos_v.z,nIndex,Index.x,Index.y,Index.z);

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

  inline __device__ __host__
  void FreeMem()
  {
    for(int i=0;i!=m_WholeGridRes*m_WholeGridRes*m_WholeGridRes;i++)
    {
      cudaFree( m_GridVolumes[i].ptr );
    }
  }


  //  // input pos_w in meter
  //  inline  __device__ __host__
  //  float GetUnitsTrilinearClamped(float3 pos_w) const
  //  {
  //    /// get pose of voxel in whole sdf, in %
  //    const float3 pos_v_presentage = (pos_w - m_bbox.Min()) / (m_bbox.Size());

  //    // get actual grid index by % of pose
  //    const float3 pf = pos_v_presentage * make_float3(m_w-1.f, m_h-1.f, m_d-1.f);

  //    // get approximate grid index
  //    const float3 pos_grid_index_approximate = make_float3( fmaxf(fminf(m_w-2, floorf(pf.x) ), 0),
  //                                                           fmaxf(fminf(m_h-2, floorf(pf.y) ), 0),
  //                                                           fmaxf(fminf(m_d-2, floorf(pf.z) ), 0)
  //                                                           );

  //    const float3 pos_v = make_float3( pos_grid_index_approximate.x/(m_w-1.f),
  //                                      pos_grid_index_approximate.y/(m_h-1.f),
  //                                      pos_grid_index_approximate.z/(m_d-1.f));

  //    // Get the index of voxel in basic sdf
  //    const int3 Index =make_int3( floorf(pos_v.x/0.25f), floorf(pos_v.y/0.25f), floorf(pos_v.z/0.25f) );

  //    // access actual vol
  //    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);

  //    /// get axis.
  //    float3 pos_v_grid = make_float3( fmod(pos_v_presentage.x,0.25f) /0.25f, fmod(pos_v_presentage.y,0.25f) /0.25f, fmod(pos_v_presentage.z,0.25f) /0.25f);

  //    //            printf("pos_v:%f,%f,%f, index=%d; x=%f,y=%f,z=%f;",pos_v.x,pos_v.y,pos_v.z, nIndex, pos_v_grid.x, pos_v_grid.y,pos_v_grid.z);

  //    return m_GridVolumes[nIndex].GetFractionalTrilinearClamped(pos_v_grid);
  //  }

  //  inline __device__ __host__
  //  float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
  //  {
  //    /// get pose of voxel in whole sdf, in %
  //    const float3 pos_v_presentage = (pos_w - m_bbox.Min()) / (m_bbox.Size());

  //    // get actual pose from int index of input pos
  //    const float3 pf = pos_v_presentage * make_float3(m_w-1.f, m_h-1.f, m_d-1.f);

  //    // get pose index in grid index, for get index
  //    const float3 pos_grid_index_approximate = make_float3( fmaxf(fminf(m_w-2, floorf(pf.x) ), 1),
  //                                                           fmaxf(fminf(m_h-2, floorf(pf.y) ), 1),
  //                                                           fmaxf(fminf(m_d-2, floorf(pf.z) ), 1)
  //                                                           );

  //    const float3 pos_v = make_float3( pos_grid_index_approximate.x/(m_w-1.f),
  //                                      pos_grid_index_approximate.y/(m_h-1.f),
  //                                      pos_grid_index_approximate.z/(m_d-1.f));

  //    // Get the index of voxel in basic sdf
  //    const int3 Index =make_int3( floorf(pos_v.x/0.25f), floorf(pos_v.y/0.25f), floorf(pos_v.z/0.25f) );

  //    // access actual vol
  //    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);

  //    /// get axis.
  //    float3 pos_v_grid = make_float3( fmod(pos_v_presentage.x,0.25f) /0.25f, fmod(pos_v_presentage.y,0.25f) /0.25f, fmod(pos_v_presentage.z,0.25f) /0.25f);

  //    //            printf("pos_v:%f,%f,%f, index=%d; x=%f,y=%f,z=%f;",pos_v.x,pos_v.y,pos_v.z, nIndex, pos_v_grid.x, pos_v_grid.y,pos_v_grid.z);

  //    const float3 deriv = m_GridVolumes[nIndex].GetFractionalBackwardDiffDxDyDz(pos_v_grid);

  //    return deriv / VoxelSizeUnits();
  //  }

public:
  size_t                                                m_d;
  size_t                                                m_w;
  size_t                                                m_h;

  // bounding box ofbounded volume grid
  BoundingBox                                           m_bbox;

  unsigned int                                          m_BasicGridRes;    // resolution of a single grid
  unsigned int                                          m_WholeGridRes;    // how many grid we want to use in one row, usually 4, 8, 16

  // volume that save all data
  VolumeGrid<T, TargetDevice, Manage>                   m_GridVolumes[64]; // 4 by 4 by 4
};


}
