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

    // init all basic SDFs
    printf("available memory before init volume is %d, Active Num of grid volume is %d\n",GetAvailableGPUMemory(), GetActiveGridVolNum());

//    for(int i=0;i!=m_w;i++)
//    {
//      for(int j=0;j!=m_h;j++)
//      {
//        for(int k=0;k!=m_d;k++)
//        {
//          InitSingleBasicSDFWithGridIndex(i,j,k);
//        }
//      }
//    }

    printf("available memory after init volume is %d, Active Num of grid volume is %d\n",GetAvailableGPUMemory(), GetActiveGridVolNum());
  }

  inline __host__
  void InitSingleBasicSDFWithGridIndex(unsigned int x, unsigned int y, unsigned int z)
  {
    int nIndex = int(floorf(x/m_BasicGridRes)) + m_WholeGridRes * ( int(floorf(y/m_BasicGridRes)) + m_WholeGridRes * int(floorf(z/m_BasicGridRes)) );

    if(m_GridVolumes[nIndex].d !=m_BasicGridRes)
    {
      m_GridVolumes[nIndex].InitVolume(m_BasicGridRes,m_BasicGridRes,m_BasicGridRes);
    }
  }


  inline __host__
  bool InitSingleBasicSDFWithIndex(int nIndex)
  {
//    printf("try to init basic sdf with index %d\n",nIndex);

    if( m_GridVolumes[nIndex].d !=m_BasicGridRes )
    {
      m_GridVolumes[nIndex].InitVolume(m_BasicGridRes,m_BasicGridRes,m_BasicGridRes);
      printf("init grid sdf %d success \n",nIndex);
      return true;
    }

    return false;
  }


  inline __host__
  void InitSingleBasicSDF(float x, float y, float z)
  {
    //    float3 pos_w = make_float3(x,y,z);

    //    /// get pose of voxel in whole sdf, in %
    //    float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    //    if(pos_v.x>=1) { pos_v.x =0.99999; }
    //    if(pos_v.y>=1) { pos_v.y =0.99999; }
    //    if(pos_v.z>=1) { pos_v.z =0.99999; }

    //    if(pos_v.x<0) { pos_v.x =0; }
    //    if(pos_v.y<0) { pos_v.y =0; }
    //    if(pos_v.z<0) { pos_v.z =0; }

    //    const float fFactor = float(m_BasicGridRes)/float(m_w);

    //    // Get the index of voxel in basic sdf
    //    const int3 Index =make_int3( floorf(pos_v.x/fFactor),  floorf(pos_v.y/fFactor),  floorf(pos_v.z/fFactor)  );

    //    // access actual vol
    //    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);

    //    if(CheckIfBasicSDFActive(nIndex) == false)
    //    {
    //      printf("init new sdf with index %d success!\n",nIndex);
    //      m_GridVolumes[nIndex].InitVolume(m_BasicGridRes,m_BasicGridRes,m_BasicGridRes);
    //    }
  }


  inline __host__
  void InitSDFs()
  {
    printf("try to init %d sdfs..\n", 512);

    for(int i=0;i!=512;i++)
    {
      if(CheckIfBasicSDFActive(i)==false && m_NextInitBasicSDFs[i]==1)
      {
        //        m_GridVolumes[i].InitVolume(m_BasicGridRes,m_BasicGridRes,m_BasicGridRes);
        printf("init new sdf %d success. \n",i );
      }

      if(m_NextInitBasicSDFs[i]==1 && CheckIfBasicSDFActive(i)==true)
      {
        printf("skip init sdf %d . \n",i );
      }
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

  inline  __device__
  T& operator()(unsigned int x,unsigned int y, unsigned int z)
  {
    const int nIndex = int(floorf(x/m_BasicGridRes)) + m_WholeGridRes * ( int(floorf(y/m_BasicGridRes)) + m_WholeGridRes * int(floorf(z/m_BasicGridRes)) );

    if(CheckIfBasicSDFActive(nIndex)==false)
    {
      printf("[operator] fatal error! sdf %d is not init yet!\n", nIndex);
    }

    return m_GridVolumes[nIndex](x%m_BasicGridRes, y%m_BasicGridRes, z%m_BasicGridRes);
  }

  // input pos_w in meter
  inline  __device__
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

    const float fFactor = float(m_BasicGridRes)/float(m_w);

    // Get the index of voxel in basic sdf
    const int3 Index =make_int3( floorf(pos_v.x/fFactor),  floorf(pos_v.y/fFactor),  floorf(pos_v.z/fFactor)  );

    // access actual vol
    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);

    /// get axis.
    float3 pos_v_grid = make_float3( fmod(pos_v.x,fFactor) /fFactor, fmod(pos_v.y,fFactor) /fFactor, fmod(pos_v.z,fFactor) /fFactor );

    if(CheckIfBasicSDFActive(nIndex)==false)
    {
      return 0;
    }

    return m_GridVolumes[nIndex].GetFractionalTrilinearClamped(pos_v_grid);
  }

  inline __device__
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

    const float fFactor = float(m_BasicGridRes)/float(m_w);

    // Get the index of voxel in basic sdf
    const int3 Index =make_int3( floorf(pos_v.x/fFactor), floorf(pos_v.y/fFactor),floorf(pos_v.z/fFactor)  );

    const int nIndex = Index.x + m_WholeGridRes* (Index.y+ m_WholeGridRes* Index.z);

    /// get axis.
    float3 pos_v_grid = make_float3( fmod(pos_v.x,fFactor) /fFactor, fmod(pos_v.y,fFactor) /fFactor, fmod(pos_v.z,fFactor) /fFactor );

    if(CheckIfBasicSDFActive(nIndex)==false)
    {
      return make_float3(0,0,0);
    }

    const float3 deriv = m_GridVolumes[nIndex].GetFractionalBackwardDiffDxDyDz(pos_v_grid);

    return deriv / VoxelSizeUnits();
  }

  inline __device__
  float3 GetUnitsOutwardNormal(float3 pos_w) const
  {
    const float3 deriv = GetUnitsBackwardDiffDxDyDz(pos_w);
    return deriv / length(deriv);
  }

  inline __device__
  float3 VoxelPositionInUnits(int x, int y, int z) const
  {
    const float3 vol_size = m_bbox.Size();

    return make_float3(
          m_bbox.Min().x + vol_size.x*x/(float)(m_w-1),
          m_bbox.Min().y + vol_size.y*y/(float)(m_h-1),
          m_bbox.Min().z + vol_size.z*z/(float)(m_d-1)
          );
  }

  inline __device__
  float3 VoxelPositionInUnits(int3 p_v) const
  {
    return VoxelPositionInUnits(p_v.x,p_v.y,p_v.z);
  }

  inline __host__
  void CopyFrom(BoundedVolumeGrid<T, Target , Management>& rVol )
  {
    for(int i=0;i!= m_WholeGridRes*m_WholeGridRes*m_WholeGridRes;i++)
    {
      m_GridVolumes[i].MemcpyFromDevice(rVol.m_GridVolumes[i]);
    }
  }

  inline __host__
  void FreeMemory()
  {
    for(int i=0;i!=m_WholeGridRes*m_WholeGridRes*m_WholeGridRes;i++)
    {
      cudaFree( m_GridVolumes[i].ptr );
    }
  }

  inline __host__ __device__
  bool CheckIfBasicSDFActive(const int nIndex) const
  {
    if(m_GridVolumes[nIndex].d == m_BasicGridRes)
    {
      return true;
    }
    else
    {
      return false;
    }
  }


  inline __host__
  int GetActiveGridVolNum()
  {
    int nNum = 0;

    for(int i=0;i!=m_WholeGridRes*m_WholeGridRes*m_WholeGridRes;i++)
    {
      if(m_GridVolumes[i].d == m_BasicGridRes)
      {
        nNum ++;
      }
    }

    return nNum;
  }


  inline __device__
  void SetNextInitSDF(unsigned int x, unsigned int y, unsigned int z)
  {
    const int nIndex = x/m_BasicGridRes+ m_WholeGridRes*( y/m_BasicGridRes+m_WholeGridRes * z/m_BasicGridRes );
    if(m_NextInitBasicSDFs[nIndex] == 0)
    {
      m_NextInitBasicSDFs[nIndex] = 1;
//      printf("set %d to true.", nIndex);
    }
  }

public:
  size_t                                                m_d;
  size_t                                                m_w;
  size_t                                                m_h;

  // bounding box ofbounded volume grid
  BoundingBox                                           m_bbox;

  unsigned int                                          m_BasicGridRes;    // resolution of a single grid
  unsigned int                                          m_WholeGridRes;    // how many grid we want to use in one row, usually 4, 8, 16

  // volume that save all data
  VolumeGrid<T, TargetDevice, Manage>                   m_GridVolumes[512]; // 4 by 4 by 4
  int                                                   m_NextInitBasicSDFs[512];
};


}
