#pragma once

#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "Sdf.h"
//#include "cu_sdffusion.h"


namespace roo
{

// A BoundedVolumeGrid consist n numbers of single volume. Each volume is d*d*d cube.
// When init the BoundedVolumeGrid, we set the dim of it and set the dim of single volume of it.
// The initialization of VolumeGrid only init one single volume and a table of active volumes.
// The table of active volumes is consist the information if a single volume is active or not.
// When access BoundedVolumeGrid(x,y,z), we will return the volume in that single volume.
// The VolumeGrid itself works like a "BoundedVolume manager".

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
class BoundedVolumeGrid
{
public:

  inline __host__
  void init(unsigned int n_w, unsigned int n_h, unsigned int n_d, unsigned int n_res, const BoundingBox& r_bbox)
  {
    m_w    = n_w;
    m_h    = n_h;
    m_d    = n_d;
    m_BasicGridRes = n_res;
    m_WholeGridRes = m_w/n_res;
    m_bbox = r_bbox;

    // init all basic SDFs
    for(int i=0;i!=64; i++)
    {
      m_GridVolumes[i].InitVolume(m_BasicGridRes,m_BasicGridRes,m_BasicGridRes);
      //      SdfReset(m_GridVolumes[i]);
    }

//    printf("[BoundedVolumeGrid] init bounded volume grid success. x=%d,y=%d,z=%d is, bbox min x is %f, input %f\n",
//           m_w,m_h,m_d, m_bbox.boxmin.x, r_bbox.boxmin.x);
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
  T& operator()(size_t x, size_t y, size_t z)
  {
    // convert x, y, z, to actual index and get the element
    return m_GridVolumes[x/m_BasicGridRes + m_WholeGridRes * (y/m_BasicGridRes + m_WholeGridRes * z/m_BasicGridRes)](x%m_BasicGridRes,y%m_BasicGridRes,z%m_BasicGridRes );
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



  inline  __device__ __host__
  float GetUnitsTrilinearClamped(float3 pos_w) const
  {
    // pose of point in whole sdf
    const float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    // convert pos_v to pose in grid sdf
    const float3 pos_v_grid = make_float3( fmod(pos_v.x, float(m_WholeGridRes) ),
                                           fmod(pos_v.y, float(m_WholeGridRes) ),
                                           fmod(pos_v.z, float(m_WholeGridRes) ));

    float fSingleGridLength = float (2 * m_bbox.Max().x/m_WholeGridRes );

    // get actual val in grid sdf
    return m_GridVolumes[ int(pos_v.x/fSingleGridLength) + m_WholeGridRes * ( int(pos_v.y/fSingleGridLength) + m_WholeGridRes * int( pos_v.z/m_WholeGridRes ))].GetFractionalTrilinearClamped(pos_v_grid);
  }

  inline __device__ __host__
  float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
  {
    // pose of point in whole sdf
    const float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    // convert pos_v to pose in grid sdf
    const float3 pos_v_grid = make_float3( fmod(pos_v.x, float(m_WholeGridRes) ),
                                           fmod(pos_v.y, float(m_WholeGridRes) ),
                                           fmod(pos_v.z, float(m_WholeGridRes) ));

    float fSingleGridLength = float (2 * m_bbox.Max().x/m_WholeGridRes );

    const float3 deriv =
        m_GridVolumes[ int(pos_v.x/fSingleGridLength) + m_WholeGridRes * ( int(pos_v.y/fSingleGridLength) + m_WholeGridRes * int( pos_v.z/m_WholeGridRes ))].GetFractionalBackwardDiffDxDyDz(pos_v_grid);

    return deriv / VoxelSizeUnits();
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
