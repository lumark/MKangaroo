#pragma once

#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "BoundedVolume.h"
#include "VolumeGrid.h"
#include "Sdf.h"
#include "cu_sdffusion.h"


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
    m_res  = n_res;
    m_bbox = r_bbox;

    for(int i=0;i!=64; i++)
    {
      m_GridVolumes[i].InitVolume(m_res,m_res,m_res);
      SdfReset(m_GridVolumes[i]);
    }

    printf("[BoundedVolumeGrid] init bounded volume grid success. x=%d,y=%d,z=%d is, bbox min x is %f, input %f\n",
           m_w,m_h,m_d, m_bbox.boxmin.x, r_bbox.boxmin.x);
  }


  inline __device__ __host__
  T GetVal(int x, int y, int z)
  {
    return m_GridVolumes[0](x,y,z);
  }


  inline __device__
  void SetVal(T val,int x, int y, int z)
  {
    m_GridVolumes[0](0,0,0) = val;
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

  inline __device__ __host__
  bool IsValid() const {
    const uint3 size = Volume<T,Target,Management>::Voxels();
    return size.x >= 8 && size.y >= 8 && size.z >= 8;
  }

  //////////////////////////////////////////////////////
  // Access VolumeGrid in units of Bounding Box
  //////////////////////////////////////////////////////
  // get element
  inline  __device__ __host__
  void GetElement(T& r_val, size_t x, size_t y, size_t z)
  {
    // try access element
    // access element. convert x, y, z to basic sdf index
    //    float x_element = x/m_res;
    //    float y_element = y/m_res;
    //    float z_element = z/m_res;

    //    printf("%d;",int(x_element));

    //    printf("%d",int(m_GridVolume[int(x_element)][int(y_element) ][int(z_element)]->w));

    //    if(m_GridVolume[int(x_element)][int(y_element) ][int(z_element)]->w == 64)
    //    {
    //      printf("find element.");

    //    r_val = m_GridVolume[int(x_element)][int(y_element)][int(z_element)]->Get(x_element,y_element,z_element);

    //    r_val = m_GridVolume[0][0][0]->Get(0,0,0);

    //      //        break;
    //    }

    //    printf("get element fail.");
  }


  inline  __device__ __host__
  void SetElement(T& r_val, size_t x, size_t y, size_t z)
  {
    // try access element
    // access element. convert x, y, z to basic sdf index
    //    float x_element = x/m_res;
    //    float y_element = y/m_res;
    //    float z_element = z/m_res;

    //    printf("%d;",int(x_element));

    //    printf("%d",int(m_GridVolume[int(x_element)][int(y_element) ][int(z_element)]->w));

    //    if(m_GridVolume[int(x_element)][int(y_element) ][int(z_element)]->w == 64)
    //    {
    //      printf("find element.");

    //    r_val = m_GridVolume[int(x_element)][int(y_element)][int(z_element)]->Get(x_element,y_element,z_element);

    //    m_GridVolume[0][0][0]->Get(0,0,0) = 1;

    //      //        break;
    //    }

    //    printf("get element fail.");
  }


  // get element
  inline  __device__
  float GetElementFloat(size_t x, size_t y, size_t z)
  {
    float fval = 0;
    GetElement(fval,x,y,z);
    return fval;
  }

  // get element
  inline  __device__
  SDF_t GetElementSdf(size_t x, size_t y, size_t z)
  {
    SDF_t   val;
    val.val = 1;
    val.w   = 0;
    GetElement(val, x, y, z);
    return val;
  }


  inline __device__
  void SetElementVolume(size_t x, size_t y, size_t z, T val)
  {
    // try access element
    // access element. convert x, y, z to basic sdf index
    float x_element = x/m_res;
    float y_element = y/m_res;
    float z_element = z/m_res;

    printf("try to set element;");

    for(int i=0;i!=4; i++)
    {
      for(int j=0;j!=4;j++)
      {
        for(int k=0;k!=4;k++)
        {
          //          // printf("in iteration;");
          //          BasicSDF<T,Target, Manage>* rBasicSDF = m_GridVolume(i,j,k);
          //          if(rBasicSDF->m_X == int(x_element) &&
          //             rBasicSDF->m_Y == int(y_element) &&
          //             rBasicSDF->m_Z == int(z_element)  )
          //          {
          //            // access element
          //            rBasicSDF->m_Vol(x_element,y_element,z_element) = val;
          //            printf("set element val success;");
          //          }
        }
      }
    }
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
  size_t                                                m_res;

  // bounding box ofbounded volume grid
  BoundingBox                                           m_bbox;

  VolumeGrid<T, TargetDevice, Manage>                   m_GridVolumes[64];
};


}
