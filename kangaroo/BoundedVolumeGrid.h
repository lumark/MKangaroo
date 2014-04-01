#pragma once

//#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "BoundedVolume.h"
#include "Sdf.h"
#include "cu_sdffusion.h"

namespace roo
{

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
class BasicSDF
{
public:
  inline __host__
  BasicSDF(T, Target, Management)
  {}


  inline __host__
  BasicSDF(unsigned int X, unsigned int Y, unsigned int Z, unsigned int nRes, roo::BoundingBox& rbbox)
    :m_bbox(rbbox),m_Vol(nRes,nRes,nRes,rbbox)
  {
    m_X = X;
    m_Y = Y;
    m_Z = Z;
    m_bbox = rbbox;
    printf("[BasicSDF] init basic bounded volume grid success. bbox min x is %f, input min is %f\n", m_bbox.boxmin.x, rbbox.boxmin.x);
  }


  inline __host__
  void Reset(){}

public:
  unsigned int                                      m_X;           // index x
  unsigned int                                      m_Y;           // index y
  unsigned int                                      m_Z;           // index z

  roo::BoundedVolume<T, Target, Management>         m_Vol;          // volume
  roo::BoundingBox                                  m_bbox; // bounding box for volume
};

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

  //////////////////////////////////////////////////////
  // Constructors
  //////////////////////////////////////////////////////

  template<typename ManagementCopyFrom> inline __host__ __device__
  BoundedVolumeGrid( const BoundedVolume<T,Target,ManagementCopyFrom>& vol )
  //    :m_VoidVol(vol),bbox(vol.bbox)
  {
    printf("[1]function not support yet!!\n");
  }

  template<typename ManagementCopyFrom> inline __host__ __device__
  BoundedVolumeGrid(const Volume<T,Target,ManagementCopyFrom>& vol, const BoundingBox& bbox)
  //    :m_VoidVol(vol), bbox(bbox)
  {
    printf("[2]function not support yet!!\n");
  }

  inline __host__ __device__
  BoundedVolumeGrid()
  {
  }

  inline __host__
  BoundedVolumeGrid(unsigned int n_w, unsigned int n_h, unsigned int n_d)
    :m_bbox(make_float3(-1,-1,-1), make_float3(1,1,1))
  {
    m_w=n_w;
    m_h=n_h;
    m_d=n_d;
  }

  inline __host__
  BoundedVolumeGrid(unsigned int n_w, unsigned int n_h, unsigned int n_d, const BoundingBox& r_bbox )
    : m_bbox(r_bbox)
  {
    m_w=n_w;
    m_h=n_h;
    m_d=n_d;
    m_bbox = r_bbox;

    printf("[BoundedVolumeGrid] init bounded volume grid success. x%d,y%d,z%d is, bbox min x is %f, input %f\n",n_w,n_h,n_d, m_bbox.boxmin.x, r_bbox.boxmin.x);
  }



  inline __host__
  BoundedVolumeGrid(unsigned int n_w, unsigned int n_h, unsigned int n_d, float3 min_bounds, float3 max_bounds)
    :m_bbox(min_bounds,max_bounds)
  {
    m_w=n_w;
    m_h=n_h;
    m_d=n_d;
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

//  // get element
//  inline  __device__ __host__
//  T& operator()(size_t x, size_t y, size_t z)
//  {
//    //    // try access element
//    //    // access element. convert x, y, z to basic sdf index
//    //    float x_element = x/10;
//    //    float y_element = y/10;
//    //    float z_element = z/10;

//    ////    printf("current size is %d",m_mActiveSDFs.size());
//    //    printf("before iteration.");
//    //    for(int i=0;i!=m_mActiveSDFs.size();i++)
//    //    {
//    //      printf("in iteration;");
//    //      BasicSDF<T,Target, Management>& rBasicSDF = m_mActiveSDFs[i];
//    //      if(rBasicSDF.m_X == size_t(x_element) &&
//    //         rBasicSDF.m_Y == size_t(y_element) &&
//    //         rBasicSDF.m_Z == size_t(z_element)  )
//    //      {
//    //        // access element
//    //        printf("find element.");
//    //        return rBasicSDF.m_Vol(x_element,y_element,z_element);
//    //      }
//    //    }

//    //    printf("does not find element. return zero");

//    //    return m_VoidVol(0,0,0);


//    // try access element
//    // access element. convert x, y, z to basic sdf index
//    float x_element = x/32;
//    float y_element = y/32;
//    float z_element = z/32;

//    //    printf("current size is %d",m_mActiveSDFs.size());
//    printf("before iteration.");

//    for(int i=0;i!=m_mActiveSDFsVolum.w; i++)
//    {
//      for(int j=0;j!=m_mActiveSDFsVolum.h;j++)
//      {
//        for(int k=0;k!=m_mActiveSDFsVolum.d;k++)
//        {
//          printf("in iteration;");
//          BasicSDF<T,Target, Management>& rBasicSDF = m_mActiveSDFsVolum(i,j,k);
//          if(rBasicSDF.m_X == size_t(x_element) &&
//             rBasicSDF.m_Y == size_t(y_element) &&
//             rBasicSDF.m_Z == size_t(z_element)  )
//          {
//            // access element
//            printf("find element.");
//            return rBasicSDF.m_Vol(x_element,y_element,z_element);
//          }
//        }
//      }
//    }

//    printf("does not find element. return zero");

//    return m_VoidVol(0,0,0);
//  }

  // get element
  inline  __device__ __host__
  void GetElement(T& r_val, size_t x, size_t y, size_t z)
  {
    // try access element
    // access element. convert x, y, z to basic sdf index
    float x_element = x/32;
    float y_element = y/32;
    float z_element = z/32;

    //    printf("current size is %d",m_mActiveSDFs.size());
    printf("before iteration.");

    for(int i=0;i!=m_mActiveSDFsVolum.w; i++)
    {
      for(int j=0;j!=m_mActiveSDFsVolum.h;j++)
      {
        for(int k=0;k!=m_mActiveSDFsVolum.d;k++)
        {
          printf("in iteration;");
          BasicSDF<T,Target, Management>& rBasicSDF = m_mActiveSDFsVolum(i,j,k);
          if(rBasicSDF.m_X == size_t(x_element) &&
             rBasicSDF.m_Y == size_t(y_element) &&
             rBasicSDF.m_Z == size_t(z_element)  )
          {
            // access element
            printf("find element.");
            r_val = rBasicSDF.m_Vol(x_element,y_element,z_element);
            break;
          }
        }
      }
    }

    printf("get element fail.");
  }

  // get element
  inline  __device__ __host__
  float GetElementFloat(size_t x, size_t y, size_t z)
  {
    float fval = 0;
    GetElement(fval,x,y,z);
    return fval;
  }


  // get element
  inline  __device__ __host__
  SDF_t GetElementSdf(size_t x, size_t y, size_t z)
  {
    SDF_t  val;
    val.val = 0;
    val.w   = 0;
    GetElement(val, x, y, z);
    return val;
  }



  inline __device__ __host__
  void SetElement(size_t x, size_t y, size_t z, T val)
  {
    // try access element
    // access element. convert x, y, z to basic sdf index
    float x_element = x/32;
    float y_element = y/32;
    float z_element = z/32;

    bool bFlag = false;

    for(int i=0;i!=m_mActiveSDFsVolum.w; i++)
    {
      for(int j=0;j!=m_mActiveSDFsVolum.h;j++)
      {
        for(int k=0;k!=m_mActiveSDFsVolum.d;k++)
        {
          // printf("in iteration;");
          BasicSDF<T,Target, Management>& rBasicSDF = m_mActiveSDFsVolum(i,j,k);
          if(rBasicSDF.m_X == size_t(x_element) &&
             rBasicSDF.m_Y == size_t(y_element) &&
             rBasicSDF.m_Z == size_t(z_element)  )
          {
            // access element
            // access element
            printf("set element directlly;");
            rBasicSDF.m_Vol(x_element,y_element,z_element) = val;
            bFlag =true;
          }
        }
      }
    }

    // init new basic SDF
    if(bFlag==false)
    {
      printf("try init new sdf for set element;");

      BasicSDF<T, Target, Management> nBasicSDF(size_t(x_element), size_t(y_element), size_t(z_element), m_res, m_bbox);
      nBasicSDF.m_Vol(x_element,y_element,z_element) = val;
      m_mActiveSDFsVolum(x,y,z) = nBasicSDF;

      printf("init new sdf for set element success;");
    }
  }


  //  inline  __device__ __host__
  //  float GetUnitsTrilinearClamped(float3 pos_w) const
  //  {
  //    //    const float3 pos_v = (pos_w - bbox.Min()) / (bbox.Size());
  //    //    return Volume<T,Target,Management>::GetFractionalTrilinearClamped(pos_v);
  //  }

  //  inline __device__ __host__
  //  float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
  //  {
  //    //        const float3 pos_v = (pos_w - bbox.Min()) / (bbox.Size());
  //    //        const float3 deriv = Volume<T,Target,Management>::GetFractionalBackwardDiffDxDyDz(pos_v);
  //    //        return deriv / VoxelSizeUnits();
  //  }

  //  inline __device__ __host__
  //  float3 GetUnitsOutwardNormal(float3 pos_w) const
  //  {
  //    const float3 deriv = GetUnitsBackwardDiffDxDyDz(pos_w);
  //    return deriv / length(deriv);
  //  }

  inline __device__ __host__
  float3 VoxelPositionInUnits(int x, int y, int z) const
  {
    const float3 vol_size = m_bbox.Size();

    //    printf("[VoxelPositionInUnits] bbmin_x:%f", m_bbox.Min().x + vol_size.x*x/(float)(m_w-1));

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

  //////////////////////////////////////////////////////
  // Access sub-regions
  //////////////////////////////////////////////////////

  inline __device__ __host__
  BoundedVolumeGrid<T,Target,DontManage> SubBoundingVolume(const BoundingBox& region)
  {
    //    //        const float3 min_fv = (region.Min() - bbox.Min()) / (bbox.Size());
    //    //        const float3 max_fv = (region.Max() - bbox.Min()) / (bbox.Size());

    //    //        const int3 min_v = make_int3(
    //    //            fmaxf((Volume<T,Target,Management>::w-1)*min_fv.x, 0),
    //    //            fmaxf((Volume<T,Target,Management>::h-1)*min_fv.y, 0),
    //    //            fmaxf((Volume<T,Target,Management>::d-1)*min_fv.z, 0)
    //    //        );
    //    //        const int3 max_v = make_int3(
    //    //            fminf(ceilf((Volume<T,Target,Management>::w-1)*max_fv.x), Volume<T,Target,Management>::w-1),
    //    //            fminf(ceilf((Volume<T,Target,Management>::h-1)*max_fv.y), Volume<T,Target,Management>::h-1),
    //    //            fminf(ceilf((Volume<T,Target,Management>::d-1)*max_fv.z), Volume<T,Target,Management>::d-1)
    //    //        );

    //    //        const int3 size_v = max((max_v - min_v) + make_int3(1,1,1), make_int3(0,0,0) );

    //    //        const BoundingBox nbbox(
    //    //            VoxelPositionInUnits(min_v),
    //    //            VoxelPositionInUnits(max_v)
    //    //        );

    //    //        return BoundedVolumeGrid<T,Target,DontManage>(
    //    //            Volume<T,Target,Management>::SubVolumeGrid(min_v, size_v),
    //    //            nbbox
    //    //        );

    printf("[1]function not support yet!!\n");

    //    return

  }

  size_t                                     m_d;
  size_t                                     m_w;
  size_t                                     m_h;
  size_t                                     m_res;
  // bounding box ofbounded volume grid
  BoundingBox                                m_bbox;

  size_t                                     m_CubeLength;
  Volume< BasicSDF<T> >                      m_mActiveSDFsVolum;
};

}
