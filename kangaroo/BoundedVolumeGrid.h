#pragma once

#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "Sdf.h"


namespace roo
{

// =============================================================================
// A BoundedVolumeGrid consist n numbers of single volume. Each volume is
// (m_BasicGridRes*m_BasicGridRes*m_BasicGridRes) cube.
// Using this function requires calling roo::SDFInitGreyGrid first.
// =============================================================================

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
class BoundedVolumeGrid
{
public:

  // ===========================================================================
  // we cannot implement a constructor for this because we have to use this class
  // as global variables in kernel function, which does not allow us to have a
  // constructor.
  // the reason we have to use this as global veriables in kernel function is that
  // it may contain more than 16 VolumeGrid, which over the maximum size of the
  // parameters that we can pass into a kernel function
  // ===========================================================================
  inline __host__
  void init(unsigned int n_w, unsigned int n_h, unsigned int n_d, unsigned int n_res, const BoundingBox& r_bbox)
  {
    // init grid sdf
    if(n_w == n_h && n_h == n_d)
    {
      m_w             = n_w;
      m_h             = n_h;
      m_d             = n_d;

      m_bbox          = r_bbox;
      m_nVolumeGridRes = n_res;
      m_nWholeGridRes  = m_w/m_nVolumeGridRes;

      m_shift = make_int3(0,0,0);
    }
    else
    {
      printf("[BoundedVolumeGrid] Fatal error! Only support cube SDF (n*n*n)!\n");
      exit(-1);
    }
  }


  // the following code init all basic SDFs directlly.
  // if we don't use this code right after init, we will have to call roo::SDFInitGreyGrid
  // before fusing
  inline __host__
  void InitAllBasicSDFs()
  {
    for(int i=0;i!=m_w;i++)
    {
      for(int j=0;j!=m_h;j++)
      {
        for(int k=0;k!=m_d;k++)
        {
          InitSingleBasicSDFWithGridIndex(i,j,k);
        }
      }
    }
  }


  inline __host__
  void InitSingleBasicSDFWithGridIndex(unsigned int x, unsigned int y, unsigned int z)
  {
    int nIndex =GetIndex( int(floorf(x/m_nVolumeGridRes)),
                          int(floorf(y/m_nVolumeGridRes)),
                          int(floorf(z/m_nVolumeGridRes)) );

    if(m_GridVolumes[nIndex].d !=m_nVolumeGridRes)
    {
      m_GridVolumes[nIndex].InitVolume(m_nVolumeGridRes,
                                       m_nVolumeGridRes,
                                       m_nVolumeGridRes);
    }
  }


  inline __host__
  bool InitSingleBasicSDFWithIndex(int nIndex)
  {
    if( m_GridVolumes[nIndex].d !=m_nVolumeGridRes )
    {
      m_GridVolumes[nIndex].InitVolume(m_nVolumeGridRes,
                                       m_nVolumeGridRes,
                                       m_nVolumeGridRes);
      return true;
    }

    return false;
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

  inline __host__
  bool IsValid() const {

    int nNum = 0;

    for(int i=0;i!=m_nWholeGridRes*m_nWholeGridRes*m_nWholeGridRes;i++)
    {
      if(m_GridVolumes[i].d == m_nVolumeGridRes)
      {
        nNum ++;
      }
    }

    return nNum>0 && m_w >= 8 && m_h >= 8 && m_d >= 8;
  }



  inline  __device__
  T& operator()(unsigned int x,unsigned int y, unsigned int z)
  {
    int nIndex = GetIndex( int(floorf(x/m_nVolumeGridRes)),
                           int(floorf(y/m_nVolumeGridRes)),
                           int(floorf(z/m_nVolumeGridRes)) );

    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      printf("basic sdf does not exist. shift x is %d; x=%d,y=%d,z=%d\n",
             m_shift.x,
             int(floorf(x/m_nVolumeGridRes)),
             int(floorf(y/m_nVolumeGridRes)),
             int(floorf(z/m_nVolumeGridRes)) );
    }

    return m_GridVolumes[nIndex](x%m_nVolumeGridRes, y%m_nVolumeGridRes, z%m_nVolumeGridRes);
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

    const float fFactor = float(m_nVolumeGridRes)/float(m_w);

    // Get the index of voxel in basic sdf
    const uint3 Index =make_uint3( floorf(pos_v.x/fFactor),
                                   floorf(pos_v.y/fFactor),
                                   floorf(pos_v.z/fFactor)  );

    int nIndex = GetIndex( Index.x, Index.y, Index.z);

    if(CheckIfBasicSDFActive(nIndex)==false)
    {
      return 0.0/0.0;
    }

    /// get axis.
    float3 pos_v_grid = make_float3( fmod(pos_v.x,fFactor) /fFactor,
                                     fmod(pos_v.y,fFactor) /fFactor,
                                     fmod(pos_v.z,fFactor) /fFactor );

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

    const float fFactor = float(m_nVolumeGridRes)/float(m_w);

    // Get the index of voxel in basic sdf
    const uint3 Index =make_uint3( floorf(pos_v.x/fFactor),
                                   floorf(pos_v.y/fFactor),
                                   floorf(pos_v.z/fFactor)  );

    int nIndex = GetIndex( Index.x, Index.y, Index.z);

    if(CheckIfBasicSDFActive(nIndex)==false)
    {
      return make_float3(0.0/0.0,0.0/0.0,0.0/0.0);
    }

    /// get axis.
    float3 pos_v_grid = make_float3( fmod(pos_v.x,fFactor) /fFactor,
                                     fmod(pos_v.y,fFactor) /fFactor,
                                     fmod(pos_v.z,fFactor) /fFactor );

    const float3 deriv = m_GridVolumes[nIndex].GetFractionalBackwardDiffDxDyDz(pos_v_grid);

    return deriv / VoxelSizeUnits();
  }

  // ============================================================================
  inline __device__ __host__
  unsigned int GetIndex(unsigned int x, unsigned int y, unsigned int z) const
  {
    if(m_shift.x==0 && m_shift.y == 0 && m_shift.z ==0)
    {
      const unsigned int nIndex =x + m_nWholeGridRes* (y+ m_nWholeGridRes* z);
      return  nIndex;
    }

    // for x
    if(m_shift.x>0 && m_shift.x<=m_nWholeGridRes)
    {
      if( x<=m_nWholeGridRes-1-m_shift.x)
      {
        x = x+m_shift.x;
      }
      else if(x>=m_nWholeGridRes-1-m_shift.x)
      {
        x = x-(m_nWholeGridRes-1)+(m_shift.x-1);
      }
    }
    else if(m_shift.x<0 && m_shift.x>=-m_nWholeGridRes)
    {
      if(x>=abs(m_shift.x) && x<=m_nWholeGridRes)
      {
        x = x+m_shift.x;
      }
      else if( x<=abs(m_shift.x) )
      {
        x = x+m_nWholeGridRes-1-abs(m_shift.x) ;
      }
    }



    // for y
    if(m_shift.y>0 && m_shift.y<=m_nWholeGridRes)
    {
      if( y<=m_nWholeGridRes-1-m_shift.y)
      {
        y = y+m_shift.y;
      }
      else if(y>=m_nWholeGridRes-1-m_shift.y)
      {
        y = y-(m_nWholeGridRes-1)+(m_shift.y-1);
      }
    }
    else if(m_shift.y<0 && m_shift.y>=-m_nWholeGridRes)
    {
      if(y>=abs(m_shift.y) && y<=m_nWholeGridRes)
      {
        y = y+m_shift.y;
      }
      else if( y<=abs(m_shift.y) )
      {
        y = y+m_nWholeGridRes-1-abs(m_shift.y) ;
      }
    }



    // for z
    if(m_shift.z>0 && m_shift.z<=m_nWholeGridRes)
    {
      if( z<=m_nWholeGridRes-1-m_shift.z)
      {
        z = z+m_shift.z;
      }
      else if(z>=m_nWholeGridRes-1-m_shift.z)
      {
        z = z-(m_nWholeGridRes-1)+(m_shift.z-1);
      }
    }
    else if(m_shift.z<0 && m_shift.z>=-m_nWholeGridRes)
    {
      if(z>=abs(m_shift.z) && z<=m_nWholeGridRes)
      {
        z = z+m_shift.z;
      }
      else if( z<=abs(m_shift.z) )
      {
        z = z+m_nWholeGridRes-1-abs(m_shift.z) ;
      }
    }


    // compute actual index
    const unsigned int nIndex =x + m_nWholeGridRes* (y+ m_nWholeGridRes* z);
    return  nIndex;
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
    for(int i=0;i!= m_nWholeGridRes*m_nWholeGridRes*m_nWholeGridRes;i++)
    {
      m_GridVolumes[i].MemcpyFromDevice(rVol.m_GridVolumes[i]);
    }
  }

  inline __host__
  void FreeMemory()
  {
    for(int i=0;i!=m_nWholeGridRes*m_nWholeGridRes*m_nWholeGridRes;i++)
    {
      cudaFree( m_GridVolumes[i].ptr );
    }
  }

  inline __host__
  void FreeMemoryByIndex(unsigned int nIndex)
  {
    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      printf("[BoundedVolumeGrid] fatal error! pointer being free must be allocate first!!\n");
      exit(-1);
    }

    cudaFree( m_GridVolumes[nIndex].ptr );
  }


  inline __host__ __device__
  bool CheckIfBasicSDFActive(const int nIndex) const
  {
    if(m_GridVolumes[nIndex].d == m_nVolumeGridRes &&
       m_GridVolumes[nIndex].w == m_nVolumeGridRes &&
       m_GridVolumes[nIndex].h == m_nVolumeGridRes)
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

    for(int i=0;i!=m_nWholeGridRes*m_nWholeGridRes*m_nWholeGridRes;i++)
    {
      if(m_GridVolumes[i].d == m_nVolumeGridRes &&
         m_GridVolumes[i].w == m_nVolumeGridRes &&
         m_GridVolumes[i].h == m_nVolumeGridRes)
      {
        nNum ++;
      }
    }

    return nNum;
  }


  // set next sdf that we want to init
  inline __device__
  void SetNextInitSDF(unsigned int x, unsigned int y, unsigned int z)
  {
    const int nIndex = GetIndex(x/m_nVolumeGridRes, y/m_nVolumeGridRes, z/m_nVolumeGridRes );

    if(m_NextInitBasicSDFs[nIndex] == 0 && CheckIfBasicSDFActive(nIndex) == true)
    {
      m_NextInitBasicSDFs[nIndex] = 1;
    }
  }


  inline __host__
  void UpdateShift(int3 shift_index)
  {
    m_shift = m_shift + shift_index;

    if(m_shift.x == m_nWholeGridRes ||  m_shift.x == -m_nWholeGridRes)
    {
      m_shift.x = 0;
      printf("[BoundedVolumeGrid] Set shift x back to zero! \n");
    }

    if(m_shift.y == m_nWholeGridRes || m_shift.y == -m_nWholeGridRes)
    {
      m_shift.y = 0;
      printf("[BoundedVolumeGrid] Set shift y back to zero! \n");
    }

    if(m_shift.z == m_nWholeGridRes || m_shift.z == -m_nWholeGridRes)
    {
      m_shift.z = 0;
      printf("[BoundedVolumeGrid] Set shift z back to zero! \n");
    }

    printf("[BoundedVolumeGrid] Update Shift success! current shift x=%d,y=%d,z=%d; Max shift is %d \n",
           m_shift.x,m_shift.y,m_shift.z, m_nWholeGridRes);
  }



public:
  size_t                                      m_d;
  size_t                                      m_w;
  size_t                                      m_h;

  // for rolling sdf. When this value is not zero, we need to recompute index based on the shift
  int3                                        m_shift;

  // bounding box ofbounded volume grid
  BoundingBox                                 m_bbox;

  unsigned int                                m_nVolumeGridRes;   // resolution of a single grid in one dim.
  unsigned int                                m_nWholeGridRes;    // resolution of a whole grid in one dim. usually 4, 8, 16

  // volume that save all data
  VolumeGrid<T, TargetDevice, Manage>         m_GridVolumes[1024];
  int                                         m_NextInitBasicSDFs[1024];  // an array that record basic SDFs we want to init
};


}
