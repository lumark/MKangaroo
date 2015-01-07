#pragma once

#include "VolumeGrid.h"
#include "kangaroo/BoundingBox.h"
#include "kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"
#include "kangaroo/launch_utils.h"

namespace roo
{

// =============================================================================
// A BoundedVolumeGrid consist n numbers of single volume. Each volume is
// (m_BasicGridRes*m_BasicGridRes*m_BasicGridRes) cube.
// Using this function requires calling roo::SDFInitGreyGrid first.
// =============================================================================

const int MAX_SUPPORT_GRID_NUM = 13824;

template<typename T, typename Target = TargetDevice, typename Management = DontManage>
class BoundedVolumeGrid
{
public:

  // ===========================================================================
  // we cannot implement a constructor for this as we have to use this class
  // as global variables in kernel function, which forbid us to have a constructor.
  // the reason we have to use this as global veriables in kernel function is that
  // it may contain more than 16 VolumeGrid, which over the maximum size of the
  // parameters that we can pass into a kernel function
  // ===========================================================================
  inline __host__
  void Init(
      unsigned int       n_w,   // num of voxels of the volume in width
      unsigned int       n_h,   // num of voxels of the volume in height
      unsigned int       n_d,   // num of voxels of the volume in depth
      unsigned int       n_res, // resolution of a single grid volume
      const BoundingBox& r_bbox)
  {
    m_w = n_w; m_h = n_h; m_d = n_d;

    m_bbox           = r_bbox;
    m_nVolumeGridRes = n_res;

    m_nGridRes_w     = m_w/m_nVolumeGridRes;
    m_nGridRes_h     = m_h/m_nVolumeGridRes;
    m_nGridRes_d     = m_d/m_nVolumeGridRes;

    m_nTotalGridRes = m_nGridRes_w * m_nGridRes_h * m_nGridRes_d;

    if(m_nTotalGridRes > MAX_SUPPORT_GRID_NUM)
    {
      std::cerr<<"[BoundedVolumeGrid/init] Error! overflow! Max allow "<<
                 MAX_SUPPORT_GRID_NUM<<"; req "<<m_nTotalGridRes<<
                 ";Please check VOL_RES and VOL_GRID_RES setting !"<<std::endl;
      exit(-1);
    }

    // -----------------------------------------------------------------------
    ResetAllGridVol();
    m_local_shift = make_int3(0,0,0);
    m_global_shift = make_int3(0,0,0);

    if(n_w != n_h || n_h != n_d || n_w!=n_d)
    {
      std::cerr<<"[BoundedVolumeGrid/init] suggest use cube size SDF!"<<std::endl;
    }
  }

  inline __host__
  void ResetAllGridVol()
  {
    for(int i=0;i!=MAX_SUPPORT_GRID_NUM;i++)
    {
      //      if(CheckIfBasicSDFActive(i)==true)
      //      {
      m_GridVolumes[i].d = 0;
      m_GridVolumes[i].w = 0;
      m_GridVolumes[i].h = 0;
      //        m_GridVolumes[i].CleanUp();
      //        GpuCheckErrors();
      //      }
    }
  }

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
  void InitSingleBasicSDFWithGridIndex(
      unsigned int x,
      unsigned int y,
      unsigned int z)
  {
    int nIndex = ConvertLocalIndexToRealIndex(
          static_cast<int>(floorf(x/m_nVolumeGridRes)),
          static_cast<int>(floorf(y/m_nVolumeGridRes)),
          static_cast<int>(floorf(z/m_nVolumeGridRes)) );

    if(m_GridVolumes[nIndex].d !=m_nVolumeGridRes &&
       m_GridVolumes[nIndex].h !=m_nVolumeGridRes &&
       m_GridVolumes[nIndex].w !=m_nVolumeGridRes)
    {
      m_GridVolumes[nIndex].InitVolume(m_nVolumeGridRes,
                                       m_nVolumeGridRes,
                                       m_nVolumeGridRes);
      GpuCheckErrors();
    }
  }

  inline __host__
  bool InitSingleBasicSDFWithIndex(int nIndex)
  {
    if( m_GridVolumes[nIndex].d !=m_nVolumeGridRes &&
        m_GridVolumes[nIndex].h !=m_nVolumeGridRes &&
        m_GridVolumes[nIndex].w !=m_nVolumeGridRes)
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
    return m_bbox.Size() / make_float3( m_w-1, m_h-1, m_d-1 );
  }

  inline __device__ __host__
  float3 VoxelSizeUnitsGlobal(int3 max_global, int3 min_global) const
  {
    return m_bbox.Size() / make_float3( (max_global.x-min_global.x + 1)*(m_w-1),
                                        (max_global.y-min_global.y + 1)*(m_h-1),
                                        (max_global.z-min_global.z + 1)*(m_d-1) );
  }

  //////////////////////////////////////////////////////
  // Tools
  //////////////////////////////////////////////////////

  inline __host__
  bool IsValid() const
  {
    int nNum = 0;
    for(unsigned int i=0;i!=m_nTotalGridRes;i++)
    {
      if(m_GridVolumes[i].d !=m_nVolumeGridRes &&
         m_GridVolumes[i].h !=m_nVolumeGridRes &&
         m_GridVolumes[i].w !=m_nVolumeGridRes)
      {
        nNum ++;
      }
    }
    return nNum>0 && m_w >= 8 && m_h >= 8 && m_d >= 8;
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

    for(int i=0;i!= GetTotalGridNum();i++)
    {
      if(CheckIfBasicSDFActive(i)==true)
      {
        nNum ++;
      }
    }

    return nNum;
  }

  inline __host__ __device__
  bool CheckIfVoxelExist(int x, int y, int z)
  {
    int nIndex = ConvertLocalIndexToRealIndex(
          static_cast<int>(floorf(x/m_nVolumeGridRes)),
          static_cast<int>(floorf(y/m_nVolumeGridRes)),
          static_cast<int>(floorf(z/m_nVolumeGridRes)) );

    if( CheckIfBasicSDFActive(nIndex) == true)
    {
      return true;
    }

    return false;
  }


  //////////////////////////////////////////////////////
  // Access Elements
  //////////////////////////////////////////////////////

  inline __device__ __host__
  uint3 Voxels() const
  {
    return make_uint3(m_w,m_h,m_d);
  }

  inline  __device__
  T& operator()(unsigned int x,unsigned int y, unsigned int z)
  {
    int nIndex = ConvertLocalIndexToRealIndex(
          static_cast<int>(floorf(x/m_nVolumeGridRes)),
          static_cast<int>(floorf(y/m_nVolumeGridRes)),
          static_cast<int>(floorf(z/m_nVolumeGridRes)) );

    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      printf("BoundedVolumeGrid] Fatal Error! BasicSDF doesn't exist."
             "shift (%d,%d,%d); index (%d,%d,%d); Max index (%d,%d,%d)\n",
             m_local_shift.x, m_local_shift.y, m_local_shift.z,
             static_cast<int>(floorf(x/m_nVolumeGridRes)),
             static_cast<int>(floorf(y/m_nVolumeGridRes)),
             static_cast<int>(floorf(z/m_nVolumeGridRes)),
             m_nGridRes_w-1, m_nGridRes_h-1, m_nGridRes_d-1);
    }

    return m_GridVolumes[nIndex](x%m_nVolumeGridRes, y%m_nVolumeGridRes, z%m_nVolumeGridRes);
  }

  inline  __device__  __host__
  T& Get(unsigned int x,unsigned int y, unsigned int z)
  {
    int nIndex = ConvertLocalIndexToRealIndex(
          static_cast<int>(floorf(x/m_nVolumeGridRes)),
          static_cast<int>(floorf(y/m_nVolumeGridRes)),
          static_cast<int>(floorf(z/m_nVolumeGridRes)) );

    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      //      return 0.0/0.0;
    }

    return m_GridVolumes[nIndex](x%m_nVolumeGridRes, y%m_nVolumeGridRes, z%m_nVolumeGridRes);
  }

  // input pos_w in meter
  inline  __device__
  float GetUnitsTrilinearClamped(float3 pos_w) const
  {
    /// get pose of voxel in whole sdf, in %
    float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    if(pos_v.x>=1) { pos_v.x =0.99999f; }
    else if(pos_v.x<0) { pos_v.x =0.f; }

    if(pos_v.y>=1) { pos_v.y =0.99999f; }
    else if(pos_v.y<0) { pos_v.y =0.f; }

    if(pos_v.z>=1) { pos_v.z =0.99999f; }
    else if(pos_v.z<0) { pos_v.z =0.f; }

    const float fFactor = static_cast<float>(m_nVolumeGridRes)/static_cast<float>(m_w);

    // Get the index of voxel in basic sdf
    const uint3 Index =make_uint3( floorf(pos_v.x/fFactor),
                                   floorf(pos_v.y/fFactor),
                                   floorf(pos_v.z/fFactor) );

    int nIndex = ConvertLocalIndexToRealIndex( Index.x, Index.y, Index.z);

    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      return 0.0/0.0;
    }

    /// get axis.
    float3 pos_v_grid = make_float3( fmod(pos_v.x, fFactor) /fFactor,
                                     fmod(pos_v.y, fFactor) /fFactor,
                                     fmod(pos_v.z, fFactor) /fFactor );

    return m_GridVolumes[nIndex].GetFractionalTrilinearClamped(pos_v_grid);
  }

  inline __device__
  float3 GetUnitsBackwardDiffDxDyDz(float3 pos_w) const
  {
    /// get pose of voxel in whole sdf, in %
    float3 pos_v = (pos_w - m_bbox.Min()) / (m_bbox.Size());

    if(pos_v.x>=1){pos_v.x =0.99999f;}
    else if(pos_v.x<0) { pos_v.x =0.f;}

    if(pos_v.y>=1){pos_v.y =0.99999f;}
    else if(pos_v.y<0){pos_v.y =0.f;}

    if(pos_v.z>=1){pos_v.z =0.99999f;}
    else if(pos_v.z<0){pos_v.z =0.f;}

    const float fFactor = static_cast<float>(m_nVolumeGridRes)/static_cast<float>(m_w);

    // Get the index of voxel in basic sdf
    const uint3 Index = make_uint3( floorf(pos_v.x/fFactor),
                                    floorf(pos_v.y/fFactor),
                                    floorf(pos_v.z/fFactor) );

    int nIndex = ConvertLocalIndexToRealIndex( Index.x, Index.y, Index.z);

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

  inline __device__
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
          m_bbox.Min().x + vol_size.x * static_cast<float>(x)/static_cast<float>(m_w-1),
          m_bbox.Min().y + vol_size.y * static_cast<float>(y)/static_cast<float>(m_h-1),
          m_bbox.Min().z + vol_size.z * static_cast<float>(z)/static_cast<float>(m_d-1)
          );
  }

  inline __device__ __host__
  float3 VoxelPositionInUnits(int3 p_v) const
  {
    return VoxelPositionInUnits(p_v.x,p_v.y,p_v.z);
  }


  // use in the marching cubes algorithm
  inline __host__
  float3 VoxelPositionInUnitsGlobal(
      int x, int y, int z,
      int3 cur_global, int3 max_global, int3 min_global) const
  {
    if(x>=m_w || y>= m_h || z>=m_d)
    {
      printf("[VoxelPositionInUnitsGlobal] Index overflow! (%d,%d,%d), dim: (%d,%d,%d)\n",
             x,y,z,static_cast<int>(m_w),static_cast<int>(m_h),static_cast<int>(m_d));
    }

    float3 local_pos =  make_float3(
          m_bbox.Min().x + m_bbox.Size().x * static_cast<float>(x)/static_cast<float>(m_w-1),
          m_bbox.Min().y + m_bbox.Size().y * static_cast<float>(y)/static_cast<float>(m_h-1),
          m_bbox.Min().z + m_bbox.Size().z * static_cast<float>(z)/static_cast<float>(m_d-1)
          );

    // ------------------------------------------------------------------------
    float3 global_pos = make_float3(0,0,0);

    // there is a case when max_global index and min global index equals to 0;
    // this check helps avoid inf global pos
    if(max_global.x != min_global.x)
    {
      global_pos.x = ( static_cast<float>(cur_global.x-min_global.x) + local_pos.x) /
          static_cast<float>(max_global.x-min_global.x);
    }
    else
    {
       if(max_global.x + min_global.x !=0)
       {
         std::cerr<<"[VoxelPositionInUnitsGlobal] Fatal error!"<<std::endl;
         exit(-1);
       }
    }

    if(max_global.y != min_global.y)
    {
     global_pos.y = (static_cast<float>(cur_global.y-min_global.y) + local_pos.y)/
        static_cast<float>(max_global.y-min_global.y);
    }
    else
    {
       if(max_global.y + min_global.y !=0)
       {
         std::cerr<<"[VoxelPositionInUnitsGlobal] Fatal error!"<<std::endl;
         exit(-1);
       }
    }

    if(max_global.z != min_global.z)
    {
      global_pos.z = (static_cast<float>(cur_global.z-min_global.z) + local_pos.z)/
          static_cast<float>(max_global.z-min_global.z);
    }
    else
    {
       if(max_global.z + min_global.z !=0)
       {
         std::cerr<<"[VoxelPositionInUnitsGlobal] Fatal error!"<<std::endl;
         exit(-1);
       }
    }

    //    printf("LocalPos:(%f,%f,%f) GlobalPos(%f,%f,%f)",
    //           local_pos.x,local_pos.y,local_pos.z,
    //           global_pos.x,global_pos.y,global_pos.z);

    return global_pos;
  }


  //////////////////////////////////////////////////////
  // Copy and Free Memory
  //////////////////////////////////////////////////////
  inline __host__
  void CopyFrom(BoundedVolumeGrid<T, TargetDevice, Management>& rVol )
  {
    for(unsigned int i=0;i!= GetTotalGridNum();i++)
    {
      m_GridVolumes[i].CopyFrom(rVol.m_GridVolumes[i]);
    }
  }

  inline __host__
  void CopyFrom(BoundedVolumeGrid<T, TargetHost, Management>& rVol )
  {
    for(unsigned int i=0;i!= GetTotalGridNum();i++)
    {
      m_GridVolumes[i].CopyFrom(rVol.m_GridVolumes[i]);
    }
  }

  inline __host__
  void CopyAndInitFrom(BoundedVolumeGrid<T, TargetDevice , Management>& rVol )
  {
    for(unsigned int i=0;i!= GetTotalGridNum();i++)
    {
      // skip void volum grid
      if(rVol.CheckIfBasicSDFActive(i) == true)
      {
        if(CheckIfBasicSDFActive(i) == false)
        {
          if(InitSingleBasicSDFWithIndex(i) == false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Error! Cannot init grid sdf!!\n");
            exit(-1);
          }

          if(CheckIfBasicSDFActive(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Error! Init grid sdf fail!!\n");
            exit(-1);
          }
        }

        m_GridVolumes[i].CopyFrom(rVol.m_GridVolumes[i]);
        GpuCheckErrors();
      }
    }
  }

  inline __host__
  void CopyAndInitFrom(BoundedVolumeGrid<T, TargetHost, Management>& rHVol )
  {
    for(unsigned int i=0; i!= GetTotalGridNum(); i++)
    {
      // skip void volum grid
      if(rHVol.CheckIfBasicSDFActive(i)== true)
      {
        if(CheckIfBasicSDFActive(i)==false)
        {
          if(InitSingleBasicSDFWithIndex(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Error! Cannot init grid sdf!!\n");
            exit(-1);
          }

          if(CheckIfBasicSDFActive(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Error! Init grid sdf fail!!\n");
            exit(-1);
          }
        }
        m_GridVolumes[i].CopyFrom(rHVol.m_GridVolumes[i]);
        GpuCheckErrors();
      }
    }
  }

  inline __host__
  void FreeMemory()
  {
    for(unsigned int i=0;i!= GetTotalGridNum();i++)
    {
      if(m_GridVolumes[i].d == m_nVolumeGridRes &&
         m_GridVolumes[i].h == m_nVolumeGridRes &&
         m_GridVolumes[i].w == m_nVolumeGridRes)
      {
        m_GridVolumes[i].d=0;
        m_GridVolumes[i].w=0;
        m_GridVolumes[i].h=0;
        cudaFree( m_GridVolumes[i].ptr );
      }

    }
  }

  inline __host__
  void FreeMemoryByIndex(unsigned int nIndex)
  {
    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      printf("[BoundedVolumeGrid] Error! Single GridSDF being free must be alloc first!!\n");
      exit(-1);
    }

    cudaFree( m_GridVolumes[nIndex].ptr );
  }


  //////////////////////////////////////////////////////
  // For rolling SDF
  //////////////////////////////////////////////////////
  inline __device__
  float3 GetPrecentagePosInBB(float3 pos_w, float3 cam_translate) const
  {
    // pos_w: world pose of the voxel in the camera frame
    // cam_translate: world pose of the camera
    // this function get pose of the voxel in whole bounding box, in %
    // notice that the bbox is in gobal pose
    float3 final_pose;
    if(pos_w.x>=0)
    {
      final_pose.x =pos_w.x + cam_translate.x - m_bbox.Min().x;
    }
    else
    {
      final_pose.x =pos_w.x + cam_translate.x - m_bbox.Max().x;
    }

    if(pos_w.y>=0)
    {
      final_pose.y =pos_w.y + cam_translate.y - m_bbox.Min().y;
    }
    else
    {
      final_pose.y =pos_w.y + cam_translate.y - m_bbox.Max().y;
    }

    if(pos_w.z>=0)
    {
      final_pose.z =pos_w.z + cam_translate.z - m_bbox.Min().z;
    }
    else
    {
      final_pose.z =pos_w.z + cam_translate.z - m_bbox.Max().z;
    }

    return final_pose / m_bbox.Size();
  }

  inline __device__ __host__
  unsigned int ConvertLocalIndexToRealIndex(int x, int y, int z) const
  {
    // return the real index directlly if no rolling sdf applied
    if(m_local_shift.x==0 && m_local_shift.y == 0 && m_local_shift.z ==0)
    {
      const unsigned int nIndex = x + m_nGridRes_w* (y+ m_nGridRes_h* z);
      return nIndex;
    }

    // convert the local index to the real index if shift is applied
    // --- for x
    if(m_local_shift.x>0 && m_local_shift.x< static_cast<int>(m_nGridRes_w))
    {
      if( x <= static_cast<int>(m_nGridRes_w) - 1 - m_local_shift.x)
      {
        x = x + m_local_shift.x;
      }
      else
      {
        x = x - (static_cast<int>(m_nGridRes_w) - m_local_shift.x);
      }
    }
    else if(m_local_shift.x<0 && m_local_shift.x > -static_cast<int>(m_nGridRes_w))
    {
      if(x >= -m_local_shift.x)
      {
        x = x + m_local_shift.x;
      }
      else
      {
        x = x + static_cast<int>(m_nGridRes_w) + m_local_shift.x;
      }
    }
    else
    {
      if(m_local_shift.x!=0)
      {
        printf("[BoundedVolumeGrid] Fatal error! Shift x OverFlow!\n");
      }
    }

    // --- for y
    if(m_local_shift.y>0 && m_local_shift.y<=static_cast<int>(m_nGridRes_h))
    {
      if( y <= static_cast<int>(m_nGridRes_h)- 1 - m_local_shift.y)
      {
        y = y + m_local_shift.y;
      }
      else
      {
        y = y - (static_cast<int>(m_nGridRes_h) - m_local_shift.y);
      }
    }
    else if(m_local_shift.y<0 && m_local_shift.y>=-static_cast<int>(m_nGridRes_h))
    {
      if(y >= -m_local_shift.y )
      {
        y = y + m_local_shift.y;
      }
      else
      {
        y = y + static_cast<int>(m_nGridRes_h) + m_local_shift.y;
      }
    }
    else
    {
      if(m_local_shift.y!=0)
      {
        printf("[BoundedVolumeGrid] Fatal error! Shift y OverFlow!\n");
      }
    }

    // --- for z
    if(m_local_shift.z>0 && m_local_shift.z<=static_cast<int>(m_nGridRes_d) )
    {
      if(z <= static_cast<int>(m_nGridRes_d) -1 - m_local_shift.z  )
      {
        z = z + m_local_shift.z;
      }
      else
      {
        z = z - (static_cast<int>(m_nGridRes_d) - m_local_shift.z);
      }
    }
    else if(m_local_shift.z<0 && m_local_shift.z>=-static_cast<int>(m_nGridRes_d))
    {
      if(z >= -m_local_shift.z )
      {
        z = z + m_local_shift.z;
      }
      else
      {
        z = z + static_cast<int>(m_nGridRes_d) + m_local_shift.z;
      }
    }
    else
    {
      if(m_local_shift.z!=0)
      {
        printf("[BoundedVolumeGrid] Fatal error! Shift z OverFlow!\n");
      }
    }

    // compute the actual index
    const unsigned int nIndex = x + m_nGridRes_w* (y+ m_nGridRes_h* z);
    return  nIndex;
  }

  inline __host__
  void UpdateLocalAndGlobalShift(int3 cur_shift)
  {
    // ------------------------------------------------------------------------
    // updage local shift
    // ------------------------------------------------------------------------
    m_local_shift = m_local_shift + cur_shift;

    // ------------------------------------------------------------------------
    // update global shift
    // ------------------------------------------------------------------------
    // --- for x
    if(m_local_shift.x >= static_cast<int>(m_nGridRes_w))
    {
      m_local_shift.x = 0;
      m_global_shift.x++;
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] update global shift in x\n");
    }
    else if(m_local_shift.x <= -static_cast<int>(m_nGridRes_w))
    {
      m_local_shift.x = 0;
      m_global_shift.x--;
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] update global shift in x\n");
    }

    // --- for y
    if(m_local_shift.y >= static_cast<int>(m_nGridRes_h))
    {
      m_local_shift.y = 0;
      m_global_shift.y++;
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] update global shift in y\n");
    }
    else if(m_local_shift.y <= -static_cast<int>(m_nGridRes_h))
    {
      m_local_shift.y = 0;
      m_global_shift.y--;
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] update global shift in y\n");
    }

    // --- for z
    if(m_local_shift.z >= static_cast<int>(m_nGridRes_d))
    {
      m_local_shift.z = 0;
      m_global_shift.z++;
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] update global shift in z\n");
    }
    else if(m_local_shift.z <= -static_cast<int>(m_nGridRes_d))
    {
      m_local_shift.z = 0;
      m_global_shift.z--;
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] update global shift in z\n");
    }

    printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] cur shift: (%d,%d,%d);"
           "local shift: (%d,%d,%d); Global shift: (%d,%d,%d); Max local shift: (%d,%d,%d) \n",
           cur_shift.x, cur_shift.y, cur_shift.z,
           m_local_shift.x, m_local_shift.y, m_local_shift.z,
           m_global_shift.x, m_global_shift.y, m_global_shift.z,
           m_nGridRes_w, m_nGridRes_h, m_nGridRes_d);

    // ------------------------------------------------------------------------
    // check if error
    // ------------------------------------------------------------------------
    if(abs(m_local_shift.x) > m_nGridRes_w ||
       abs(m_local_shift.y) > m_nGridRes_h ||
       abs(m_local_shift.z) > m_nGridRes_d )
    {
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] fatal error! local shift overflow!\n");
      exit(-1);
    }

    if(abs(m_global_shift.x)>99999 ||
       abs(m_global_shift.y)>99999 ||
       abs(m_global_shift.z)>99999 )
    {
      printf("   [BoundedVolumeGrid/UpdateLocalAndGlobalShift] fatal error! global shift overflow!\n");
      exit(-1);
    }
  }

  inline __host__
  void UpdateLocalShiftOld(int3 shift_index)
  {
    m_local_shift = m_local_shift + shift_index;

    if(m_local_shift.x == m_nGridRes_w || m_local_shift.x == -m_nGridRes_w)
    {
      m_local_shift.x = 0;
      printf("[BoundedVolumeGrid] Set shift x back to zero! \n");
    }

    if(m_local_shift.y == m_nGridRes_h || m_local_shift.y == -m_nGridRes_h)
    {
      m_local_shift.y = 0;
      printf("[BoundedVolumeGrid] Set shift y back to zero! \n");
    }

    if(m_local_shift.z == m_nGridRes_d || m_local_shift.z == -m_nGridRes_d)
    {
      m_local_shift.z = 0;
      printf("[BoundedVolumeGrid] Set shift z back to zero! \n");
    }

    printf("[BoundedVolumeGrid] Update Shift success! current shift (%d,%d,%d);"
           " Max shift (%d,%d,%d) \n", m_local_shift.x, m_local_shift.y, m_local_shift.z,
           m_nGridRes_w, m_nGridRes_h, m_nGridRes_d);
  }

  // set next sdf that we want to init
  inline __device__
  void SetNextInitSDF(unsigned int x, unsigned int y, unsigned int z)
  {
    const int nIndex =  ConvertLocalIndexToRealIndex(x/m_nVolumeGridRes,
                                                     y/m_nVolumeGridRes,
                                                     z/m_nVolumeGridRes );

    if(m_NextInitBasicSDFs[nIndex] == 0 && CheckIfBasicSDFActive(nIndex) == true)
    {
      m_NextInitBasicSDFs[nIndex] = 1;
    }
  }

  //////////////////////////////////////////////////////
  // Access sub-regions
  //////////////////////////////////////////////////////
  inline __device__ __host__
  void SubBoundingVolume( const BoundingBox& region)
  {
    const float3 min_fv = (region.Min() - m_bbox.Min()) / (m_bbox.Size());
    const float3 max_fv = (region.Max() - m_bbox.Min()) / (m_bbox.Size());

    const int3 min_v = make_int3(
          fmaxf((m_w-1)*min_fv.x, 0),
          fmaxf((m_h-1)*min_fv.y, 0),
          fmaxf((m_d-1)*min_fv.z, 0)
          );
    const int3 max_v = make_int3(
          fminf(ceilf((m_w-1)*max_fv.x), m_w-1),
          fminf(ceilf((m_h-1)*max_fv.y), m_h-1),
          fminf(ceilf((m_d-1)*max_fv.z), m_d-1)
          );

    const int3 size_v = max((max_v - min_v) + make_int3(1,1,1), make_int3(0,0,0) );

    const BoundingBox nbbox(
          VoxelPositionInUnits(min_v),
          VoxelPositionInUnits(max_v)
          );

    printf("min_v: x%d,y%d,z%d\n",min_v.x,min_v.y,min_v.z);
    printf("size_v: x%d,y%d,z%d\n",size_v.x,size_v.y,size_v.z);

    // now get subvol for rVol
    //    BoundedVolumeGrid<T, Target, Management> rVol;
    //    rVol.init();
    //    for(int i=0;i!= m_nGridRes_w * m_nGridRes_h * m_nGridRes_d;i++)
    //    {
    //      // skip void volum grid
    //      if(rVol.CheckIfBasicSDFActive(i)== true)
    //      {
    //        if(CheckIfBasicSDFActive(i)==false)
    //        {
    //          if(InitSingleBasicSDFWithIndex(i)==false)
    //          {
    //            printf("[Kangaroo/BoundedVolumeGrid] Fatal error! cannot init grid sdf!!\n");
    //            exit(-1);
    //          }
    //        }
    //        m_GridVolumes[i].CopyFrom(rVol.m_GridVolumes[i]);
    //        GpuCheckErrors();
    //      }
    //    }

    //    BoundedVolumeGrid<T,Target,DontManage> testBB;
    //    testBB.CopyAndInitFrom();
    //    return BoundedVolumeGrid<T,Target,DontManage>();

    //    return BoundedVolumeGrid<T,Target,DontManage>(
    //          Volume<T,Target,Management>::SubVolume(min_v, size_v),
    //          nbbox
    //          );
  }

  inline __device__ __host__
  unsigned int GetTotalGridNum()
  {
    return m_nTotalGridRes;
  }

  //////////////////////////////////////////////////////
  // Global SDF (Save/Load SDF)
  //////////////////////////////////////////////////////
  // ===========================================================================
  // get bb of current global index without any shift parameters
  // ===========================================================================
  inline __host__
  BoundingBox GetDesireBB(int3 GlobalIndex)
  {
    BoundingBox mBBox = m_bbox;

    mBBox.boxmax.x = m_bbox.boxmax.x - m_bbox.Size().x * float(m_local_shift.x)/
        float(m_nGridRes_w)+ m_bbox.Size().x*(float(GlobalIndex.x-m_global_shift.x));
    mBBox.boxmax.y = m_bbox.boxmax.y - m_bbox.Size().y * float(m_local_shift.y)/
        float(m_nGridRes_h)+ m_bbox.Size().y*(float(GlobalIndex.y-m_global_shift.y));
    mBBox.boxmax.z = m_bbox.boxmax.z - m_bbox.Size().z * float(m_local_shift.z)/
        float(m_nGridRes_d)+ m_bbox.Size().z*(float(GlobalIndex.z-m_global_shift.z));

    mBBox.boxmin.x = m_bbox.boxmin.x - m_bbox.Size().x * float(m_local_shift.x)/
        float(m_nGridRes_w)+ m_bbox.Size().x*(float(GlobalIndex.x-m_global_shift.x));
    mBBox.boxmin.y = m_bbox.boxmin.y - m_bbox.Size().y * float(m_local_shift.y)/
        float(m_nGridRes_h)+ m_bbox.Size().y*(float(GlobalIndex.y-m_global_shift.y));
    mBBox.boxmin.z = m_bbox.boxmin.z - m_bbox.Size().z * float(m_local_shift.z)/
        float(m_nGridRes_d)+ m_bbox.Size().z*(float(GlobalIndex.z-m_global_shift.z));

    return mBBox;
  }

  // ===========================================================================
  // input the index of grid sdf that need to access. return its global index.
  // If the index indicates shift, its global index will++. Otherwise, its global index
  // will be the same as it was. (used when access grids as mesh)
  // ===========================================================================
  inline __host__
  int3 GetGlobalIndex(int x, int y, int z) const
  {
    if(m_local_shift.x==0 && m_local_shift.y == 0 && m_local_shift.z ==0)
    {
      return m_global_shift;
    }

    // compute the global index for the desire grid
    int3 GlobalIndex = m_global_shift;

    // for x
    if(m_local_shift.x>=0 && m_local_shift.x<=static_cast<int>(m_nGridRes_w))
    {
      if(x<m_local_shift.x)
      {
        GlobalIndex.x = m_global_shift.x+1;
      }
    }
    else if( m_local_shift.x<0 && m_local_shift.x>=-static_cast<int>(m_nGridRes_w) )
    {
      if( x<= abs(m_local_shift.x) )
      {
        GlobalIndex.x = m_global_shift.x-1;
      }
    }
    else
    {
      printf("[BoundedVolumeGrid/GetGlobalIndex] Error! Unknown local Gird index!\n");
      exit(-1);
    }

    // for y
    if(m_local_shift.y>=0 && m_local_shift.y<= static_cast<int>(m_nGridRes_h))
    {
      if(y<m_local_shift.y)
      {
        GlobalIndex.y = m_global_shift.y+1;
      }
    }
    else if(m_local_shift.y<0 && m_local_shift.y>=-static_cast<int>(m_nGridRes_h))
    {
      if( y<=abs(m_local_shift.y) )
      {
        GlobalIndex.y = m_global_shift.y-1;
      }
    }
    else
    {
      printf("[BoundedVolumeGrid/GetGlobalIndex] Error! Unknown local Gird index!\n");
      exit(-1);
    }

    // for z
    if(m_local_shift.z>=0 && m_local_shift.z<=static_cast<int>(m_nGridRes_d) )
    {
      if(z<m_local_shift.z)
      {
        GlobalIndex.z = m_global_shift.z+1;
      }
    }
    else if(m_local_shift.z<0 && m_local_shift.z>=-static_cast<int>(m_nGridRes_d))
    {
      if( z<=abs(m_local_shift.z) )
      {
        GlobalIndex.z = m_global_shift.z-1;
      }
    }
    else
    {
      printf("[BoundedVolumeGrid/GetGlobalIndex] Error! Unknown local Gird index!\n");
      exit(-1);
    }

    return  GlobalIndex;
  }

public:
  size_t        m_w;               // value usually 128, 256
  size_t        m_h;               // value usually 128, 256
  size_t        m_d;               // value usually 128, 256

  // The acculmate local shift; cur_local_shift = pre_local_shift + cur_shift
  // we can compute the real index based on the local shift, its val range: 1 ~ 8
  int3          m_local_shift;

  // global shift of the bounded box; when m_local_shift set to 0, global will ++
  int3          m_global_shift;

  BoundingBox   m_bbox;            // bounding box of bounded volume grid

  unsigned int  m_nVolumeGridRes;  // resolution the whole sdf in one dim. e.g. 256, 512;
  unsigned int  m_nGridRes_w;      // resolution of grid in x. e.g. 4, 8, 16
  unsigned int  m_nGridRes_h;      // resolution of grid in y. e.g. 4, 8, 16
  unsigned int  m_nGridRes_d;      // resolution of grid in z. e.g. 4, 8, 16

  // total num of grids we use. = m_nGridRes_w * m_nGridRes_h * m_nGridRes_d
  unsigned int  m_nTotalGridRes;

  // an array that record basic SDFs we want to init
  int           m_NextInitBasicSDFs[MAX_SUPPORT_GRID_NUM];

  // Volume that save all data; Maximum size of grid vol is MAX_SUPPORT_GRID_NUM.
  // larger size will lead to a slow profermance.
  VolumeGrid<T, Target, Manage>  m_GridVolumes[MAX_SUPPORT_GRID_NUM];
};

}





