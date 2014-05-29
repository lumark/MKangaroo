#pragma once

#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "Sdf.h"
#include "launch_utils.h"

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
    //    if(n_w == n_h && n_h == n_d)
    //    {
    m_w              = n_w;
    m_h              = n_h;
    m_d              = n_d;

    m_bbox           = r_bbox;
    m_nVolumeGridRes = n_res;

    m_nWholeGridRes_w  = m_w/m_nVolumeGridRes;
    m_nWholeGridRes_h  = m_h/m_nVolumeGridRes;
    m_nWholeGridRes_d  = m_d/m_nVolumeGridRes;


    if(m_nWholeGridRes_w * m_nWholeGridRes_h * m_nWholeGridRes_d > MAX_SUPPORT_GRID_NUM)
    {
      printf("[BoundedVolumeGrid/init] fatal error, overflow! Max allow 4096, request %d .\n",
             m_nWholeGridRes_w * m_nWholeGridRes_h * m_nWholeGridRes_d);
      printf("Please reset VOL_RES and VOL_GRID_RES parameters!!!");
      exit(-1);
    }

    ResetAllGridVol();
    m_local_shift = make_int3(0,0,0);
    m_global_shift = make_int3(0,0,0);
    //    }
    //    else
    //    {
    //      printf("[BoundedVolumeGrid/init] Fatal error! Only support cube SDF with (n*n*n) size!\n");
    //      exit(-1);
    //    }
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
  void InitSingleBasicSDFWithGridIndex(unsigned int x, unsigned int y, unsigned int z)
  {
    int nIndex =ConvertLocalIndexToRealIndex( int(floorf(x/m_nVolumeGridRes)),
                               int(floorf(y/m_nVolumeGridRes)),
                               int(floorf(z/m_nVolumeGridRes)) );

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
      GpuCheckErrors();

      return true;
    }
    return false;
  }


  //////////////////////////////////////////////////////
  // Memory copy and free
  //////////////////////////////////////////////////////
  inline __host__
  void CopyFrom(BoundedVolumeGrid<T, TargetDevice, Management>& rVol )
  {
    for(int i=0;i!= m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      m_GridVolumes[i].CopyFrom(rVol.m_GridVolumes[i]);
    }
  }

  inline __host__
  void CopyFrom(BoundedVolumeGrid<T, TargetHost, Management>& rVol )
  {
    for(int i=0;i!= m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      m_GridVolumes[i].CopyFrom(rVol.m_GridVolumes[i]);
    }
  }

  inline __host__
  void CopyAndInitFrom(BoundedVolumeGrid<T, TargetDevice , Management>& rVol )
  {
    for(int i=0;i!= m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      // skip void volum grid, only copy non-void grid
      if(rVol.CheckIfBasicSDFActive(i)== true)
      {
        if(CheckIfBasicSDFActive(i)==false)
        {
          if(InitSingleBasicSDFWithIndex(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Fatal error! cannot init grid sdf!!\n");
            exit(-1);
          }

          if(CheckIfBasicSDFActive(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Fatal error! Init grid sdf fail!!\n");
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
    for(int i=0;i!= m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      // skip void volum grid
      if(rHVol.CheckIfBasicSDFActive(i)== true)
      {
        if(CheckIfBasicSDFActive(i)==false)
        {
          if(InitSingleBasicSDFWithIndex(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Fatal error! cannot init grid sdf!!\n");
            exit(-1);
          }

          if(CheckIfBasicSDFActive(i)==false)
          {
            printf("[Kangaroo/BoundedVolumeGrid] Fatal error! Init grid sdf fail!!\n");
            exit(-1);
          }
        }

        m_GridVolumes[i].CopyFrom(rHVol.m_GridVolumes[i]);
        GpuCheckErrors();

        printf("copy success.\n");
      }
    }
  }

  inline __host__
  void FreeMemory()
  {
    for(int i=0;i!= m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
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
      printf("[BoundedVolumeGrid] fatal error! Single GridSDF being free must be allocate first!!\n");
      exit(-1);
    }

    cudaFree( m_GridVolumes[nIndex].ptr );
    GpuCheckErrors();
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
    return m_bbox.Size() / make_float3( (max_global.x-min_global.x)*(m_w-1),
                                        (max_global.y-min_global.y)*(m_h-1),
                                        (max_global.z-min_global.z)*(m_d-1) );
  }

  //////////////////////////////////////////////////////
  // Return true if this BoundedVolumeGrid represents a positive
  // amount of space.
  //////////////////////////////////////////////////////

  inline __host__
  bool IsValid() const {

    int nNum = 0;

    for(int i=0;i!=m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      if(m_GridVolumes[i].d !=m_nVolumeGridRes &&
         m_GridVolumes[i].h !=m_nVolumeGridRes &&
         m_GridVolumes[i].w !=m_nVolumeGridRes)
      {
        nNum ++;
      }
    }

    return nNum>0 && m_w >= 2 && m_h >= 2 && m_d >= 2;
  }


  //////////////////////////////////////////////////////
  // Access Elements
  //////////////////////////////////////////////////////

  inline __device__ __host__
  uint3 Voxels() const
  {
    return make_uint3(m_w,m_h,m_d);
  }

  inline __host__ __device__
  bool CheckIfVoxelExist(int x, int y, int z)
  {
    int nIndex = ConvertLocalIndexToRealIndex( int(floorf(x/m_nVolumeGridRes)),
                                int(floorf(y/m_nVolumeGridRes)),
                                int(floorf(z/m_nVolumeGridRes)) );

    if( CheckIfBasicSDFActive(nIndex) == true)
    {
      return true;
    }

    return false;
  }

  inline  __device__
  T& operator()(unsigned int x,unsigned int y, unsigned int z)
  {
    int nIndex = ConvertLocalIndexToRealIndex( int(floorf(x/m_nVolumeGridRes)),
                                int(floorf(y/m_nVolumeGridRes)),
                                int(floorf(z/m_nVolumeGridRes)) );

    if(CheckIfBasicSDFActive(nIndex) == false)
    {
      printf("[Kangaroo/BoundedVolumeGrid] Fatal Error!!!!! basicSDF doesn't exist."
             "shift (x,y,z)=(%d,%d,%d); index x=%d,y=%d,z=%d; Max index (x,y,z)=(%d,%d,%d)\n",
             m_local_shift.x,
             m_local_shift.y,
             m_local_shift.z,
             int(floorf(x/m_nVolumeGridRes)),
             int(floorf(y/m_nVolumeGridRes)),
             int(floorf(z/m_nVolumeGridRes)),
             m_nWholeGridRes_w-1,
             m_nWholeGridRes_h-1,
             m_nWholeGridRes_d-1);
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

    int nIndex = ConvertLocalIndexToRealIndex( Index.x, Index.y, Index.z);

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

  // actual position in current bounding box
  inline __device__ __host__
  float3 VoxelPositionInUnits(int x, int y, int z) const
  {
    const float3 vol_size = m_bbox.Size();

    return make_float3(
          m_bbox.Min().x + vol_size.x * (float)x/(float)(m_w-1),
          m_bbox.Min().y + vol_size.y * (float)y/(float)(m_h-1),
          m_bbox.Min().z + vol_size.z * (float)z/(float)(m_d-1)
          );
  }

  inline __device__ __host__
  float3 VoxelPositionInUnits(int3 p_v) const
  {
    return VoxelPositionInUnits(p_v.x,p_v.y,p_v.z);
  }

  inline __device__ __host__
  float3 VoxelPositionInUnitsGlobal(int x, int y, int z,
                                    int3 cur_global, int3 max_global, int3 min_global) const
  {
    if(x>=m_w || y>= m_h || z>=m_d)
    {
      printf("[VoxelPositionInUnitsGlobal] fatal error! index overflow! (%d,%d,%d), dim: (%d,%d,%d)\n",
             x,y,z,int(m_w),int(m_h),int(m_d));
    }

    const float3 vol_size = m_bbox.Size();

    float3 local_pos =  make_float3(
          m_bbox.Min().x + vol_size.x * (float)x/(float)(m_w-1),
          m_bbox.Min().y + vol_size.y * (float)y/(float)(m_h-1),
          m_bbox.Min().z + vol_size.z * (float)z/(float)(m_d-1)
          );

    float global_pos_x = (float(cur_global.x-min_global.x) + local_pos.x)/float(max_global.x-min_global.x);
    float global_pos_y = (float(cur_global.y-min_global.y) + local_pos.y)/float(max_global.y-min_global.y);
    float global_pos_z = (float(cur_global.z-min_global.z) + local_pos.z)/float(max_global.z-min_global.z);

    float3 global_pos = make_float3(global_pos_x, global_pos_y, global_pos_z);

    printf("LocalPos:(%f,%f,%f) GlobalPos(%f,%f,%f)",
           local_pos.x,local_pos.y,local_pos.z,
           global_pos.x,global_pos.y,global_pos.z);

    return global_pos;
  }

  // ===========================================================================
  // get sub bounding volume by bounding box for speed.
  inline __device__ __host__
  BoundedVolumeGrid<T,Target,Management> SubBoundingVolume(const BoundingBox& region)
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

    //    printf("New SubVol Size is %d,%d,%d\n",size_v.x,size_v.y,size_v.z );

    // return a new BoundedVolumeGrid
    BoundedVolumeGrid<T,Target,Management> SubBoundedVolumeGrid;
    SubBoundedVolumeGrid.init(size_v.x,size_v.y,size_v.z, m_nVolumeGridRes, nbbox);

    // put shift parameters
    m_subVol_shift = make_int3(min_v.x * m_w, min_v.y*m_h, min_v.z * m_d);

    // change point of it
    for(int i=0;i!=m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      SubBoundedVolumeGrid.m_GridVolumes[i] = m_GridVolumes[i];
    }

    return SubBoundedVolumeGrid;
  }


  // ======================== Index/Shift ====================================
  inline __device__
  float3 GetShiftValue(float3 pos_w, float3 cam_pose) const
  {
    /// get pose of voxel in whole sdf, in %
    // change pos_w to global pose
    pos_w = pos_w+cam_pose;

    return (pos_w - m_bbox.Min()) / (m_bbox.Size());
  }


  // ===========================================================================
  // given desire grid index (x,y,z), get real index in Volume when shift is applied
  // ===========================================================================
  inline __device__ __host__
  unsigned int ConvertLocalIndexToRealIndex(int x, int y, int z) const
  {
    if(m_local_shift.x==0 && m_local_shift.y == 0 && m_local_shift.z ==0)
    {
      const unsigned int nIndex =x + m_nWholeGridRes_w* (y+ m_nWholeGridRes_h* z);
      return nIndex;
    }

    // for x
    if(m_local_shift.x>0 && m_local_shift.x<=int(m_nWholeGridRes_w))
    {
      if( x<=int(m_nWholeGridRes_w)-1-m_local_shift.x)
      {
        x = x+m_local_shift.x;
      }
      else
      {
        x = x-( int(m_nWholeGridRes_w) - m_local_shift.x );
      }
    }
    else if(m_local_shift.x<0 && m_local_shift.x>=-int(m_nWholeGridRes_w))
    {
      if(x>=abs(m_local_shift.x) && x<=int(m_nWholeGridRes_w))
      {
        x = x+m_local_shift.x;
      }
      else if( x<=abs(m_local_shift.x) )
      {
        x = x+int(m_nWholeGridRes_w)-1-abs(m_local_shift.x) ;
      }
    }


    // for y
    if(m_local_shift.y>0 && m_local_shift.y<=int(m_nWholeGridRes_h))
    {
      if( y<=int(m_nWholeGridRes_h)-1-m_local_shift.y)
      {
        y = y+m_local_shift.y;
      }
      else
      {
        y = y- ( int(m_nWholeGridRes_h) - m_local_shift.y );
      }
    }
    else if(m_local_shift.y<0 && m_local_shift.y>=-int(m_nWholeGridRes_h))
    {
      if(y>=abs(m_local_shift.y) && y<=int(m_nWholeGridRes_h))
      {
        y = y+m_local_shift.y;
      }
      else if( y<=abs(m_local_shift.y) )
      {
        y = y+int(m_nWholeGridRes_h)-1-abs(m_local_shift.y) ;
      }
    }

    // for z
    if(m_local_shift.z>0 && m_local_shift.z<=int(m_nWholeGridRes_d) )
    {
      if(z <= int(m_nWholeGridRes_d) -1 - m_local_shift.z  )
      {
        z = z + m_local_shift.z;
      }
      else
      {
        z = z - ( int(m_nWholeGridRes_d) - m_local_shift.z );
      }
    }
    else if(m_local_shift.z<0 && m_local_shift.z>=-int(m_nWholeGridRes_d))
    {
      if(z>=abs(m_local_shift.z) && z<=int(m_nWholeGridRes_d))
      {
        z = z+m_local_shift.z;
      }
      else if( z<=abs(m_local_shift.z) )
      {
        z = z+int(m_nWholeGridRes_d)-1-abs(m_local_shift.z) ;
      }
    }


    // compute actual index
    const unsigned int nIndex =x + m_nWholeGridRes_w* (y+ m_nWholeGridRes_h* z);
    return  nIndex;
  }


  // ===========================================================================
  // input the index of grid sdf that we want to access. return the global index for it.
  // If the index is shift, its global index will ++. Otherwise, the global index
  // will be the same as it was. The role of this function is to see if the m_global_shift
  // does not reset yet but there is a current shift for the grid.
  // ===========================================================================
  inline __device__ __host__
  int3 GetGlobalIndex(int x, int y, int z) const
  {
    if(m_local_shift.x==0 && m_local_shift.y == 0 && m_local_shift.z ==0)
    {
      return m_global_shift;
    }

    // compute global index for single grid
    int3 GlobalIndex = m_global_shift;

    // for x
    if(m_local_shift.x>0 && m_local_shift.x<=int(m_nWholeGridRes_w))
    {
      if(x>=int(m_nWholeGridRes_w)-m_local_shift.x)
      {
        GlobalIndex.x = m_global_shift.x+1;
      }
    }
    else if( m_local_shift.x<0 && m_local_shift.x>=-int(m_nWholeGridRes_w) )
    {
      if( x<=abs(m_local_shift.x) )
      {
        GlobalIndex.x = m_global_shift.x-1;
      }
    }

    // for y
    if(m_local_shift.y>0 && m_local_shift.y<= int(m_nWholeGridRes_h))
    {
      if(y>=int(m_nWholeGridRes_h)-m_local_shift.y)
      {
        GlobalIndex.y = m_global_shift.y+1;
      }
    }
    else if(m_local_shift.y<0 && m_local_shift.y>=-int(m_nWholeGridRes_h))
    {
      if( y<=abs(m_local_shift.y) )
      {
        GlobalIndex.y = m_global_shift.y-1;
      }
    }

    // for z
    if(m_local_shift.z>0 && m_local_shift.z<=int(m_nWholeGridRes_d) )
    {
      if(z>=int(m_nWholeGridRes_d)-m_local_shift.z)
      {
        GlobalIndex.z = m_global_shift.z+1;
      }
    }
    else if(m_local_shift.z<0 && m_local_shift.z>=-int(m_nWholeGridRes_d))
    {
      if( z<=abs(m_local_shift.z) )
      {
        GlobalIndex.z = m_global_shift.z-1;
      }
    }

    // compute actual index
    return  GlobalIndex;
  }


  // ===========================================================================
  // in some cases a grid sdf in x, y, z position is actually sdf in x1,y1,z1 with
  // global pose gx,gy,gz. this function return x1,y1,z1
  // ===========================================================================
  inline __host__
  int3 ConvertRealIndexToLocalIndex(int x, int y, int z) const
  {
    // compute local index for single grid
    int3 LocalIndex = make_int3(x,y,z);

    if(m_local_shift.x==0 && m_local_shift.y == 0 && m_local_shift.z ==0)
    {
      return LocalIndex;
    }


//    // x
//    if(m_local_shift.x>0 && m_local_shift.x<=int(m_nWholeGridRes_w))
//    {
//      if( LocalIndex.x< m_local_shift.x)
//      {
//        LocalIndex.x = LocalIndex.x + int(m_nWholeGridRes_w) - m_local_shift.x;
//      }
//      else
//      {
//        LocalIndex.x = LocalIndex.x- m_local_shift.x ;
//      }
//    }

//    // y
//    if(m_local_shift.y>0 && m_local_shift.y<=int(m_nWholeGridRes_h))
//    {
//      if( LocalIndex.y< m_local_shift.y)
//      {
//        LocalIndex.y = LocalIndex.y + int(m_nWholeGridRes_h) - m_local_shift.y;
//      }
//      else
//      {
//        LocalIndex.y = LocalIndex.y- m_local_shift.y ;
//      }
//    }

//    // z
//    if(m_local_shift.z>0 && m_local_shift.z<=int(m_nWholeGridRes_d))
//    {
//      if( LocalIndex.z< m_local_shift.z)
//      {
//        LocalIndex.z = LocalIndex.z + int(m_nWholeGridRes_d) - m_local_shift.z;
//      }
//      else
//      {
//        LocalIndex.z = LocalIndex.z - m_local_shift.z ;
//      }
//    }



    return LocalIndex;
  }



  // ===========================================================================
  // check if need to reset local shift and set global shift
  inline __host__
  void ResetShift(int3 shift_index)
  {
    m_local_shift = m_local_shift + shift_index;

    // for x
    if(m_local_shift.x >= int(m_nWholeGridRes_w)+1)
    {
      m_local_shift.x = m_local_shift.x-int(m_nWholeGridRes_w);
      m_global_shift.x++;
      printf("[BoundedVolumeGrid] Set local shift x back to zero! \n");
    }

    if(m_local_shift.x <= -int(m_nWholeGridRes_w+1))
    {
      m_local_shift.x = m_local_shift.x-(-int(m_nWholeGridRes_w));
      m_global_shift.x--;
      printf("[BoundedVolumeGrid] Set local shift x back to zero! \n");
    }



    // for y
    if(m_local_shift.y >= int(m_nWholeGridRes_h)+1)
    {
      m_local_shift.y = m_local_shift.y - int(m_nWholeGridRes_h);
      m_global_shift.y++;
      printf("[BoundedVolumeGrid] Set local shift y back to zero! \n");
    }

    if(m_local_shift.y <= -int(m_nWholeGridRes_h+1))
    {
      m_local_shift.y = m_local_shift.y-(-int(m_nWholeGridRes_h));
      m_global_shift.y--;
      printf("[BoundedVolumeGrid] Set local shift y back to zero! \n");
    }



    // for z
    if(m_local_shift.z >= int(m_nWholeGridRes_d)+1)
    {
      m_local_shift.z = m_local_shift.z-int(m_nWholeGridRes_d);
      m_global_shift.z++;
      printf("[BoundedVolumeGrid] Set local shift z back to zero! \n");
    }

    if(m_local_shift.z <= -int(m_nWholeGridRes_d+1))
    {
      m_local_shift.z = m_local_shift.z-(-int(m_nWholeGridRes_d));
      m_global_shift.z--;
      printf("[BoundedVolumeGrid] Set local shift z back to zero! \n");
    }

    printf("[BoundedVolumeGrid] Update Shift success! local shift: x=%d,y=%d,z=%d;"
           "Global shift: x=%d,y=%d,z=%d; Max shift (%d,%d,%d) \n",
           m_local_shift.x,m_local_shift.y,m_local_shift.z,
           m_global_shift.x,m_global_shift.y,m_global_shift.z,
           m_nWholeGridRes_w, m_nWholeGridRes_h,m_nWholeGridRes_d);
  }



  // =============================== Others ===================================
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

    for(int i=0;i!=m_nWholeGridRes_w*m_nWholeGridRes_h*m_nWholeGridRes_d;i++)
    {
      if(CheckIfBasicSDFActive(i)==true)
      {
        nNum ++;
      }
    }

    return nNum;
  }


public:
  size_t                                      m_d;
  size_t                                      m_w;
  size_t                                      m_h;

  // for rolling sdf. When this value is not zero, we need to recompute index based on the shift
  int3                                        m_local_shift;
  int3                                        m_global_shift; // when m_shift set to 0, global will ++
  int3                                        m_subVol_shift;  // shift for sub bounded volume.

  // bounding box ofbounded volume grid
  BoundingBox                                 m_bbox;

  unsigned int                                m_nVolumeGridRes;         // resolution of a single grid in one dim.
  unsigned int                                m_nWholeGridRes_w;        // resolution of a whole grid in one dim. usually 4, 8, 16
  unsigned int                                m_nWholeGridRes_h;        // resolution of a whole grid in one dim. usually 4, 8, 16
  unsigned int                                m_nWholeGridRes_d;        // resolution of a whole grid in one dim. usually 4, 8, 16

  // volume that save all data
  // maximum allow size of grid vol is 4096. larger than this size will lead to a very slow profermance.
  VolumeGrid<T, Target, Manage>               m_GridVolumes[MAX_SUPPORT_GRID_NUM];
  int                                         m_NextInitBasicSDFs[MAX_SUPPORT_GRID_NUM];  // an array that record basic SDFs we want to init
};


}
