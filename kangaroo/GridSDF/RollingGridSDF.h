#ifndef ROLLINGGRIDSDFMesh_H
#define ROLLINGGRIDSDFMesh_H

#include <kangaroo/GridSDF/cu_sdf_reset.h>
#include <kangaroo/GridSDF/BoundedVolumeGrid.h>
#include <kangaroo/GridSDF/SavePPMGrid.h>

namespace roo {

class RollingGridSDF
{
public:
  // ===========================================================================
  // update shift parameters. if true repersent update global shift
  // ===========================================================================
  template<typename T>
  void ApplyShiftToVolume(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        shift_index,
      bool                                                        bVerbose = false)
  {
    // -------------------------------------------------------------------------
    // update the bbox min and max value based on the shif parameters
    // -------------------------------------------------------------------------
    if(bVerbose)
    {
      printf("  [ApplyShiftToVolume] Shift for the current frame is (%d,%d,%d); Updating BB.\n",
             shift_index.x, shift_index.y, shift_index.z);
    }

    float3 BBSize = pVol->m_bbox.Size();

    /// -------------------------------------------------------------------------
    /// Compute the latest bounding box based on the shift parameter
    /// -------------------------------------------------------------------------
    if(shift_index.x!=0)
    {
      float new_x = static_cast<float>(shift_index.x) * BBSize.x /
          static_cast<float>(pVol->m_nGridRes_w);

      pVol->m_bbox.boxmin.x = pVol->m_bbox.boxmin.x + new_x;
      pVol->m_bbox.boxmax.x = pVol->m_bbox.boxmax.x + new_x;

      if(bVerbose)
      {
        printf("  [ApplyShiftToVolume] shift x:%d (index), %f(m); bbmin x->%f, bbmax x->%f\n",
               shift_index.x, new_x, pVol->m_bbox.boxmin.x, pVol->m_bbox.boxmax.x);
      }
    }

    if(shift_index.y!=0)
    {
      float new_y = static_cast<float>(shift_index.y) * BBSize.y /
          static_cast<float>(pVol->m_nGridRes_h);

      pVol->m_bbox.boxmin.y = pVol->m_bbox.boxmin.y + new_y;
      pVol->m_bbox.boxmax.y = pVol->m_bbox.boxmax.y + new_y;

      if(bVerbose)
      {
        printf("  [ApplyShiftToVolume] shift y:%d(index), %f(m); bbmin y->%f, bbmax y->%f\n",
               shift_index.y, new_y, pVol->m_bbox.boxmin.y, pVol->m_bbox.boxmax.y);
      }
    }

    if(shift_index.z!=0)
    {
      float new_z = static_cast<float>(shift_index.z) * BBSize.z /
          static_cast<float>(pVol->m_nGridRes_d);

      pVol->m_bbox.boxmin.z = pVol->m_bbox.boxmin.z + new_z;
      pVol->m_bbox.boxmax.z = pVol->m_bbox.boxmax.z + new_z;

      if(bVerbose)
      {
        printf("  [ApplyShiftToVolume] shift z:%d(index), %f(m); bbmin z->%f, bbmax z->%f\n",
               shift_index.z, new_z, pVol->m_bbox.boxmin.z, pVol->m_bbox.boxmax.z);
      }
    }

    // -------------------------------------------------------------------------
    // update local and global shift in the bounded volume
    // -------------------------------------------------------------------------
    if(shift_index.x!=0 || shift_index.y!= 0 || shift_index.z!=0)
    {
      pVol->UpdateLocalAndGlobalShift(shift_index);
    }
  }

  // ===========================================================================
  // Get index of grid that need to be freed. Only free grid that just "shift"
  // shift value from 1 ~ 8
  // ===========================================================================
  template<typename T>
  void GetGridSDFIndexNeedReset(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        CurShift)
  {
    if (CurShift.x==0 && CurShift.y==0 && CurShift.z==0)
    {
      return;
    }

    // -------------------------------------------------------------------------
    int3 PreLocalShift = pVol->m_local_shift;
    int3 CurLocalShift = pVol->m_local_shift + CurShift;

    printf("[GetGridSDFIndexNeedFree] Marking Grids that need to be reset.."
           "cur shift (%d,%d,%d); local shift(%d,%d,%d)\n",
           CurShift.x, CurShift.y, CurShift.z,
           CurLocalShift.x, CurLocalShift.y, CurLocalShift.z);

    int nReqResetNum = 0;
    int nActualResetNum  = 0;

    // real index of a grid, from 0 ~ 7
    int3 Index;

    // dimision of a grid, usually 8, 16 etc
    int3 GridDim = make_int3(static_cast<int>(pVol->m_nGridRes_w),
                             static_cast<int>(pVol->m_nGridRes_h),
                             static_cast<int>(pVol->m_nGridRes_d));

    // for each grid sdf in the volume
    for(Index.x = 0; Index.x!= GridDim.x; Index.x++)
    {
      for(Index.y = 0; Index.y!= GridDim.y; Index.y++)
      {
        for(Index.z = 0; Index.z!= GridDim.z; Index.z++)
        {
          bool bReset = false;
          bool bCheckIfUpdateGlobalIndex_x = false;
          bool bCheckIfUpdateGlobalIndex_y = false;
          bool bCheckIfUpdateGlobalIndex_z = false;

          int3 GlobalIndex = pVol->m_global_shift;

          // -------------------------------------------------------------------
          // cur_shift is the shift in current frame.
          // notice that the local shift is the acculmate cur shift. Which means:
          // local_shift = pre_local_shift + cur_shift
          // -------------------------------------------------------------------

          // -------------------------------------------------------------------
          //----- for x -----
          // case 1
          if( CurShift.x >0 && PreLocalShift.x >=0)
          {
            if(Index.x >= PreLocalShift.x && Index.x < CurLocalShift.x)
            {
              bReset = true;
              bCheckIfUpdateGlobalIndex_x = true;
            }
          }
          // case 2
          else if( CurShift.x <0 && PreLocalShift.x <=0 )
          {
            if( Index.x >= GridDim.x + CurLocalShift.x &&
                Index.x < GridDim.x + PreLocalShift.x)
            {
              bReset = true;
              bCheckIfUpdateGlobalIndex_x = true;
            }
          }
          // case 3
          else if( CurShift.x <0 && PreLocalShift.x >0)
          {
            if( Index.x >= CurLocalShift.x && Index.x < PreLocalShift.x )
            {
              bReset = true;
              GlobalIndex.x = pVol->m_global_shift.x + 1 * (1+CurShift.x/GridDim.x);
              bCheckIfUpdateGlobalIndex_x = true;
            }
          }
          // case 4
          else if( CurShift.x >0 && PreLocalShift.x <0 )
          {
            if(Index.x >= GridDim.x + PreLocalShift.x &&
               Index.x < GridDim.x + CurLocalShift.x)
            {
              bReset = true;
              GlobalIndex.x = pVol->m_global_shift.x - 1 * (1+CurShift.x/GridDim.x);
              bCheckIfUpdateGlobalIndex_x = true;
            }
          }
          else
          {
            if(CurShift.x!=0)
            {
              std::cerr<<"[GetGridSDFIndexNeedFree] unknown condition in x direction!"<<std::endl;
              exit(-1);
            }
          }


          // -------------------------------------------------------------------
          //----- for y -----
          // case 1
          if( CurShift.y >0 && PreLocalShift.y >=0)
          {
            if(Index.y >= PreLocalShift.y && Index.y < CurLocalShift.y)
            {
              bReset = true;
              bCheckIfUpdateGlobalIndex_y = true;
            }
          }
          // case 2
          else if( CurShift.y <0 && PreLocalShift.y <=0 )
          {
            if( Index.y >= GridDim.y + CurLocalShift.y &&
                Index.y < GridDim.y + PreLocalShift.y)
            {
              bReset = true;
              bCheckIfUpdateGlobalIndex_y = true;
            }
          }
          // case 3
          else if( CurShift.y <0 && PreLocalShift.y >0)
          {
            if( Index.y >= CurLocalShift.y && Index.y < PreLocalShift.y )
            {
              bReset = true;
              GlobalIndex.y = pVol->m_global_shift.y + 1 * (1+CurShift.y/GridDim.y);
              bCheckIfUpdateGlobalIndex_y = true;
            }
          }
          // case 4
          else if( CurShift.y >0 && PreLocalShift.y <0)
          {
            if( Index.y >= GridDim.y + PreLocalShift.y &&
                Index.y < GridDim.y + CurLocalShift.y)
            {
              bReset = true;
              GlobalIndex.y = pVol->m_global_shift.y - 1 * (1+CurShift.y/GridDim.y);
              bCheckIfUpdateGlobalIndex_y = true;
            }
          }
          else
          {
            if(CurShift.y!=0)
            {
              std::cerr<<"[GetGridSDFIndexNeedFree] unknown condition in y direction!"<<std::endl;
              exit(-1);
            }
          }

          // -------------------------------------------------------------------
          //----- for z -----
          // case 1
          if( CurShift.z >0 && PreLocalShift.z >=0 )
          {
            if(Index.z >= PreLocalShift.z && Index.z < CurLocalShift.z)
            {
              bReset = true;
              bCheckIfUpdateGlobalIndex_z = true;
            }
          }
          // case 2
          else if( CurShift.z <0 && PreLocalShift.z <=0)
          {
            if( Index.z >= GridDim.z + CurLocalShift.z &&
                Index.z < GridDim.z + PreLocalShift.z )
            {
              bReset = true;
              bCheckIfUpdateGlobalIndex_z = true;
            }
          }
          // case 3
          else if( CurShift.z <0 && PreLocalShift.z >0)
          {
            if( Index.z >= CurLocalShift.z && Index.z < PreLocalShift.z)
            {
              bReset = true;
              GlobalIndex.z = pVol->m_global_shift.z + 1 * (1+CurShift.z/GridDim.z);
              bCheckIfUpdateGlobalIndex_z = true;
            }
          }
          // case 4
          else if( CurShift.z >0 && PreLocalShift.z <0)
          {
            if( Index.z >= GridDim.z + PreLocalShift.z &&
                Index.z < GridDim.z + CurLocalShift.z )
            {
              bReset = true;
              GlobalIndex.z = pVol->m_global_shift.z - 1 * (1+CurShift.z/GridDim.z);
              bCheckIfUpdateGlobalIndex_z = true;
            }
          }
          else
          {
            if(CurShift.z!=0)
            {
              std::cerr<<"[GetGridSDFIndexNeedFree] unknown condition in z direction!"<<std::endl;
              exit(-1);
            }
          }

          // -------------------------------------------------------------------
          // check if we need to update the global index
          if(bCheckIfUpdateGlobalIndex_x == false && CurLocalShift.x!=0)
          {
             if(CurLocalShift.x>0 && Index.x<=CurLocalShift.x-1)
             {
               GlobalIndex.x = pVol->m_global_shift.x +1;
             }
             else if(CurLocalShift.x <0 && Index.x >= CurLocalShift.x+GridDim.x)
             {
               GlobalIndex.x = pVol->m_global_shift.x -1;
             }
          }

          if(bCheckIfUpdateGlobalIndex_y == false && CurLocalShift.y!=0)
          {
             if(CurLocalShift.y>0 && Index.y<=CurLocalShift.y-1)
             {
               GlobalIndex.y = pVol->m_global_shift.y +1;
             }
             else if(CurLocalShift.y <0 && Index.y >= CurLocalShift.y+GridDim.y)
             {
               GlobalIndex.y = pVol->m_global_shift.y -1;
             }
          }

          if(bCheckIfUpdateGlobalIndex_z == false && CurLocalShift.z!=0)
          {
             if(CurLocalShift.z>0 && Index.z<=CurLocalShift.z-1)
             {
               GlobalIndex.z = pVol->m_global_shift.z +1;
             }
             else if(CurLocalShift.z <0 && Index.z >= CurLocalShift.z+GridDim.z)
             {
               GlobalIndex.z = pVol->m_global_shift.z -1;
             }
          }

          // -------------------------------------------------------------------
          // ---------- set flag for grid that need to be freed ----------------
          // -------------------------------------------------------------------
          // get the index of the voxel, notice that we do not need to apply any
          // shift here
          int nGridIndex =
              Index.x + pVol->m_nGridRes_w * (Index.y + pVol->m_nGridRes_h * Index.z);

          m_nGlobalIndex_x[nGridIndex] = GlobalIndex.x;
          m_nGlobalIndex_y[nGridIndex] = GlobalIndex.y;
          m_nGlobalIndex_z[nGridIndex] = GlobalIndex.z;

          if(bReset)
          {
            nReqResetNum ++;
            m_nNextResetSDFs[nGridIndex] = 1;

            if(pVol->CheckIfBasicSDFActive(nGridIndex))
            {
              nActualResetNum ++ ;
            }
          }
          else
          {
            m_nNextResetSDFs[nGridIndex] = 0;
          }
        }
      }
    }

    std::cout<<"  [GetGridSDFIndexNeedFree] Finished. Req Reset "<< nReqResetNum<<
               " grid; Actual reset (active grids)"<< nActualResetNum<<std::endl;
  }


  // ===========================================================================
  template<typename T>
  void ResetAndFreeGird(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol)
  {
    for(unsigned int i=0; i!=pVol->GetTotalGridNum(); i++)
    {
      if(m_nNextResetSDFs[i] == 1 && pVol->CheckIfBasicSDFActive(i))
      {
        roo::SdfReset(pVol->m_GridVolumes[i]);
        pVol->FreeMemoryByIndex(i);
        pVol->m_GridVolumes[i].d = 0;
        pVol->m_GridVolumes[i].w = 0;
        pVol->m_GridVolumes[i].h = 0;
      }
    }
  }

public:
  int m_nNextResetSDFs[MAX_SUPPORT_GRID_NUM];
  int m_nGlobalIndex_x[MAX_SUPPORT_GRID_NUM];
  int m_nGlobalIndex_y[MAX_SUPPORT_GRID_NUM];
  int m_nGlobalIndex_z[MAX_SUPPORT_GRID_NUM];
};

}


#endif // ROLLINGGRIDSDF_H
