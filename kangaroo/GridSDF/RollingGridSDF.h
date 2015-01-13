#ifndef ROLLINGGRIDSDFMesh_H
#define ROLLINGGRIDSDFMesh_H

#include <kangaroo/GridSDF/cu_sdf_reset.h>
#include <kangaroo/GridSDF/BoundedVolumeGrid.h>
#include <kangaroo/GridSDF/SavePPMGrid.h>

namespace roo {


class NextRestSDFs
{
public:
  int m_nNextResetSDFs[MAX_SUPPORT_GRID_NUM];
  int GlobalIndex_x[MAX_SUPPORT_GRID_NUM];
  int GlobalIndex_y[MAX_SUPPORT_GRID_NUM];
  int GlobalIndex_z[MAX_SUPPORT_GRID_NUM];
};

class RollingGridSDF
{
public:
  // ===========================================================================
  // update shift parameters. if true repersent update global shift
  // ===========================================================================
  template<typename T>
  inline void ApplyShiftToVolume(
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
  inline void GetGridSDFIndexNeedReset(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        CurShift)
  {
    if (CurShift.x==0 && CurShift.y==0 && CurShift.z==0)
    {
      return;
    }

    // -------------------------------------------------------------------------
    int3 LocalShift = pVol->m_local_shift + CurShift;

    printf("[GetGridSDFIndexNeedFree] Marking Grids that need to be reset.."
           "cur shift (%d,%d,%d); local shift(%d,%d,%d)\n",
           CurShift.x, CurShift.y, CurShift.z,
           LocalShift.x, LocalShift.y, LocalShift.z);

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
          int3 GlobalIndex = pVol->m_global_shift;

          // -------------------------------------------------------------------
          // cur_shift is the shift in current frame.
          // notice that the local shift is the acculmate cur shift. Which means:
          // local_shift = pre_local_shift + cur_shift
          // -------------------------------------------------------------------

          // -------------------------------------------------------------------
          //----- for x -----
          // case 1
          if(/*bReset == false &&*/ CurShift.x > 0 && LocalShift.x>0)
          {
            if(Index.x >= LocalShift.x - CurShift.x && Index.x < LocalShift.x)
            {
              bReset = true;
            }
          }
          // case 2
          else if(/*bReset == false &&*/ CurShift.x <0 && LocalShift.x<0 )
          {
            if( Index.x >= GridDim.x + LocalShift.x &&
                Index.x < GridDim.x + (LocalShift.x - CurShift.x))
            {
              bReset = true;
            }
          }
          // case 3
          else if(/*bReset == false &&*/ CurShift.x <0 && (LocalShift.x-CurShift.x)>0)
          {
            if( Index.x >= LocalShift.x && Index.x < LocalShift.x - CurShift.x )
            {
              bReset = true;
              GlobalIndex.x = pVol->m_global_shift.x + 1 /* * (1+CurShift.x/GridDim.x)*/;
              printf("special case x [1]; original global shift: (%d), new global shift (%d)\n",
                     pVol->m_global_shift.x, GlobalIndex.x );
            }
          }
          // case 4
          else if(/*bReset == false &&*/ CurShift.x >0 && (LocalShift.x-CurShift.x)<0 )
          {
            if(Index.x >= GridDim.x + (LocalShift.x - CurShift.x) &&
               Index.x < GridDim.x + LocalShift.x)
            {
              GlobalIndex.x = pVol->m_global_shift.x - 1 /** (1+CurShift.x/GridDim.x)*/;
              bReset = true;
              printf("special case x [2]; original global shift: (%d), new global shift (%d)\n",
                     pVol->m_global_shift.x, GlobalIndex.x );
            }
          }
          else
          {
            if(CurShift.x!=0)
            {
              std::cerr<<"Extra condition in x!"<<std::endl;
              exit(-1);
            }
          }


          // -------------------------------------------------------------------
          //----- for y -----
          // case 1
          if(/*bReset == false &&*/ CurShift.y >0 && LocalShift.y>0)
          {
            if(Index.y >= LocalShift.y - CurShift.y && Index.y < LocalShift.y)
            {
              bReset = true;
            }
          }
          // case 2
          else if(/*bReset == false &&*/ CurShift.y <0 && LocalShift.y<0 )
          {
            if( Index.y >= GridDim.y + LocalShift.y &&
                Index.y < GridDim.y + (LocalShift.y - CurShift.y))
            {
              bReset = true;
            }
          }
          // case 3
          else if(/*bReset == false &&*/ CurShift.y <0 && (LocalShift.y-CurShift.y)>0)
          {
            if( Index.y >= LocalShift.y && Index.y < LocalShift.y - CurShift.y )
            {
              bReset = true;
              GlobalIndex.y = pVol->m_global_shift.y + 1 /** (1+CurShift.y/GridDim.y)*/;
              printf("special case y;\n");
            }
          }
          // case 4
          else if(/*bReset == false &&*/ CurShift.y >0 && (LocalShift.y-CurShift.y)<0)
          {
            if( Index.y >= GridDim.y + (LocalShift.y - CurShift.y) &&
                Index.y < GridDim.y + LocalShift.y)
            {
              GlobalIndex.y = pVol->m_global_shift.y - 1 /** (1+CurShift.y/GridDim.y)*/;
              bReset = true;
              printf("special case y;\n");
            }
          }
          else
          {
            if(CurShift.y!=0)
            {
              std::cerr<<"Extra condition in y!"<<std::endl;
              exit(-1);
            }
          }

          // -------------------------------------------------------------------
          //----- for z -----
          // case 1
          if(/*bReset == false &&*/ CurShift.z > 0 && LocalShift.z >0 )
          {
            if(Index.z >= LocalShift.z - CurShift.z && Index.z < LocalShift.z)
            {
              bReset = true;
            }
          }
          // case 2
          else if(/*bReset == false &&*/ CurShift.z <0 && LocalShift.z< 0)
          {
            if( Index.z >= GridDim.z + LocalShift.z &&
                Index.z < GridDim.z + (LocalShift.z - CurShift.z))
            {
              bReset = true;
            }
          }
          // case 3
          else if(/*bReset == false &&*/ CurShift.z <0 && (LocalShift.z-CurShift.z)>0)
          {
            if( Index.z >= LocalShift.z && Index.z < LocalShift.z - CurShift.z)
            {
              bReset = true;
              GlobalIndex.z = pVol->m_global_shift.z + 1 /* * (1+CurShift.z/GridDim.z)*/;
              printf("special case z;\n");
            }
          }
          // case 4
          else if(/*bReset == false &&*/ CurShift.z >0 && (LocalShift.z-CurShift.z) <0)
          {
            if( Index.z >= GridDim.z + (LocalShift.z - CurShift.z) &&
                Index.z < GridDim.z + LocalShift.z)
            {
              bReset = true;
              GlobalIndex.z = pVol->m_global_shift.z - 1 /* * (1+CurShift.z/GridDim.z)*/;
              printf("special case z;\n");
            }
          }
          else
          {
            if(CurShift.z!=0)
            {
              std::cerr<<"Extra condition in z! CurShift z:"<<CurShift.z<<std::endl;
              exit(-1);
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
  inline void ResetAndFreeGird(
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
