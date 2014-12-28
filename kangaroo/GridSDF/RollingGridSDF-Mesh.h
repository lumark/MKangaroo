#ifndef ROLLINGGRIDSDFMesh_H
#define ROLLINGGRIDSDFMesh_H

#include <kangaroo/cu_sdffusion.h>
#include <kangaroo/GridSDF/BoundedVolumeGrid.h>
#include <kangaroo/GridSDF/SavePPMGrid.h>

namespace roo {

class NextResetSDF
{
public:
  int  m_nNextResetSDFs[MAX_SUPPORT_GRID_NUM];
  int  m_x[MAX_SUPPORT_GRID_NUM];
  int  m_y[MAX_SUPPORT_GRID_NUM];
  int  m_z[MAX_SUPPORT_GRID_NUM];
};

class RollingGridSDFMesh
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
    // change bbox min and max value based on the shif parameters
    // -------------------------------------------------------------------------
    if(bVerbose)
    {
      printf("[UpdateShift] new shift for current frame is x=%d,y=%d,z=%d; Updating BB.\n",
             shift_index.x, shift_index.y, shift_index.z);
    }

    float3 BBSize = pVol->m_bbox.Size();

    // -------------------------------------------------------------------------
    // Compute the latest bounding box
    // -------------------------------------------------------------------------
    if(shift_index.x!=0)
    {
      float fx = static_cast<float>(shift_index.x) * BBSize.x/static_cast<float>(pVol->m_nGridRes_w);
      pVol->m_bbox.boxmin.x = pVol->m_bbox.boxmin.x + fx;
      pVol->m_bbox.boxmax.x = pVol->m_bbox.boxmax.x + fx;

      if(bVerbose)
      {
        printf("[ApplyShiftToVolume] shift x:%d (index), %f(m), bbmin x->%f, bbmax x->%f\n",
               shift_index.x, fx, pVol->m_bbox.boxmin.x, pVol->m_bbox.boxmax.x);
      }
    }

    if(shift_index.y!=0)
    {
      float fy = static_cast<float>(shift_index.y) * BBSize.y/static_cast<float>(pVol->m_nGridRes_h);
      pVol->m_bbox.boxmin.y = pVol->m_bbox.boxmin.y + fy;
      pVol->m_bbox.boxmax.y = pVol->m_bbox.boxmax.y + fy;

      if(bVerbose)
      {
        printf("[ApplyShiftToVolume] shift y:%d(index), %f(m), bbmin y->%f, bbmax y->%f\n",
               shift_index.y, fy, pVol->m_bbox.boxmin.y, pVol->m_bbox.boxmax.y);
      }
    }

    if(shift_index.z!=0)
    {
      float fz = static_cast<float>(shift_index.z) * BBSize.z/static_cast<float>(pVol->m_nGridRes_d);
      pVol->m_bbox.boxmin.z = pVol->m_bbox.boxmin.z + fz;
      pVol->m_bbox.boxmax.z = pVol->m_bbox.boxmax.z + fz;

      if(bVerbose)
      {
        printf("[ApplyShiftToVolume] shift z:%d(index), %f(m), bbmin z->%f, bbmax z->%f\n",
               shift_index.z, fz, pVol->m_bbox.boxmin.z, pVol->m_bbox.boxmax.z);
      }
    }

    // -------------------------------------------------------------------------
    // reset local and global shift
    // -------------------------------------------------------------------------
    if(shift_index.x!=0 || shift_index.y!= 0 || shift_index.z!=0)
    {
      pVol->UpdateLocalAndGlobalShift(shift_index);
    }
  }

  // ===========================================================================
  // Get index of grid that need to be freed. Only free grid that just "shift"
  // ===========================================================================
  template<typename T>
  inline void GetGridSDFIndexNeedFree(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        shift)
  {
    if (shift.x==0 && shift.y==0 && shift.z==0)
    {
      return;
    }

    std::cout<<"[GetGridSDFIndexNeedFree] Getting Grids index need to be free."<<
               "local shift ("<<shift.x<<","<<shift.y<<","<< shift.z<<")"<<std::endl;

    int nResetNum = 0;
    int3 Index;
    bool bReset = false;
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
          bReset = false;

          //----- for x -----
          if(bReset == false && shift.x > 0 &&
             Index.x >= pVol->m_local_shift.x - shift.x &&
             Index.x < pVol->m_local_shift.x)
          {
            bReset = true;
          }
          else if(bReset == false && shift.x < 0 &&
                  Index.x >= GridDim.x + pVol->m_local_shift.x &&
                  Index.x < GridDim.x + pVol->m_local_shift.x - shift.x)
          {
            bReset = true;
          }

          //----- for y -----
          if(bReset == false && shift.y > 0 &&
             Index.y >= pVol->m_local_shift.y - shift.y &&
             Index.y < pVol->m_local_shift.y)
          {
            bReset = true;
          }
          else if(bReset == false && shift.y < 0 &&
                  Index.x >= GridDim.y + pVol->m_local_shift.y &&
                  Index.x < GridDim.y + pVol->m_local_shift.y - shift.y)
          {
            bReset = true;
          }

          //----- for z -----
          if(bReset == false && shift.z > 0 &&
             Index.z >= pVol->m_local_shift.z - shift.z &&
             Index.z < pVol->m_local_shift.z)
          {
            bReset = true;
          }
          else if(bReset == false && shift.z < 0 &&
                  Index.x >= GridDim.z + pVol->m_local_shift.z &&
                  Index.x < GridDim.z + pVol->m_local_shift.z - shift.z)
          {
            bReset = true;
          }

          // -------------------------------------------------------------------
          // ---------- set flag for grid that need to be freed ----------------
          // -------------------------------------------------------------------
          int nGridIndex = Index.x + pVol->m_nGridRes_w * (Index.y + pVol->m_nGridRes_h*Index.z);

          if(bReset == true)
          {
            nResetNum ++;
            m_nNextResetSDFs.m_nNextResetSDFs[nGridIndex] = 1;
          }
          else
          {
            m_nNextResetSDFs.m_nNextResetSDFs[nGridIndex] = 0;
          }
        }
      }
    }

    std::cout<<"[GetGridSDFIndexNeedFree] Finished. Prepare to Reset & Free "<<
               nResetNum<<" grid;"<<std::endl;
  }

  // ===========================================================================
  template<typename T>
  inline void ResetAndFreeGird(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol)
  {
    for(unsigned int i=0; i!=pVol->GetTotalGridNum(); i++)
    {
      if(m_nNextResetSDFs.m_nNextResetSDFs[i] == 1 &&
         pVol->CheckIfBasicSDFActive(i) == true)
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
  NextResetSDF  m_nNextResetSDFs;
};

}


#endif // ROLLINGGRIDSDF_H
