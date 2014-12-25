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

// the folling is the CPU version of rolling grid sdf
class RollingGridSDFMesh
{
public:
  // ---------------------------------------------------------------------------
  // update shift parameters. if true repersent update global shift
  // ---------------------------------------------------------------------------
  template<typename T>
  inline void ApplyShiftToVolume(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        shift_index,
      bool                                                        bVerbose = false)
  {
    // -------------------------------------------------------------------------
    // change bbox min and max value based on shif parameters
    // -------------------------------------------------------------------------
    if(bVerbose)
    {
      printf("[UpdateShift] new shift for current frame is x=%d,y=%d,z=%d; Updating BB.\n",
             shift_index.x, shift_index.y,shift_index.z);
    }

    float3 BBSize = pVol->m_bbox.Size();

    // -------------------------------------------------------------------------
    // 1, Compute the latest bounding box
    // -------------------------------------------------------------------------
    if(shift_index.x!=0)
    {
      pVol->m_bbox.boxmin.x = pVol->m_bbox.boxmin.x +
          static_cast<float>(shift_index.x) * BBSize.x/static_cast<float>(pVol->m_nGridRes_w);

      pVol->m_bbox.boxmax.x = pVol->m_bbox.boxmax.x +
          static_cast<float>(shift_index.x) * BBSize.x/static_cast<float>(pVol->m_nGridRes_w);

      if(bVerbose)
      {
        printf("[UpdateShift] shift x:%d (index), %f(m), change bbox bbmin x to %f, bbmax x to %f\n",
               shift_index.x, static_cast<float>(shift_index.x) * BBSize.x/static_cast<float>(pVol->m_nGridRes_w),
               pVol->m_bbox.boxmin.x, pVol->m_bbox.boxmax.x);
      }
    }

    if(shift_index.y!=0)
    {
      pVol->m_bbox.boxmin.y = pVol->m_bbox.boxmin.y +
          static_cast<float>(shift_index.y) * BBSize.y/static_cast<float>(pVol->m_nGridRes_h);

      pVol->m_bbox.boxmax.y = pVol->m_bbox.boxmax.y +
          static_cast<float>(shift_index.y) * BBSize.y/static_cast<float>(pVol->m_nGridRes_h);

      if(bVerbose)
      {
        printf("[UpdateShift] shift y:%d(index), %f(m), change bbox bbmin y to %f, bbmax y to %f\n",
               shift_index.y, static_cast<float>(shift_index.y) * BBSize.y/static_cast<float>(pVol->m_nGridRes_h),
               pVol->m_bbox.boxmin.y, pVol->m_bbox.boxmax.y);
      }
    }

    if(shift_index.z!=0)
    {
      pVol->m_bbox.boxmin.z = pVol->m_bbox.boxmin.z +
          static_cast<float>(shift_index.z) * BBSize.z/static_cast<float>(pVol->m_nGridRes_d);

      pVol->m_bbox.boxmax.z = pVol->m_bbox.boxmax.z +
          static_cast<float>(shift_index.z) * BBSize.z/static_cast<float>(pVol->m_nGridRes_d);

      if(bVerbose)
      {
        printf("[UpdateShift] shift z:%d(index), %f(m), change bbox bbmin z to %f, bbmax z to %f\n",
               shift_index.z, static_cast<float>(shift_index.z) * BBSize.z/static_cast<float>(pVol->m_nGridRes_d),
               pVol->m_bbox.boxmin.z, pVol->m_bbox.boxmax.z);
      }
    }

    // reset local and global shift
    if(shift_index.x!=0 || shift_index.y!= 0 || shift_index.z!=0)
    {
      pVol->UpdateGlobalShift(shift_index);
    }
  }

  // ---------------------------------------------------------------------------
  // compute index of grid sdf that need to be reset and freed.
  // only free that part that we just "shift"
  // ---------------------------------------------------------------------------
  template<typename T>
  inline void GetGridSDFIndexNeedFree(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        CurLocalShift)
  {
    std::cout<<"[GetGridSDFIndexNeedFree] Computing Grids index need to be freed"<<std::endl;

    // for each grid sdf in the volume
    if (CurLocalShift.x!=0 || CurLocalShift.y!=0 || CurLocalShift.z!=0)
    {
      bool bReset = false;
      bool bx = false; bool by = false; bool bz = false;

      printf("cur local shif x%d,y%d,z%d\n", CurLocalShift.x,CurLocalShift.y,CurLocalShift.z);

      for(int nIndex_x=0; nIndex_x!=int(pVol->m_nGridRes_w); nIndex_x++)
      {
        for(int nIndex_y=0; nIndex_y!=int(pVol->m_nGridRes_h); nIndex_y++)
        {
          for(int nIndex_z=0; nIndex_z!=int(pVol->m_nGridRes_d); nIndex_z++)
          {
            // reset params;
            bReset = false;
            bx = false; by = false; bz = false;

            //----- for x
            if(CurLocalShift.x> 0 &&
               nIndex_x >= pVol->m_local_shift.x - CurLocalShift.x &&
               nIndex_x < pVol->m_local_shift.x)
            {
              bx = true;
              bReset = true;
            }
            else if(CurLocalShift.x<0 &&
                    nIndex_x >= static_cast<int>(pVol->m_nGridRes_w) + pVol->m_local_shift.x &&
                    nIndex_x < static_cast<int>(pVol->m_nGridRes_w) + pVol->m_local_shift.x-CurLocalShift.x)
            {
              //              bx = true;
              //              bReset = true;
            }

            //----- for y
            if(CurLocalShift.y>0 &&
               nIndex_y >= pVol->m_local_shift.y - CurLocalShift.y &&
               nIndex_y < pVol->m_local_shift.y)
            {
              by = true;
              bReset = true;
            }
            else if(CurLocalShift.y<0 &&
                    nIndex_x >= static_cast<int>(pVol->m_nGridRes_h) + pVol->m_local_shift.y &&
                    nIndex_x < static_cast<int>(pVol->m_nGridRes_h) + pVol->m_local_shift.y-CurLocalShift.y)
            {
              //              by = true;
              //              bReset = true;
            }

            //----- for z
            if(CurLocalShift.z>0 &&
               nIndex_z >= pVol->m_local_shift.z - CurLocalShift.z &&
               nIndex_z < pVol->m_local_shift.z)
            {
              bz = true;
              bReset = true;
            }
            else if(CurLocalShift.z<0 &&
               nIndex_x >= static_cast<int>(pVol->m_nGridRes_d) + pVol->m_local_shift.z &&
               nIndex_x < static_cast<int>(pVol->m_nGridRes_d) + pVol->m_local_shift.z - CurLocalShift.z)
            {
              //              bz = true;
              //              bReset = true;
            }

            // ---------- set flag for grid that need to be freed ------------
            int nIndex = nIndex_x+pVol->m_nGridRes_w*(nIndex_y+pVol->m_nGridRes_h*nIndex_z);

            if(bReset == true)
            {
              m_nNextResetSDFs.m_nNextResetSDFs[nIndex] = 1;
            }
            else
            {
              m_nNextResetSDFs.m_nNextResetSDFs[nIndex] = 0;
            }

            if(bx == true)
            {
              m_nNextResetSDFs.m_x[nIndex] = 1;
            }
            else
            {
              m_nNextResetSDFs.m_x[nIndex] = 0;
            }

            if(by == true)
            {
              m_nNextResetSDFs.m_y[nIndex] = 1;
            }
            else
            {
              m_nNextResetSDFs.m_y[nIndex] = 0;
            }

            if(bz == true)
            {
              m_nNextResetSDFs.m_z[nIndex] = 1;
            }
            else
            {
              m_nNextResetSDFs.m_z[nIndex] = 0;
            }
          }
        }
      }
    }

    std::cout<<"[GetGridSDFIndexNeedFree] Finished"<<std::endl;
  }

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
