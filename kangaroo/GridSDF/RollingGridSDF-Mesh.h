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
      pVol->UpdateLocalAndGlobalShift(shift_index);
    }
  }

  // ---------------------------------------------------------------------------
  // compute the index of grid sdf that need to be reset and freed.
  // only free that part that we just "shift"
  // ---------------------------------------------------------------------------
  template<typename T>
  inline void GetGridSDFIndexNeedFree(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        CurLocalShift)
  {
    std::cout<<"[GetGridSDFIndexNeedFree] Computing Grids index need to be freed.."<<std::endl;

    if (CurLocalShift.x==0 && CurLocalShift.y==0 && CurLocalShift.z==0)
    {
      return;
    }

    // for each grid sdf in the volume
    bool bReset = false;
    bool bx = false; bool by = false; bool bz = false;
    int nResetNum = 0;
    printf("Cur local shif (%d,%d,%d)\n",CurLocalShift.x,CurLocalShift.y,CurLocalShift.z);

    int3 Index;
    for(Index.x = 0; Index.x!=static_cast<int>(pVol->m_nGridRes_w); Index.x++)
    {
      for(Index.y = 0; Index.y!=static_cast<int>(pVol->m_nGridRes_h); Index.y++)
      {
        for(Index.z = 0; Index.z!=static_cast<int>(pVol->m_nGridRes_d); Index.z++)
        {
          // ----
          bReset = false;
          bx = false; by = false; bz = false;

          //----- for x
          if(CurLocalShift.x> 0 &&
             Index.x >= pVol->m_local_shift.x - CurLocalShift.x &&
             Index.x < pVol->m_local_shift.x)
          {
            bx = true;
            bReset = true;
          }
          else if(CurLocalShift.x<0 &&
                  Index.x >= static_cast<int>(pVol->m_nGridRes_w) + pVol->m_local_shift.x &&
                  Index.x < static_cast<int>(pVol->m_nGridRes_w) + pVol->m_local_shift.x-CurLocalShift.x)
          {
            //              bx = true;
            //              bReset = true;
          }

          //----- for y
          if(CurLocalShift.y>0 &&
             Index.y >= pVol->m_local_shift.y - CurLocalShift.y &&
             Index.y < pVol->m_local_shift.y)
          {
            by = true;
            bReset = true;
          }
          else if(CurLocalShift.y<0 &&
                  Index.x >= static_cast<int>(pVol->m_nGridRes_h) + pVol->m_local_shift.y &&
                  Index.x < static_cast<int>(pVol->m_nGridRes_h) + pVol->m_local_shift.y-CurLocalShift.y)
          {
            //              by = true;
            //              bReset = true;
          }

          //----- for z
          if(CurLocalShift.z>0 &&
             Index.z >= pVol->m_local_shift.z - CurLocalShift.z &&
             Index.z < pVol->m_local_shift.z)
          {
            bz = true;
            bReset = true;
          }
          else if(CurLocalShift.z<0 &&
                  Index.x >= static_cast<int>(pVol->m_nGridRes_d) + pVol->m_local_shift.z &&
                  Index.x < static_cast<int>(pVol->m_nGridRes_d) + pVol->m_local_shift.z - CurLocalShift.z)
          {
            //              bz = true;
            //              bReset = true;
          }

          // ---------- set flag for grid that need to be freed ------------
          int nGridIndex = Index.x + pVol->m_nGridRes_w*(Index.y + pVol->m_nGridRes_h*Index.z);

          if(bReset == true)
          {
            m_nNextResetSDFs.m_nNextResetSDFs[nGridIndex] = 1;
            nResetNum ++;
          }
          else
          {
            m_nNextResetSDFs.m_nNextResetSDFs[nGridIndex] = 0;
          }

          if(bx == true)
          {
            m_nNextResetSDFs.m_x[nGridIndex] = 1;
          }
          else
          {
            m_nNextResetSDFs.m_x[nGridIndex] = 0;
          }

          if(by == true)
          {
            m_nNextResetSDFs.m_y[nGridIndex] = 1;
          }
          else
          {
            m_nNextResetSDFs.m_y[nGridIndex] = 0;
          }

          if(bz == true)
          {
            m_nNextResetSDFs.m_z[nGridIndex] = 1;
          }
          else
          {
            m_nNextResetSDFs.m_z[nGridIndex] = 0;
          }
        }
      }
    }

    std::cout<<"[GetGridSDFIndexNeedFree] Finished. Reset & Free "<<nResetNum<<" grid;"<<std::endl;
  }

  // ---------------------------------------------------------------------------
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
