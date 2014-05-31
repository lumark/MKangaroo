#ifndef ROLLINGGRIDSDF_H
#define ROLLINGGRIDSDF_H

#include "BoundedVolumeGrid.h"
#include "cu_sdffusion.h"
#include "extra/SavePPMGrid.h"


namespace roo {

// the folling is a CPU version of rolling grid sdf
class RollingGridSDF
{
public:
  // ---------------------------------------------------------------------------
  // update shift parameters. if true repersent update global shift
  template<typename T> inline
  void UpdateShift(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol,
                   int3 shift_index, bool bVerbose=false)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// change bbox min and max value based on shif parameters
    //////////////////////////////////////////////////////////////////////////////
    if(bVerbose==true)
    {
      printf("[UpdateShift] new shift for current frame is x=%d,y=%d,z=%d; Updating BB.\n",
             shift_index.x, shift_index.y,shift_index.z);
    }

    float3 BBSize = pVol->m_bbox.Size();

    /// 1, Compute the latest bounding box
    if(shift_index.x!=0)
    {
      pVol->m_bbox.boxmin.x = pVol->m_bbox.boxmin.x +
          float(shift_index.x) * BBSize.x/float(pVol->m_nWholeGridRes_w);

      pVol->m_bbox.boxmax.x = pVol->m_bbox.boxmax.x +
          float(shift_index.x) * BBSize.x/float(pVol->m_nWholeGridRes_w);

      if(bVerbose==true)
      {
        printf("[UpdateShift] shift x:%d (index), %f(m), change bbox bbmin x to %f, bbmax x to %f\n",
               shift_index.x, float(shift_index.x) * BBSize.x/float(pVol->m_nWholeGridRes_w),
               pVol->m_bbox.boxmin.x, pVol->m_bbox.boxmax.x);
      }
    }

    if(shift_index.y!=0)
    {
      pVol->m_bbox.boxmin.y = pVol->m_bbox.boxmin.y +
          float(shift_index.y) * BBSize.y/float(pVol->m_nWholeGridRes_h);

      pVol->m_bbox.boxmax.y = pVol->m_bbox.boxmax.y +
          float(shift_index.y) * BBSize.y/float(pVol->m_nWholeGridRes_h);

      if(bVerbose==true)
      {
        printf("[UpdateShift] shift y:%d(index), %f(m), change bbox bbmin y to %f, bbmax y to %f\n",
               shift_index.y, float(shift_index.y) * BBSize.y/float(pVol->m_nWholeGridRes_h),
               pVol->m_bbox.boxmin.y, pVol->m_bbox.boxmax.y);
      }
    }

    if(shift_index.z!=0)
    {
      pVol->m_bbox.boxmin.z = pVol->m_bbox.boxmin.z +
          float(shift_index.z) * BBSize.z/float(pVol->m_nWholeGridRes_d);

      pVol->m_bbox.boxmax.z = pVol->m_bbox.boxmax.z +
          float(shift_index.z) * BBSize.z/float(pVol->m_nWholeGridRes_d);

      if(bVerbose==true)
      {
        printf("[UpdateShift] shift z:%d(index), %f(m), change bbox bbmin z to %f, bbmax z to %f\n",
               shift_index.z, float(shift_index.z) * BBSize.z/float(pVol->m_nWholeGridRes_d),
               pVol->m_bbox.boxmin.z, pVol->m_bbox.boxmax.z);
      }
    }

    // reset local and global shift
    if(shift_index.x!=0 || shift_index.y!= 0 || shift_index.z!=0)
    {
      pVol->ResetShift(shift_index);
    }
  }



  class NextResetSDF
  {
  public:
    int  m_nNextResetSDFs[MAX_SUPPORT_GRID_NUM];
    int  m_x[MAX_SUPPORT_GRID_NUM];
    int  m_y[MAX_SUPPORT_GRID_NUM];
    int  m_z[MAX_SUPPORT_GRID_NUM];
  };

  // ---------------------------------------------------------------------------
  // compute index of grid sdf that need to be reset and freed.
  // only free that part that we just "shift"
  template<typename T> inline
  void GetGridSDFIndexNeedFree(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>*  pVol,
      int3                                                        CurLocalShift)
  {
    // for each grid sdf in the volume
    if (CurLocalShift.x!=0 || CurLocalShift.y!=0 || CurLocalShift.z!=0)
    {
      bool bReset = false;
      bool bx = false;
      bool by = false;
      bool bz = false;

      for(int i=0;i!=int(pVol->m_nWholeGridRes_w);i++)
      {
        for(int j=0;j!=int(pVol->m_nWholeGridRes_h);j++)
        {
          for(int k=0;k!=int(pVol->m_nWholeGridRes_d);k++)
          {
            // reset params;
            bReset = false;
            bx = false;
            by = false;
            bz = false;

            //----- for x
            if(CurLocalShift.x>0 && i>=pVol->m_local_shift.x - CurLocalShift.x && i<pVol->m_local_shift.x)
            {
              bx = true;
              bReset = true;
            }
            if(CurLocalShift.x<0 && i>= int(pVol->m_nWholeGridRes_w) + CurLocalShift.x)
            {
              bx = true;
              bReset = true;
            }


            //----- for y
            if(CurLocalShift.y>0 && j>=pVol->m_local_shift.y - CurLocalShift.y && j <pVol->m_local_shift.y)
            {
              by = true;
              bReset = true;
            }
            if(CurLocalShift.y<0 && j>= int(pVol->m_nWholeGridRes_h) + CurLocalShift.y)
            {
              by = true;
              bReset = true;
            }


            //----- for z
            if(CurLocalShift.z>0 && k>=pVol->m_local_shift.z - CurLocalShift.z && k<pVol->m_local_shift.z)
            {
              bz = true;
              bReset = true;
            }
            if(CurLocalShift.z<0 && k>= int(pVol->m_nWholeGridRes_d) + CurLocalShift.z)
            {
              bz = true;
              bReset = true;
            }

            // set flag
            int nIndex = i+pVol->m_nWholeGridRes_w*(j+pVol->m_nWholeGridRes_h*k);

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
  }



  template<typename T> inline
  void ResetAndFreeGird(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// free grid sdf
    //////////////////////////////////////////////////////////////////////////////
    for(unsigned int i=0;i!=pVol->m_nWholeGridRes_w* pVol->m_nWholeGridRes_h* pVol->m_nWholeGridRes_d; i++)
    {
      if(m_nNextResetSDFs.m_nNextResetSDFs[i] == 1 && pVol->CheckIfBasicSDFActive(i) == true)
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
