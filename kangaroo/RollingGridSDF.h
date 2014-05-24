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
  RollingGridSDF()
  {
    m_local_shift = make_int3(0,0,0);
  }

  // ---------------------------------------------------------------------------
  // update shift parameters
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

    // update total shift
    m_local_shift = pVol->m_local_shift;
  }



  // ---------------------------------------------------------------------------
  // compute index of grid sdf that need to be reset and freed.
  // only free that part that we just "shift"
  template<typename T> inline
  void GetGridSDFIndexNeedFree(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol, int3 current_shift)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// iterator through all grid sdf and see if it need to be free
    //////////////////////////////////////////////////////////////////////////////
    // for each grid sdf in the volume
    if (current_shift.x!=0 || current_shift.y!=0 || current_shift.z!=0)
    {
      bool bReset = false;

      for(int i=0;i!=int(pVol->m_nWholeGridRes_w);i++)
      {
        for(int j=0;j!=int(pVol->m_nWholeGridRes_h);j++)
        {
          for(int k=0;k!=int(pVol->m_nWholeGridRes_d);k++)
          {
            // reset params;
            bReset = false;

            //----- for x
            if(current_shift.x>0 && i>=m_local_shift.x - current_shift.x && i<m_local_shift.x)
            {
              bReset = true;
              //              printf("[x]prepare free index %d,%d,%d\n", i, j, k);
            }

            if(current_shift.x<0 && i>= int(pVol->m_nWholeGridRes_w) + current_shift.x)
            {
              bReset = true;
              //          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
            }


            //----- for y
            if(current_shift.y>0 && j>=m_local_shift.y - current_shift.y && j <m_local_shift.y)
            {
              bReset = true;
              //              printf("[y]prepare free index %d,%d,%d\n", i, j, k);
            }

            if(current_shift.y<0 && j>= int(pVol->m_nWholeGridRes_h) + current_shift.y)
            {
              bReset = true;
              //          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
            }


            //----- for z
            if(current_shift.z>0 && k>=m_local_shift.z - current_shift.z && k<m_local_shift.z)
            {
              bReset = true;
              //              printf("[z]prepare free index %d,%d,%d\n", i, j, k);
            }

            if(current_shift.z<0 && k>= int(pVol->m_nWholeGridRes_d) + current_shift.z)
            {
              bReset = true;
              //          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
            }

            if(bReset == true)
            {
              m_nNextResetSDFs[i+pVol->m_nWholeGridRes_w*(j+pVol->m_nWholeGridRes_h*k)] = 1;
            }
            else
            {
              m_nNextResetSDFs[i+pVol->m_nWholeGridRes_w*(j+pVol->m_nWholeGridRes_h*k)] = 0;
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
      if(m_nNextResetSDFs[i] == 1 && pVol->CheckIfBasicSDFActive(i) == true)
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
  int  m_nNextResetSDFs[MAX_SUPPORT_GRID_NUM];
  int3 m_local_shift;
};


}


#endif // ROLLINGGRIDSDF_H
