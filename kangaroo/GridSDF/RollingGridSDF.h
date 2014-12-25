#ifndef ROLLINGGRIDSDF_H
#define ROLLINGGRIDSDF_H

#include "BoundedVolumeGrid.h"
#include "kangaroo/cu_sdffusion.h"
#include "cu_sdffusion_grid.h"

namespace roo {

class RollingGridSDF
{
public:
  RollingGridSDF()
  {
    TotalShift = make_int3(0,0,0);
  }

  // ---------------------------------------------------------------------------
  // update shift parameters
  template<typename T> inline
  void UpdateShift(
      roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol,
      int3                                                       shift_index)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// change bbox min and max value based on the shif parameters
    //////////////////////////////////////////////////////////////////////////////
    printf("BB size x=%f, y=%f, z=%f; Current shift is x=%d,y=%d,z=%d\n ",
           pVol->m_bbox.Size().x,pVol->m_bbox.Size().y,pVol->m_bbox.Size().z,
           shift_index.x, shift_index.y,shift_index.z);

    float3 BBSize = pVol->m_bbox.Size();

    /// 1, Compute the latest bounding box
    if(shift_index.x!=0)
    {
      pVol->m_bbox.boxmin.x = pVol->m_bbox.boxmin.x +
          float(shift_index.x) * BBSize.x/float(pVol->m_nGridRes_w);

      pVol->m_bbox.boxmax.x = pVol->m_bbox.boxmax.x +
          float(shift_index.x) * BBSize.x/float(pVol->m_nGridRes_w);

      printf("shift x is %d, total shift x is %f, change bbox bbmin x to %f, bbmax x to %f\n",
             shift_index.x, float(shift_index.x) * BBSize.x/float(pVol->m_nGridRes_w),
             pVol->m_bbox.boxmin.x, pVol->m_bbox.boxmax.x);
    }

    if(shift_index.y!=0)
    {
      pVol->m_bbox.boxmin.y = pVol->m_bbox.boxmin.y +
          float(shift_index.y) * BBSize.y/float(pVol->m_nGridRes_h);

      pVol->m_bbox.boxmax.y = pVol->m_bbox.boxmax.y +
          float(shift_index.y) * BBSize.y/float(pVol->m_nGridRes_h);

      printf("shift y is %d, total shift y is %f, change bbox bbmin y to %f, bbmax y to %f\n",
             shift_index.y, float(shift_index.y) * BBSize.y/float(pVol->m_nGridRes_h),
             pVol->m_bbox.boxmin.y, pVol->m_bbox.boxmax.y);
    }

    if(shift_index.z!=0)
    {
      pVol->m_bbox.boxmin.z = pVol->m_bbox.boxmin.z +
          float(shift_index.z) * BBSize.z/float(pVol->m_nGridRes_d);

      pVol->m_bbox.boxmax.z = pVol->m_bbox.boxmax.z +
          float(shift_index.z) * BBSize.z/float(pVol->m_nGridRes_d);

      printf("shift z index is %d, shift z is %f, change bbox bbmin z to %f, bbmax z to %f\n",
             shift_index.z, float(shift_index.z) * BBSize.z/float(pVol->m_nGridRes_d),
             pVol->m_bbox.boxmin.z, pVol->m_bbox.boxmax.z);
    }

    // 2, save shift params in grid and grid grey sdf data struct
    pVol->UpdateShift(shift_index);

    // update total shift
    TotalShift = pVol->m_local_shift;
  }

  // ---------------------------------------------------------------------------
  // compute index of grid sdf that need to be freed. for each single that was shift,
  // we need to reset it
  template<typename T> inline
  void GetGridSDFIndexNeedFree(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// iterator through all grid sdf and see if it need to be free
    //////////////////////////////////////////////////////////////////////////////
    // for each grid sdf in the volume
    int nNum = 0;
    for(int i=0;i!=int(pVol->m_nGridRes_w);i++)
    {
      for(int j=0;j!=int(pVol->m_nGridRes_h);j++)
      {
        for(int k=0;k!=int(pVol->m_nGridRes_d);k++)
        {
          // set it back to 0;
          nNextResetSDFs[i]=0;

          // for x
          if(TotalShift.x>0 && i<TotalShift.x)
          {
            nNextResetSDFs[i+pVol->m_nGridRes_w*(j+pVol->m_nGridRes_h*k)] =1;
            //          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
          }
          if(TotalShift.x<0 && i>= int(pVol->m_nGridRes_w) + TotalShift.x)
          {
            nNextResetSDFs[i+pVol->m_nGridRes_w*(j+pVol->m_nGridRes_h*k)] =1;
            //          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
          }
          // for y
          if(TotalShift.y>0 && j<TotalShift.y)
          {
            nNextResetSDFs[i+pVol->m_nGridRes_w*(j+pVol->m_nGridRes_h*k)] =1;
            //          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
          }
          if(TotalShift.y<0 && j>= int(pVol->m_nGridRes_h) + TotalShift.y)
          {
            nNextResetSDFs[i+pVol->m_nGridRes_w*(j+pVol->m_nGridRes_h*k)] =1;
            //          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
          }

          // for z
          if(TotalShift.z>0 && k<TotalShift.z)
          {
            nNextResetSDFs[i+pVol->m_nGridRes_w*(j+pVol->m_nGridRes_h*k)] =1;
            //          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
          }

          if(TotalShift.z<0 && k>= int(pVol->m_nGridRes_d) + TotalShift.z)
          {
            nNextResetSDFs[i+pVol->m_nGridRes_w*(j+pVol->m_nGridRes_h*k)] =1;
            //          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
          }
        }
      }
    }

    printf("Num of Grid SDF need to be freed is %d\n", nNum);
  }

  // ---------------------------------------------------------------------------
  template<typename T> inline
  void FreeGird(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// free grid sdf
    //////////////////////////////////////////////////////////////////////////////
    int nFreeNum = 0;
    int nNeedFreeNum = 0;
    for(unsigned int i=0;i!=pVol->GetTotalGridNum(); i++)
    {
      if(nNextResetSDFs[i] == 1)
      {
        nNeedFreeNum++;

        // check if this vol is valid.
        if (pVol->CheckIfBasicSDFActive(i) == true)
        {
          roo::SdfReset(pVol->m_GridVolumes[i]);
          pVol->FreeMemoryByIndex(i);
          pVol->m_GridVolumes[i].d = 0;
          pVol->m_GridVolumes[i].w = 0;
          pVol->m_GridVolumes[i].h = 0;
          nFreeNum ++;
        }
      }
    }

    printf("[RollingGridSDF/FreeGrid] Free %d grid sdf. Need to free %d grid sdf.\n",
           nFreeNum, nNeedFreeNum);
  }

private:
  int       nNextResetSDFs[1024];
  int3      TotalShift;
};


}


#endif // ROLLINGGRIDSDF_H
