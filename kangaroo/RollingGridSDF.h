#ifndef ROLLINGGRIDSDF_H
#define ROLLINGGRIDSDF_H

#include "BoundedVolumeGrid.h"
#include "cu_sdffusion.h"

namespace roo {

// the folling is a CPU version of rolling grid sdf
class RollingGridSDF
{
public:
  // update shift parameters
  template<typename T> inline
  void UpdateShift(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol, int3 shift_index)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// change bbox min and max value based on shif parameters
    //////////////////////////////////////////////////////////////////////////////
    printf("size x=%f, y=%f, z=%f; shift is x=%d,y=%d,z=%d\n ",
           pVol->m_bbox.Size().x,pVol->m_bbox.Size().y,pVol->m_bbox.Size().z,
           shift_index.x, shift_index.y,shift_index.z);

    float3 BBSize = pVol->m_bbox.Size();

    /// 1, Compute the latest bounding box
    if(shift_index.x!=0)
    {
      pVol->m_bbox.boxmin.x = pVol->m_bbox.boxmin.x +
          float(shift_index.x) * BBSize.x/float(pVol->m_nWholeGridRes);

      pVol->m_bbox.boxmax.x = pVol->m_bbox.boxmax.x +
          float(shift_index.x) * BBSize.x/float(pVol->m_nWholeGridRes);

      printf("shift x is %d, total shift x is %f, change bbox bbmin x to %f, bbmax x to %f\n",
             shift_index.x, float(shift_index.x) * BBSize.x/float(pVol->m_nWholeGridRes),
             pVol->m_bbox.boxmin.x, pVol->m_bbox.boxmax.x);
    }

    if(shift_index.y!=0)
    {
      pVol->m_bbox.boxmin.y = pVol->m_bbox.boxmin.y +
          float(shift_index.y) * BBSize.y/float(pVol->m_nWholeGridRes);

      pVol->m_bbox.boxmax.y = pVol->m_bbox.boxmax.y +
          float(shift_index.y) * BBSize.y/float(pVol->m_nWholeGridRes);

      printf("shift y is %d, total shift y is %f, change bbox bbmin y to %f, bbmax y to %f\n",
             shift_index.y, float(shift_index.y) * BBSize.y/float(pVol->m_nWholeGridRes),
             pVol->m_bbox.boxmin.y, pVol->m_bbox.boxmax.y);
    }

    if(shift_index.z!=0)
    {
      pVol->m_bbox.boxmin.z = pVol->m_bbox.boxmin.z +
          float(shift_index.z) * BBSize.z/float(pVol->m_nWholeGridRes);

      pVol->m_bbox.boxmax.z = pVol->m_bbox.boxmax.z +
          float(shift_index.z) * BBSize.z/float(pVol->m_nWholeGridRes);

      printf("shift z is %d, total shift z is %f, change bbox bbmin z to %f, bbmax z to %f\n",
             shift_index.z, float(shift_index.z) * BBSize.z/float(pVol->m_nWholeGridRes),
             pVol->m_bbox.boxmin.z, pVol->m_bbox.boxmax.z);
    }

    // save shift params in grid and grid grey sdf data struct
    pVol->UpdateShift(shift_index);
  }



  // compute index of grid sdf that need to be freed
  template<typename T> inline
  void GetGridSDFIndexNeedFree(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol, int3 shift_index)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// iterator through all grid sdf and see if it need to be free
    //////////////////////////////////////////////////////////////////////////////
    // for each grid sdf in the volume
    for(int i=0;i!=int(pVol->m_nWholeGridRes);i++)
    {
      for(int j=0;j!=int(pVol->m_nWholeGridRes);j++)
      {
        for(int k=0;k!=int(pVol->m_nWholeGridRes);k++)
        {
          // for x
          if(shift_index.x>0 && i<shift_index.x)
          {
            nNextResetSDFs[i+pVol->m_nWholeGridRes*(j+pVol->m_nWholeGridRes*k)] =1;
            //          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
          }

          if(shift_index.x<0 && i>= int(pVol->m_nWholeGridRes) + shift_index.x)
          {
            nNextResetSDFs[i+pVol->m_nWholeGridRes*(j+pVol->m_nWholeGridRes*k)] =1;
            //          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
          }


          // for y
          if(shift_index.y>0 && j<shift_index.y)
          {
            nNextResetSDFs[i+pVol->m_nWholeGridRes*(j+pVol->m_nWholeGridRes*k)] =1;
            //          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
          }

          if(shift_index.y<0 && j>= int(pVol->m_nWholeGridRes) + shift_index.y)
          {
            nNextResetSDFs[i+pVol->m_nWholeGridRes*(j+pVol->m_nWholeGridRes*k)] =1;
            //          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
          }


          // for z
          if(shift_index.z>0 && k<shift_index.z)
          {
            nNextResetSDFs[i+pVol->m_nWholeGridRes*(j+pVol->m_nWholeGridRes*k)] =1;
            //          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
          }

          if(shift_index.z<0 && k>= int(pVol->m_nWholeGridRes) + shift_index.z)
          {
            nNextResetSDFs[i+pVol->m_nWholeGridRes*(j+pVol->m_nWholeGridRes*k)] =1;
            //          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
          }
        }
      }
    }
  }

  template<typename T> inline
  void FreeGird(roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage>* pVol)
  {
    //////////////////////////////////////////////////////////////////////////////
    /// free grid sdf
    //////////////////////////////////////////////////////////////////////////////

    for(unsigned int i=0;i!=pVol->m_nWholeGridRes* pVol->m_nWholeGridRes* pVol->m_nWholeGridRes; i++)
    {
      if(nNextResetSDFs[i] == 1)
      {
        if (pVol->CheckIfBasicSDFActive(i) == true)
        {
          roo::SdfReset(pVol->m_GridVolumes[i]);
          pVol->FreeMemoryByIndex(i);

          pVol->m_GridVolumes[i].d = 0;
          pVol->m_GridVolumes[i].w = 0;
          pVol->m_GridVolumes[i].h = 0;
        }

        nNextResetSDFs[i]=0;
      }
    }
  }

private:
  int nNextResetSDFs[1024];
};


}


#endif // ROLLINGGRIDSDF_H
