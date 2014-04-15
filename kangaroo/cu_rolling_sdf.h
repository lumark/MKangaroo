#pragma once

#include "Mat.h"
#include "Image.h"
#include "ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "BoundedVolume.h"
#include "Sdf.h"

namespace roo
{
void SdfResetPartial(BoundedVolume<SDF_t> vol, float3 shift);

// move grid sdf around which enable it to fuse new pixels
void RollingGridSdfCuda(BoundedVolume<SDF_t> vol, float3 shift);


inline void RollingGridSdf(BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol, int3 shift_index)
{
  // 1, Compute the latest bounding box
  if(shift_index.x!=0)
  {
    vol.m_bbox.boxmin.x = vol.m_bbox.boxmin.x + shift_index.x * vol.m_bbox.Size().x/float(vol.m_WholeGridRes);
    vol.m_bbox.boxmax.x = vol.m_bbox.boxmax.x + shift_index.x * vol.m_bbox.Size().x/float(vol.m_WholeGridRes);
  }

  if(shift_index.y!=0)
  {
    vol.m_bbox.boxmin.y = vol.m_bbox.boxmin.y + shift_index.y * vol.m_bbox.Size().y/float(vol.m_WholeGridRes);
    vol.m_bbox.boxmax.y = vol.m_bbox.boxmax.y + shift_index.y * vol.m_bbox.Size().y/float(vol.m_WholeGridRes);
  }

  if(shift_index.z!=0)
  {
    vol.m_bbox.boxmin.z = vol.m_bbox.boxmin.z + shift_index.z * vol.m_bbox.Size().z/float(vol.m_WholeGridRes);
    vol.m_bbox.boxmax.z = vol.m_bbox.boxmax.z + shift_index.z * vol.m_bbox.Size().z/float(vol.m_WholeGridRes);
  }

  // save shift params in grid sdf data struct
  vol.m_shift = shift_index;

  int nNextResetSDFs[vol.m_WholeGridRes*vol.m_WholeGridRes*vol.m_WholeGridRes];

  // for each grid sdf in the volume
  for(int i=0;i!=int(vol.m_WholeGridRes);i++)
  {
    for(int j=0;j!=int(vol.m_WholeGridRes);j++)
    {
      for(int k=0;k!=int(vol.m_WholeGridRes);k++)
      {
        // for x
        if(shift_index.x>0 && i<shift_index.x)
        {
          nNextResetSDFs[i+vol.m_WholeGridRes*(j+vol.m_WholeGridRes*k)] =1;
          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
        }

        if(shift_index.x<0 && i>= int(vol.m_WholeGridRes) + shift_index.x)
        {
          nNextResetSDFs[i+vol.m_WholeGridRes*(j+vol.m_WholeGridRes*k)] =1;
          printf("[x]prepare free index %d,%d,%d\n", i, j, k);
        }


        // for y
        if(shift_index.y>0 && j<shift_index.y)
        {
          nNextResetSDFs[i+vol.m_WholeGridRes*(j+vol.m_WholeGridRes*k)] =1;
          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
        }

        if(shift_index.y<0 && j>= int(vol.m_WholeGridRes) + shift_index.y)
        {
          nNextResetSDFs[i+vol.m_WholeGridRes*(j+vol.m_WholeGridRes*k)] =1;
          printf("[y]prepare free index %d,%d,%d\n", i, j, k);
        }


        // for z
        if(shift_index.z>0 && k<shift_index.z)
        {
          nNextResetSDFs[i+vol.m_WholeGridRes*(j+vol.m_WholeGridRes*k)] =1;
          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
        }

        if(shift_index.z<0 && k>= int(vol.m_WholeGridRes) + shift_index.z)
        {
          nNextResetSDFs[i+vol.m_WholeGridRes*(j+vol.m_WholeGridRes*k)] =1;
          printf("[z]prepare free index %d,%d,%d\n", i, j, k);
        }
      }
    }
  }


  // reset sdf
  // reset
  for(unsigned int i=0;i!=vol.m_WholeGridRes*vol.m_WholeGridRes*vol.m_WholeGridRes;i++)
  {
    if(nNextResetSDFs[i]==1)
    {
      //      vol.FreeMemoryByIndex(i);
      //      printf("free index %d, shitf is:%f,%f,%f;",i,shift_index.x,shift_index.y, shift_index.z);
    }
  }

}


}
