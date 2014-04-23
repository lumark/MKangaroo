#include "cu_sdffusion.h"

#include "MatUtils.h"
#include "launch_utils.h"

namespace roo
{
//////////////////////////////////////////////////////
/// Rolling SDF
//////////////////////////////////////////////////////

// Boxmin and boxmax define the box that is to be kept intact, rest will be cleared.
// This approach makes if conditions inside simpler.
// TODO: Name the function better.
__global__ void KernSdfResetPartial(BoundedVolume<SDF_t> vol, float3 boxmin, float3 boxmax)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;
  const int z = blockIdx.z*blockDim.z + threadIdx.z;

  const float3 P_w = vol.VoxelPositionInUnits(x,y,z);

  bool mincrit, maxcrit;//if mincrit and maxcrit are true, point is inside the box, i.e. valid.
  mincrit = P_w.x > boxmin.x && P_w.y < boxmax.y && P_w.z > boxmin.z;
  maxcrit = P_w.x < boxmax.x && P_w.y > boxmin.y && P_w.z < boxmax.z;

  if(!mincrit || !maxcrit)//i.e. the point is outside the box.
  {
    vol(x,y,z) = SDF_t(0.0/0.0,0.0);
  }
}


// TODO: Name the function better.
void SdfResetPartial(BoundedVolume<SDF_t> vol, float3 shift)
{
  //Initialization for GPU parallelization
  dim3 blockDim(8,8,8);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);

  // compute the box to keep, it's conter intuitive to the name of function but
  // more efficient.
  float3 bn = vol.bbox.boxmin, bx = vol.bbox.boxmax;//bn is box min and bx is box max.

  if(shift.x>0)
    bn.x += shift.x;
  else
    bx.x += shift.x;

  // y is -ve, but boxmax and boxmin for y are also inverse. i.e. the bottom most
  // point is min.x,max.y,min.z
  if(shift.y>0)
    bn.y += shift.y;
  else
    bx.y += shift.y;

  if(shift.z>0)
    bn.z += shift.z;
  else
    bx.z += shift.z;

  KernSdfResetPartial<<<gridDim,blockDim>>>(vol, bn, bx);
  GpuCheckErrors();
}



//////////////////////////////////////////////////////
/// Rolling GRID SDF
//////////////////////////////////////////////////////

__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>  g_vol;
__device__ int                                                       g_NextResetSDFs[512];

// =============================================================================
// Boxmin and boxmax define the box that is to be kept intact, rest will be cleared.
// This approach makes if conditions inside simpler.
// When we clean a grid sdf, we also need to free its memory.. This maybe a little
// bit expensive
// =============================================================================

__global__ void KernRollingGridSdf(float3 boxmin, float3 boxmax, float3 shift)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol.m_d; ++z)
  {
    const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

    bool mincrit, maxcrit;//if mincrit and maxcrit are true, point is inside the box, i.e. valid.
    mincrit = P_w.x > boxmin.x && P_w.y > boxmin.y && P_w.z > boxmin.z;
    maxcrit = P_w.x < boxmax.x && P_w.y < boxmax.y && P_w.z < boxmax.z;

    if(!mincrit || !maxcrit)//i.e. the point is outside the box.
    {
      g_vol(x,y,z) = SDF_t(0.0/0.0,0.0);

      // get the index of grid sdf that need to be reseted
      int nIndex = int(floorf(x/g_vol.m_nVolumeGridRes)) +
          g_vol.m_nWholeGridRes * ( int(floorf(y/g_vol.m_nVolumeGridRes)) +
                                   g_vol.m_nWholeGridRes * int(floorf(z/g_vol.m_nVolumeGridRes)) );

      // save index of sdf that need to be reset later
      g_NextResetSDFs[nIndex] = 1;
    }

  }

}



// =============================================================================
// Rolling grid sdf. Each time we compute the index of sdf that we are going
// to reset and free the memory of it.
// the nature of rolling sdf is just shifting the index of sdf.
// shift is n * grid num in one diminsion. It can be positive or negative.
// by now each time we need to move at least one grid volume
// =============================================================================
void RollingGridSdfCuda(int* pNextInitSDFs, BoundedVolumeGrid<SDF_t> vol, int3 shift)
{
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // 1, Compute the latest bounding box
  float3 bb_min = vol.m_bbox.boxmin, bb_max = vol.m_bbox.boxmax;

  if(shift.x!=0)
  {
    bb_min.x = bb_min.x + shift.x * vol.m_bbox.Size().x/float(vol.m_nWholeGridRes);
    bb_max.x = bb_max.x + shift.x * vol.m_bbox.Size().x/float(vol.m_nWholeGridRes);
  }

  if(shift.y!=0)
  {
    bb_min.y = bb_min.y + shift.y * vol.m_bbox.Size().y/float(vol.m_nWholeGridRes);
    bb_max.y = bb_max.y + shift.y * vol.m_bbox.Size().y/float(vol.m_nWholeGridRes);
  }

  if(shift.z!=0)
  {
    bb_min.z = bb_min.z + shift.z * vol.m_bbox.Size().z/float(vol.m_nWholeGridRes);
    bb_max.z = bb_max.z + shift.z * vol.m_bbox.Size().z/float(vol.m_nWholeGridRes);
  }

  // save shift params in grid sdf data struct
  vol.m_shift = shift;

  // 2, Kernel functin. Initialization for GPU parallelization
//  dim3 blockDim(16,16);
//  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
//  KernRollingGridSdf<<<gridDim,blockDim>>>(bb_min, bb_max, shift);
//  GpuCheckErrors();


  // 3, copy array back
  int nNextResetSDFs[vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes];
  cudaMemcpyFromSymbol(nNextResetSDFs, g_NextResetSDFs, sizeof(g_NextResetSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  for(int i=0;i!=vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes;i++)
  {
    pNextInitSDFs[i] = nNextResetSDFs[i];

    nNextResetSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextResetSDFs,nNextResetSDFs,sizeof(nNextResetSDFs),0,cudaMemcpyHostToDevice);

  // for each grid sdf that need to be reset, free it.
  g_vol.FreeMemory();

  // reset
  for(int i=0;i!=vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes;i++)
  {
    if(nNextResetSDFs[i]==1)
    {
      vol.FreeMemoryByIndex(i);
    }
  }
}

}
