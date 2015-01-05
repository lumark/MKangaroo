#include "cu_rolling_sdf.h"
#include "BoundedVolumeGrid.h"
#include "kangaroo/MatUtils.h"
#include "kangaroo/launch_utils.h"

namespace roo
{
//////////////////////////////////////////////////////
/// Rolling SDF by partial reset
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
__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>        g_vol;
__device__ BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage>  g_vol_smart;
__device__ int                                                             g_NextResetSDFs[MAX_SUPPORT_GRID_NUM];

// =============================================================================
// Boxmin & boxmax define the box that is to be kept intact, rest will be cleared.
// This approach makes if conditions inside simpler.
// When we clean a grid sdf, we also need to free its memory.. This maybe a little
// bit expensive
// =============================================================================
__global__ void KernRollingGridSdf(
    float3                                boxmin,
    float3                                boxmax,
    float3                                shift)
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
      int nIndex = static_cast<int>(floorf(x/g_vol.m_nVolumeGridRes)) +
          g_vol.m_nGridRes_w * ( static_cast<int>(floorf(y/g_vol.m_nVolumeGridRes)) +
                                 g_vol.m_nGridRes_h * static_cast<int>(floorf(z/g_vol.m_nVolumeGridRes)) );

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
void RollingGridSdfCuda(
    int*                                  pNextInitSDFs,
    BoundedVolumeGrid<SDF_t>              vol,
    int3                                  shift)
{
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // 1, Compute the latest bounding box
  float3 bb_min = vol.m_bbox.boxmin, bb_max = vol.m_bbox.boxmax;

  if(shift.x!=0)
  {
    bb_min.x = bb_min.x + shift.x * vol.m_bbox.Size().x/float(vol.m_nGridRes_w);
    bb_max.x = bb_max.x + shift.x * vol.m_bbox.Size().x/float(vol.m_nGridRes_w);
  }

  if(shift.y!=0)
  {
    bb_min.y = bb_min.y + shift.y * vol.m_bbox.Size().y/float(vol.m_nGridRes_h);
    bb_max.y = bb_max.y + shift.y * vol.m_bbox.Size().y/float(vol.m_nGridRes_h);
  }

  if(shift.z!=0)
  {
    bb_min.z = bb_min.z + shift.z * vol.m_bbox.Size().z/float(vol.m_nGridRes_d);
    bb_max.z = bb_max.z + shift.z * vol.m_bbox.Size().z/float(vol.m_nGridRes_d);
  }

  // save shift params in grid sdf data struct
  vol.m_local_shift = shift;

  // 2, Kernel functin. Initialization for GPU parallelization
  //  dim3 blockDim(16,16);
  //  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  //  KernRollingGridSdf<<<gridDim,blockDim>>>(bb_min, bb_max, shift);
  //  GpuCheckErrors();


  // 3, copy array back
  int nNextResetSDFs[vol.GetTotalGridNum()];
  cudaMemcpyFromSymbol(nNextResetSDFs, g_NextResetSDFs, sizeof(g_NextResetSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    pNextInitSDFs[i] = nNextResetSDFs[i];

    nNextResetSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextResetSDFs,nNextResetSDFs,sizeof(nNextResetSDFs),0,cudaMemcpyHostToDevice);

  // for each grid sdf that need to be reset, free it.
  g_vol.FreeMemory();

  // reset
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    if(nNextResetSDFs[i]==1)
    {
      vol.FreeMemoryByIndex(i);
    }
  }
}

void RollingGridSdfCuda(
    int*                                  pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart>        vol,
    int3                                  shift)
{
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // 1, Compute the latest bounding box
  float3 bb_min = vol.m_bbox.boxmin, bb_max = vol.m_bbox.boxmax;

  if(shift.x!=0)
  {
    bb_min.x = bb_min.x + shift.x * vol.m_bbox.Size().x/float(vol.m_nGridRes_w);
    bb_max.x = bb_max.x + shift.x * vol.m_bbox.Size().x/float(vol.m_nGridRes_w);
  }

  if(shift.y!=0)
  {
    bb_min.y = bb_min.y + shift.y * vol.m_bbox.Size().y/float(vol.m_nGridRes_h);
    bb_max.y = bb_max.y + shift.y * vol.m_bbox.Size().y/float(vol.m_nGridRes_h);
  }

  if(shift.z!=0)
  {
    bb_min.z = bb_min.z + shift.z * vol.m_bbox.Size().z/float(vol.m_nGridRes_d);
    bb_max.z = bb_max.z + shift.z * vol.m_bbox.Size().z/float(vol.m_nGridRes_d);
  }

  // save shift params in grid sdf data struct
  vol.m_local_shift = shift;

  // 2, Kernel functin. Initialization for GPU parallelization
  //  dim3 blockDim(16,16);
  //  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  //  KernRollingGridSdf<<<gridDim,blockDim>>>(bb_min, bb_max, shift);
  //  GpuCheckErrors();

  // 3, copy array back
  int nNextResetSDFs[vol.GetTotalGridNum()];
  cudaMemcpyFromSymbol(nNextResetSDFs, g_NextResetSDFs, sizeof(g_NextResetSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    pNextInitSDFs[i] = nNextResetSDFs[i];

    nNextResetSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextResetSDFs,nNextResetSDFs,sizeof(nNextResetSDFs),0,cudaMemcpyHostToDevice);

  // for each grid sdf that need to be reset, free it.
  g_vol.FreeMemory();

  // reset
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    if(nNextResetSDFs[i]==1)
    {
      vol.FreeMemoryByIndex(i);
    }
  }
}

// =============================================================================
// raycast grid gray SDF
// =============================================================================
__device__ float3 g_positive_shift;
__device__ float3 g_negative_shift;
__global__ void KernDetectRollingSdfShift(
    Image<float>                                                  imgdepth,
    const Mat<float,3,4>                                          T_wc,
    float3                                                        T_wc_translate,
    ImageIntrinsics                                               K )
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if( u < imgdepth.w && v < imgdepth.h )
  {
    if(imgdepth(u,v) > 0)
    {
      const float3 ray_c = K.Unproject(u,v);
      const float3 ray_w = mulSO3(T_wc, ray_c);

      // check if the pixel is out of BB. shift is the change of % of poses in BB
      float3 shift_precentage = g_vol.GetShiftValue(ray_w, T_wc_translate);

      // if shift > 1: (the current bounding box (BB) cannot hold the new pixel)
      if(abs(shift_precentage.x) > 1.f)
      {
        if(shift_precentage.x > g_positive_shift.x)
        {
          g_positive_shift.x = shift_precentage.x;
        }
        else if(shift_precentage.x < g_negative_shift.x)
        {
          g_negative_shift.x = shift_precentage.x;
        }
      }

      // if shift > 1: (the current bounding box (BB) cannot hold the new pixel)
      if( abs(shift_precentage.y) > 1.f)
      {
        if(shift_precentage.y > g_positive_shift.y)
        {
          g_positive_shift.y = shift_precentage.y;
        }
        else if(shift_precentage.y < g_negative_shift.y)
        {
          g_negative_shift.y = shift_precentage.y;
        }
      }

      // if shift > 1: (the current bounding box (BB) cannot hold the new pixel)
      if( abs(shift_precentage.z) > 1.f)
      {
        if(shift_precentage.z > g_positive_shift.z)
        {
          g_positive_shift.z = shift_precentage.z;
        }
        else if(shift_precentage.z < g_negative_shift.z)
        {
          g_negative_shift.z = shift_precentage.z;
        }
      }
    }
  }
}

void RollingDetShift(
    float3&                                                       positive_shift,
    float3&                                                       negative_shift,
    Image<float>                                                  depth,
    const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
    const Mat<float,3,4>                                          T_wc,
    ImageIntrinsics                                               K)
{
  // load vol val to golbal memory
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // init the pose_shift to 0
  positive_shift = make_float3(0,0,0);
  negative_shift = make_float3(0,0,0);

  cudaMemcpyToSymbol(g_positive_shift,&positive_shift,sizeof(positive_shift),0,cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_negative_shift,&negative_shift,sizeof(negative_shift),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  printf("[cu_rolling_sdf.cu/RollingDetShift] camera translate (%f,%f,%f)\n",
         SE3Translation(T_wc).x,SE3Translation(T_wc).y, SE3Translation(T_wc).z);

  dim3 blockDim, gridDim;
  InitDimFromOutputImageOver(blockDim, gridDim, depth);
  KernDetectRollingSdfShift<<<gridDim,blockDim>>>(depth, T_wc, SE3Translation(T_wc), K);
  GpuCheckErrors();

  // copy camera shift back to host memory
  cudaMemcpyFromSymbol(&positive_shift,g_positive_shift,sizeof(positive_shift),0,cudaMemcpyDeviceToHost);
  cudaMemcpyFromSymbol(&negative_shift,g_negative_shift,sizeof(negative_shift),0,cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  g_vol.FreeMemory();
}

void RollingDetShift(
    float3&                                                             positive_shift,
    float3&                                                             negative_shift,
    Image<float>                                                        depth,
    const BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
    const Mat<float,3,4>                                                T_wc,
    ImageIntrinsics                                                     K)
{

  // load vol val to golbal memory
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // set shift to 0
  positive_shift = make_float3(0,0,0);
  negative_shift = make_float3(0,0,0);

  cudaMemcpyToSymbol(g_positive_shift,&positive_shift,sizeof(positive_shift),0,cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_negative_shift,&negative_shift,sizeof(negative_shift),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  printf("[cu_rolling_sdf.cu/RollingDetShift] camera translate (%0.3f,%0.3f,%0.3f);"
         " cur bbox min (%0.3f,%0.3f,%0.3f); bbox max (%0.3f,%0.3f,%0.3f)\n",
         SE3Translation(T_wc).x,SE3Translation(T_wc).y, SE3Translation(T_wc).z,
         vol.m_bbox.boxmin.x, vol.m_bbox.boxmin.y, vol.m_bbox.boxmin.z,
         vol.m_bbox.boxmax.x, vol.m_bbox.boxmax.y, vol.m_bbox.boxmax.z);

  dim3 blockDim, gridDim;
  InitDimFromOutputImageOver(blockDim, gridDim, depth);
  KernDetectRollingSdfShift<<<gridDim,blockDim>>>(depth, T_wc, SE3Translation(T_wc), K);
  GpuCheckErrors();

  cudaMemcpyFromSymbol(&positive_shift,g_positive_shift,sizeof(positive_shift),0,cudaMemcpyDeviceToHost);
  cudaMemcpyFromSymbol(&negative_shift,g_negative_shift,sizeof(negative_shift),0,cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  g_vol.FreeMemory();
}

}
