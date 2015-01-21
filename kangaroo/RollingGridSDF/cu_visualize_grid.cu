#include "cu_visualize_grid.h"

namespace roo
{
__device__ BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage>  g_vol_grid_smart;
__device__ BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>  g_vol_grid_gray;

__global__ void KernVisualizeGrid()
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //  const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol_grid_smart.m_d; ++z)
  {
    int nIndex = g_vol_grid_smart.ConvertLocalIndexToRealIndex(
          static_cast<int>(floorf(x/g_vol_grid_smart.m_nVolumeGridRes)),
          static_cast<int>(floorf(y/g_vol_grid_smart.m_nVolumeGridRes)),
          static_cast<int>(floorf(z/g_vol_grid_smart.m_nVolumeGridRes)) );

    if(g_vol_grid_smart.CheckIfBasicSDFActive(nIndex))
    {
      if(x <= 30 || x >= g_vol_grid_smart.m_w -30 ||
         y <= 30 || y >= g_vol_grid_smart.m_h -30 ||
         z <= 30 || z >= g_vol_grid_smart.m_d -30 )
      {
        g_vol_grid_gray(x,y,z) = 1;
      }
    }
  }
}

void VisualizeGrid(
    BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, TargetDevice, Manage> colorVol)
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol_grid_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_vol_grid_gray, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernVisualizeGrid<<<gridDim,blockDim>>>();
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol_grid_smart);
  colorVol.CopyFrom(g_vol_grid_gray);
  GpuCheckErrors();
}

}
