#include "cu_sdf_reset.h"
#include "kangaroo/MatUtils.h"
#include "kangaroo/launch_utils.h"

namespace roo
{
////////////////////////////////////////////////////////////////////////////////
///////////////////////////// For Grid SDF Fusion //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>  g_vol;
__device__ BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>  g_grayVol;

// have a large size of array to save index of grid sdf that need to init
__device__ int                            g_NextInitSDFs[MAX_SUPPORT_GRID_NUM];

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfInitGrayGrid(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        if(md>min_depth)
        {
          // normal value at image coordinate
          const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

          const float costheta = dot(mdn, P_c) / -length(P_c);
          const float sd = costheta * (md - vd);

          if(sd <= -trunc_dist)
          {
            // Further than truncation distance from surface
            // We do nothing.
          }
          // update SDF
          else
          {
            //        }else if(sd < 5*trunc_dist) {
            if(/*sd < 5*trunc_dist && */isfinite(md)  && costheta > mincostheta )
            {
              int nIndex = g_vol.GetIndex(int(floorf(x/g_vol.m_nVolumeGridRes)),
                                          int(floorf(y/g_vol.m_nVolumeGridRes)),
                                          int(floorf(z/g_vol.m_nVolumeGridRes)) );
              g_NextInitSDFs[nIndex] = 1;
            }
          }
        }
      }
    }
  }
}


void SDFInitGrayGrid(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> grayVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  if(vol.GetTotalGridNum()>MAX_SUPPORT_GRID_NUM)
  {
    printf("[SDFInitgrayGrid.cu] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  // load grid sdf to global memory. We do this instead of pass it via function
  // param because there is a size limit of the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &grayVol, sizeof(grayVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfInitGrayGrid<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, gray, T_iw, Krgb, trunc_dist, max_w, mincostheta, min_depth);
  GpuCheckErrors();

  //  printf("[SDFInitgrayGrid.cu] Finished kernel.\n");

  int nNextInitSDFs[MAX_SUPPORT_GRID_NUM];
  cudaMemcpyFromSymbol(nNextInitSDFs, g_NextInitSDFs, sizeof(g_NextInitSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  //  printf("[SDFInitgrayGrid.cu] Finished copy.\n");

  // copy array back
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    pNextInitSDFs[i] = nNextInitSDFs[i];
    nNextInitSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextInitSDFs,nNextInitSDFs,sizeof(nNextInitSDFs),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();

  //  free(nNextInitSDFs);

  //  printf("[SDFInitgrayGrid.cu] Finished.\n");
}

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
// the following must be used with SDFInitgrayGrid
__global__ void KernSdfFuseDirectGrayGrid(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        // normal value at image coordinate
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        const float w = costheta * 1.0f/vd;

        if(sd <= -trunc_dist)
        {
          // Further than truncation distance from surface
          // We do nothing.
        }
        // update SDF
        else
        {
          //        }else if(sd < 5*trunc_dist) {

          if(/*sd < 5*trunc_dist && */isfinite(md) && md> min_depth && costheta > mincostheta )
          {
            const SDF_t curvol = g_vol(x,y,z);

            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            sdf += curvol;
            sdf.LimitWeight(max_w);

            /// set val
            g_vol(x, y, z) = sdf;
            g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGrid(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGrayGrid<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth,
                                                  gray, T_iw, Krgb, trunc_dist,
                                                  max_w, mincostheta, min_depth);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
// the following must be used with SDFInitgrayGrid
__global__ void KernSdfFuseDirectGrayGridSafe(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        // normal value at image coordinate
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        const float w = costheta * 1.0f/vd;

        if(sd <= -trunc_dist)
        {
          // Further than truncation distance from surface
          // We do nothing.
        }
        // update SDF
        else
        {
          //        }else if(sd < 5*trunc_dist) {
          int nIndex = g_vol.GetIndex(int(floorf(x/g_vol.m_nVolumeGridRes)),
                                      int(floorf(y/g_vol.m_nVolumeGridRes)),
                                      int(floorf(z/g_vol.m_nVolumeGridRes)) );

          if(/*sd < 5*trunc_dist && */isfinite(md) && md>min_depth && costheta > mincostheta )
          {
            if(g_vol.CheckIfBasicSDFActive(nIndex)==true )
            {
              const SDF_t curvol = g_vol(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              /// set val
              g_vol(x, y, z) = sdf;
              g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
              //              printf("fuse:%f,", c);
            }
            else
            {
              printf("[KernSdfFuseDirectgrayGridSafe] warnning!!! skip %d,%d,%d when fusing!!!\n",
                     int(floorf(x/g_vol.m_nVolumeGridRes)),
                     int(floorf(y/g_vol.m_nVolumeGridRes)),
                     int(floorf(z/g_vol.m_nVolumeGridRes)) );
            }

          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGrayGridSafe<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth,
                                                      gray, T_iw, Krgb, trunc_dist,
                                                      max_w, mincostheta, min_depth);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion for certain index without consideing void (zero intensity) pixels
// notice that in this function, we check each voxel we have and see if we need
// to fuse any information into it. This is different from check each pixel we have
// and try to fuse it into the sdf.
__global__ void KernSdfFuseDirectgrayGridDesireIndex(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol.m_d; ++z)
  {
    int nIndex = g_vol.GetIndex(int(floorf(x/g_vol.m_nVolumeGridRes)),
                                int(floorf(y/g_vol.m_nVolumeGridRes)),
                                int(floorf(z/g_vol.m_nVolumeGridRes)) );

    if(g_NextInitSDFs[nIndex] == 1)
    {
      // See if this voxel is possible to be in the image boundary
      // Get voxel position in certain radius in world coordinate (good)
      const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

      // Get voxel position in camera coordinate (good)
      const float3 P_c = T_cw * P_w;

      // Project a 3D voxel point to 2D depth an gray image coordinate
      const float2 p_c = Kdepth.Project(P_c);

      const float3 P_i = T_iw * P_w;
      const float2 p_i = Krgb.Project(P_i);

      // If the voxel is in image coordinate (inside of image boundary), then we
      // see if we should fuse this voxel
      if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
      {
        // prepare to fuse a gray pixel into this voxel
        const float c =  gray.GetBilinear<float>(p_i);

        // discard pixel value equals 0
        if(c>0)
        {
          // depth value at camera coorniate
          const float vd   = P_c.z;

          // depth value at image coordinate
          const float md   = depth.GetBilinear<float>(p_c);

          // normal value at image coordinate
          const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

          const float costheta = dot(mdn, P_c) / -length(P_c);
          const float sd = costheta * (md - vd);
          const float w = costheta * 1.0f/vd;

          if(sd <= -trunc_dist)
          {
            // Further than truncation distance from surface
            // We do nothing.
          }
          // update SDF
          else
          {
            //        }else if(sd < 5*trunc_dist) {

            if(/*sd < 5*trunc_dist && */isfinite(md) && md>min_depth && costheta > mincostheta )
            {
              const SDF_t curvol = g_vol(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              // set val
              g_vol(x, y, z) = sdf;

              if(bWeight == true)
              {
                g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
              }
              else
              {
                if(g_grayVol(x,y,z)>0)
                {
                  printf("skipFuse;");
                }
                else
                {
                  g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
                }
              }

            }
          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGridDesireIndex(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  if(vol.GetTotalGridNum()>MAX_SUPPORT_GRID_NUM)
  {
    printf("[SdfFuseDirectgrayGridAutoInit] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // copy array back
  int nNextInitSDFs[MAX_SUPPORT_GRID_NUM];

  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    nNextInitSDFs[i] = pNextInitSDFs[i] ;
  }

  cudaMemcpyToSymbol(g_NextInitSDFs, nNextInitSDFs, sizeof(nNextInitSDFs), 0, cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectgrayGridDesireIndex<<<gridDim,blockDim>>>(depth, norm, T_cw,
                                                             Kdepth, gray, T_iw,
                                                             Krgb, trunc_dist,
                                                             max_w, mincostheta,
                                                             min_depth, bWeight);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();

  printf("[SdfFuseDirectgrayGridDesireIndex/cu] Finished all.\n");
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// the following do Grid SDF fusion and also mark voxels that cannot be fused in
// due to uninitialized Grid SDF.
__global__ void KernSdfFuseDirectGrayGridAutoInit(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        // normal value at image coordinate
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        const float w = costheta * 1.0f/vd;

        if(sd <= -trunc_dist)
        {
          // Further than truncation distance from surface
          // We do nothing.
        }
        // update SDF
        else
        {
          //        }else if(sd < 5*trunc_dist) {

          if(/*sd < 5*trunc_dist && */isfinite(md) && md>min_depth && costheta > mincostheta )
          {
            int nIndex = g_vol.GetIndex(int(floorf(x/g_vol.m_nVolumeGridRes)),
                                        int(floorf(y/g_vol.m_nVolumeGridRes)),
                                        int(floorf(z/g_vol.m_nVolumeGridRes)) );

            if(g_vol.CheckIfBasicSDFActive(nIndex) == true)
            {
              const SDF_t curvol = g_vol(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              /// set val
              g_vol(x, y, z) = sdf;

              if(bWeight == true)
              {
                g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
              }
              else
              {
                if(g_grayVol(x,y,z)==0.5)
                {
                  g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
                }
              }

            }
            else
            {
              g_NextInitSDFs[nIndex] = 1;
            }

          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGridAutoInit(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  if(vol.GetTotalGridNum()>MAX_SUPPORT_GRID_NUM)
  {
    printf("[SdfFuseDirectgrayGridAutoInit] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGrayGridAutoInit<<<gridDim,blockDim>>>(depth, norm, T_cw,
                                                          Kdepth, gray, T_iw,
                                                          Krgb, trunc_dist,
                                                          max_w, mincostheta,
                                                          min_depth, bWeight);
  GpuCheckErrors();
  // check if need to init new grid sdf
  int nNextInitSDFs[MAX_SUPPORT_GRID_NUM];
  cudaMemcpyFromSymbol(nNextInitSDFs, g_NextInitSDFs, sizeof(g_NextInitSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  // copy array back
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    pNextInitSDFs[i] = nNextInitSDFs[i];
    nNextInitSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextInitSDFs,nNextInitSDFs,sizeof(nNextInitSDFs),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();
}


// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
//__global__ void KernSdfFuseDirectgrayGrid(float* depth, float4* normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
//                                          Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
//                                          float trunc_dist, float max_w, float mincostheta
//                                          )
//{
//  const int x = blockIdx.x*blockDim.x + threadIdx.x;
//  const int y = blockIdx.y*blockDim.y + threadIdx.y;

//  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

//  // For each voxel (x,y,z) we have in a bounded volume
//  for(int z=0; z < g_vol.m_d; ++z)
//  {
//    // See if this voxel is possible to be in the image boundary
//    // Get voxel position in certain radius in world coordinate (good)
//    const float3 P_w = g_vol.VoxelPositionInUnits(x,y,z);

//    // Get voxel position in camera coordinate (good)
//    const float3 P_c = T_cw * P_w;

//    // Project a 3D voxel point to 2D depth an gray image coordinate
//    const float2 p_c = Kdepth.Project(P_c);

//    const float3 P_i = T_iw * P_w;
//    const float2 p_i = Krgb.Project(P_i);

//    // If the voxel is in image coordinate (inside of image boundary), then we
//    // see if we should fuse this voxel
//    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
//    {
//      // prepare to fuse a gray pixel into this voxel
//      const float c =  gray.GetBilinear<float>(p_i);

//      // discard pixel value equals 0
//      if(c!=0)
//      {
//        // depth value at camera coorniate
//        const float vd   = P_c.z;

//        // depth value at image coordinate
//        const float md   = depth.GetBilinear<float>(p_c);

//        // normal value at image coordinate
//        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

//        const float costheta = dot(mdn, P_c) / -length(P_c);
//        const float sd = costheta * (md - vd);
//        const float w = costheta * 1.0f/vd;

//        if(sd <= -trunc_dist)
//        {
//          // Further than truncation distance from surface
//          // We do nothing.
//        }
//        // update SDF
//        else
//        {
//          //        }else if(sd < 5*trunc_dist) {

//          /// here 0.5 is for kinect sensor
//          if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta )
//          {
//            //            printf("fuse:x%d,y%d,z%d",x,y,z);

////            printf("shift:%d",g_vol.m_shift.x);

//            const SDF_t curvol = g_vol(x,y,z);

//            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
//            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
//            sdf += curvol;
//            sdf.LimitWeight(max_w);

//            /// set val
//            g_vol(x, y, z) = sdf;
//            g_colorVol(x,y,z) = (w*c + g_colorVol(x,y,z) * curvol.w) / (w + curvol.w);
//          }
//        }
//      }
//    }
//  }
//}


//void SdfFuseDirectgrayGrid(
//    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
//    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
//    float* depth, float4* norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
//    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
//    float trunc_dist, float max_w, float mincostheta
//    )
//{
//  /// load grid sdf to golbal memory. We do this because there is a size limit of
//  // the parameters that we can send the the kernel function.
//  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
//  cudaMemcpyToSymbol(g_colorVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
//  GpuCheckErrors();

//  // launch kernel for SDF fusion
//  dim3 blockDim(32,32);
//  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
//  KernSdfFuseDirectgrayGrid<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, gray, T_iw, Krgb, trunc_dist, max_w, mincostheta);
//  GpuCheckErrors();

//  // copy data back after launch the kernel
//  vol.CopyFrom(g_vol);
//  colorVol.CopyFrom(g_colorVol);
//  GpuCheckErrors();

//  // cuda free memory
//  g_vol.FreeMemory();
//  g_colorVol.FreeMemory();
//  GpuCheckErrors();
//}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//////////////////////// DO SDF Fusion with smart SDF //////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

__device__ BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage>  g_vol_smart;

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfInitGrayGridSmart(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol_smart.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol_smart.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        if(md>min_depth)
        {
          // normal value at image coordinate
          const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

          const float costheta = dot(mdn, P_c) / -length(P_c);
          const float sd = costheta * (md - vd);

          if(sd <= -trunc_dist)
          {
            // Further than truncation distance from surface
            // We do nothing.
          }
          // update SDF
          else
          {
            //        }else if(sd < 5*trunc_dist) {
            if(/*sd < 5*trunc_dist && */isfinite(md)  && costheta > mincostheta )
            {
              int nIndex = g_vol_smart.GetIndex(int(floorf(x/g_vol_smart.m_nVolumeGridRes)),
                                                int(floorf(y/g_vol_smart.m_nVolumeGridRes)),
                                                int(floorf(z/g_vol_smart.m_nVolumeGridRes)) );
              g_NextInitSDFs[nIndex] = 1;
            }
          }
        }
      }
    }
  }
}


void SDFInitGrayGrid(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> grayVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  if(vol.GetTotalGridNum()>MAX_SUPPORT_GRID_NUM)
  {
    printf("[SDFInitgrayGrid.cu] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  // load grid sdf to global memory. We do this instead of pass it via function
  // param because there is a size limit of the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &grayVol, sizeof(grayVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfInitGrayGridSmart<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, gray,
                                                 T_iw, Krgb, trunc_dist, max_w,
                                                 mincostheta, min_depth);
  GpuCheckErrors();

  //  printf("[SDFInitgrayGrid.cu] Finished kernel.\n");

  int nNextInitSDFs[MAX_SUPPORT_GRID_NUM];
  cudaMemcpyFromSymbol(nNextInitSDFs, g_NextInitSDFs, sizeof(g_NextInitSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  //  printf("[SDFInitgrayGrid.cu] Finished copy.\n");

  // copy array back
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    pNextInitSDFs[i] = nNextInitSDFs[i];
    nNextInitSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextInitSDFs,nNextInitSDFs,sizeof(nNextInitSDFs),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // cuda free memory
  g_vol_smart.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();

  //  free(nNextInitSDFs);
}

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
// the following must be used with SDFInitgrayGrid
__global__ void KernSdfFuseDirectGrayGridSmart(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float min_depth, float mincostheta
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol_smart.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol_smart.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        // normal value at image coordinate
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        const float w = costheta * 1.0f/vd;

        if(sd <= -trunc_dist)
        {
          // Further than truncation distance from surface
          // We do nothing.
        }
        // update SDF
        else
        {
          //        }else if(sd < 5*trunc_dist) {

          if(/*sd < 5*trunc_dist && */isfinite(md) && md> min_depth && costheta > mincostheta )
          {
            const SDF_t_Smart curvol = g_vol_smart(x,y,z);

            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t_Smart sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            sdf += curvol;
            sdf.LimitWeight(max_w);

            /// set val
            g_vol_smart(x, y, z) = sdf;
            g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGrid(
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGrayGridSmart<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth,
                                                       gray, T_iw, Krgb, trunc_dist,
                                                       max_w, mincostheta, min_depth);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol_smart);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol_smart.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
// the following must be used with SDFInitgrayGrid
__global__ void KernSdfFuseDirectGrayGridSafeSmart(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol_smart.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol_smart.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        // normal value at image coordinate
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        const float w = costheta * 1.0f/vd;

        if(sd <= -trunc_dist)
        {
          // Further than truncation distance from surface
          // We do nothing.
        }
        // update SDF
        else
        {
          //        }else if(sd < 5*trunc_dist) {
          int nIndex = g_vol_smart.GetIndex(int(floorf(x/g_vol_smart.m_nVolumeGridRes)),
                                            int(floorf(y/g_vol_smart.m_nVolumeGridRes)),
                                            int(floorf(z/g_vol_smart.m_nVolumeGridRes)) );

          if(/*sd < 5*trunc_dist && */isfinite(md) && md> min_depth && costheta > mincostheta )
          {
            if(g_vol_smart.CheckIfBasicSDFActive(nIndex)==true )
            {
              const SDF_t_Smart curvol = g_vol_smart(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t_Smart sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              /// set val
              g_vol_smart(x, y, z) = sdf;
              g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
              //              printf("fuse:%f,", c);
            }
            else
            {
              printf("[KernSdfFuseDirectgrayGridSafe] warnning!!! skip %d,%d,%d when fusing!!!\n",
                     int(floorf(x/g_vol_smart.m_nVolumeGridRes)),
                     int(floorf(y/g_vol_smart.m_nVolumeGridRes)),
                     int(floorf(z/g_vol_smart.m_nVolumeGridRes)) );
            }

          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    )
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGrayGridSafeSmart<<<gridDim,blockDim>>>(depth, norm, T_cw,
                                                           Kdepth, gray, T_iw,
                                                           Krgb, trunc_dist,
                                                           max_w, mincostheta,
                                                           min_depth);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol_smart);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol_smart.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion for certain index without consideing void (zero intensity) pixels
// notice that in this function, we check each voxel we have and see if we need
// to fuse any information into it. This is different from check each pixel we have
// and try to fuse it into the sdf.
__global__ void KernSdfFuseDirectgrayGridDesireIndexSmart(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol_smart.m_d; ++z)
  {
    int nIndex = g_vol_smart.GetIndex(int(floorf(x/g_vol_smart.m_nVolumeGridRes)),
                                      int(floorf(y/g_vol_smart.m_nVolumeGridRes)),
                                      int(floorf(z/g_vol_smart.m_nVolumeGridRes)) );

    if(g_NextInitSDFs[nIndex] == 1)
    {
      // See if this voxel is possible to be in the image boundary
      // Get voxel position in certain radius in world coordinate (good)
      const float3 P_w = g_vol_smart.VoxelPositionInUnits(x,y,z);

      // Get voxel position in camera coordinate (good)
      const float3 P_c = T_cw * P_w;

      // Project a 3D voxel point to 2D depth an gray image coordinate
      const float2 p_c = Kdepth.Project(P_c);

      const float3 P_i = T_iw * P_w;
      const float2 p_i = Krgb.Project(P_i);

      // If the voxel is in image coordinate (inside of image boundary), then we
      // see if we should fuse this voxel
      if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
      {
        // prepare to fuse a gray pixel into this voxel
        const float c =  gray.GetBilinear<float>(p_i);

        // discard pixel value equals 0
        if(c>0)
        {
          // depth value at camera coorniate
          const float vd   = P_c.z;

          // depth value at image coordinate
          const float md   = depth.GetBilinear<float>(p_c);

          // normal value at image coordinate
          const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

          const float costheta = dot(mdn, P_c) / -length(P_c);
          const float sd = costheta * (md - vd);
          const float w = costheta * 1.0f/vd;

          if(sd <= -trunc_dist)
          {
            // Further than truncation distance from surface
            // We do nothing.
          }
          // update SDF
          else
          {
            //        }else if(sd < 5*trunc_dist) {

            if(/*sd < 5*trunc_dist && */isfinite(md) && md>min_depth && costheta > mincostheta )
            {
              const SDF_t_Smart curvol = g_vol_smart(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t_Smart sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              // set val
              g_vol_smart(x, y, z) = sdf;

              if(bWeight == true)
              {
                g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
              }
              else
              {
                if(g_grayVol(x,y,z)>0)
                {
                  printf("skipFuse;");
                }
                else
                {
                  g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
                }
              }

            }
          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGridDesireIndex(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  if(vol.GetTotalGridNum()>MAX_SUPPORT_GRID_NUM)
  {
    printf("[SdfFuseDirectgrayGridAutoInit] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // copy array back
  int nNextInitSDFs[MAX_SUPPORT_GRID_NUM];

  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    nNextInitSDFs[i] = pNextInitSDFs[i] ;
  }

  cudaMemcpyToSymbol(g_NextInitSDFs, nNextInitSDFs, sizeof(nNextInitSDFs), 0, cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectgrayGridDesireIndexSmart<<<gridDim,blockDim>>>(depth, norm, T_cw,
                                                                  Kdepth, gray, T_iw,
                                                                  Krgb, trunc_dist,
                                                                  max_w, mincostheta,
                                                                  min_depth, bWeight);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol_smart);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol_smart.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();

  printf("[SdfFuseDirectgrayGridDesireIndex/cu] Finished all.\n");
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// the following do Grid SDF fusion and also mark voxels that cannot be fused in
// due to uninitialized Grid SDF.
__global__ void KernSdfFuseDirectGrayGridAutoInitSmart(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_vol_smart.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_vol_smart.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate (good)
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an gray image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && gray.InBounds(p_i,2) )
    {
      // prepare to fuse a gray pixel into this voxel
      const float c =  gray.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        // normal value at image coordinate
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        const float w = costheta * 1.0f/vd;

        if(sd <= -trunc_dist)
        {
          // Further than truncation distance from surface
          // We do nothing.
        }
        // update SDF
        else
        {
          //        }else if(sd < 5*trunc_dist) {

          if(/*sd < 5*trunc_dist && */isfinite(md) && md> min_depth && costheta > mincostheta )
          {
            int nIndex = g_vol_smart.GetIndex(int(floorf(x/g_vol_smart.m_nVolumeGridRes)),
                                              int(floorf(y/g_vol_smart.m_nVolumeGridRes)),
                                              int(floorf(z/g_vol_smart.m_nVolumeGridRes)) );

            if(g_vol_smart.CheckIfBasicSDFActive(nIndex) == true)
            {
              const SDF_t_Smart curvol = g_vol_smart(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t_Smart sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              /// set val
              g_vol_smart(x, y, z) = sdf;

              if(bWeight == true)
              {
                g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
              }
              else
              {
                if(g_grayVol(x,y,z) == 0.5)
                {
                  g_grayVol(x,y,z) = (w*c + g_grayVol(x,y,z) * curvol.w) / (w + curvol.w);
                }
              }
            }
            else
            {
              g_NextInitSDFs[nIndex] = 1;
            }

          }
        }
      }
    }
  }
}


void SdfFuseDirectGrayGridAutoInit(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight
    )
{
  if(vol.GetTotalGridNum()>MAX_SUPPORT_GRID_NUM)
  {
    printf("[SdfFuseDirectgrayGridAutoInit] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGrayGridAutoInitSmart<<<gridDim,blockDim>>>(depth, norm, T_cw,
                                                               Kdepth, gray, T_iw,
                                                               Krgb, trunc_dist,
                                                               max_w, mincostheta,
                                                               min_depth, bWeight);
  GpuCheckErrors();

  // check if need to init new grid sdf
  int nNextInitSDFs[MAX_SUPPORT_GRID_NUM];
  cudaMemcpyFromSymbol(nNextInitSDFs, g_NextInitSDFs, sizeof(g_NextInitSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  // copy array back
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    pNextInitSDFs[i] = nNextInitSDFs[i];
    nNextInitSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextInitSDFs,nNextInitSDFs,sizeof(nNextInitSDFs),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol_smart);
  colorVol.CopyFrom(g_grayVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol_smart.FreeMemory();
  g_grayVol.FreeMemory();
  GpuCheckErrors();
}


}
