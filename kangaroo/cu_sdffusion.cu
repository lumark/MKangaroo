#include "cu_sdffusion.h"

#include "MatUtils.h"
#include "launch_utils.h"

namespace roo
{

//////////////////////////////////////////////////////
// Truncated SDF Fusion
// KinectFusion: Real-Time Dense Surface Mapping and Tracking, Newcombe et. al.
// http://www.doc.ic.ac.uk/~rnewcomb/
//////////////////////////////////////////////////////

__global__ void KernSdfFuse(BoundedVolume<SDF_t> vol, Image<float> depth,
                            Image<float4> normals, Mat<float,3,4> T_cw,
                            ImageIntrinsics K, float trunc_dist, float max_w, float mincostheta )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;
  const int z = blockIdx.z*blockDim.z + threadIdx.z;

  const float3 P_w = vol.VoxelPositionInUnits(x,y,z);
  const float3 P_c = T_cw * P_w;
  const float2 p_c = K.Project(P_c);

  if( depth.InBounds(p_c, 2) )
  {
    const float vd = P_c.z;
    //        const float md = depth.GetNearestNeighbour(p_c);
    //        const float3 mdn = make_float3(normals.GetNearestNeighbour(p_c));

    const float md = depth.GetBilinear<float>(p_c);
    const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

    const float costheta = dot(mdn, P_c) / -length(P_c);
    const float sd = costheta * (md - vd);
    const float w = costheta * 1.0f/vd;

    if(sd <= -trunc_dist) {
      // Further than truncation distance from surface
      // We do nothing.
    }else{
      //        }else if(sd < 5*trunc_dist) {
      if(isfinite(md) && isfinite(w) && costheta > mincostheta ) {
        SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
        sdf += vol(x,y,z);
        //                sdf.Clamp(-trunc_dist, trunc_dist);
        sdf.LimitWeight(max_w);
        vol(x,y,z) = sdf;
      }
    }
  }
}

void SdfFuse(BoundedVolume<SDF_t> vol, Image<float> depth, Image<float4> norm,
             Mat<float,3,4> T_cw, ImageIntrinsics K, float trunc_dist, float max_w, float mincostheta)
{
  dim3 blockDim(8,8,8);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);
  KernSdfFuse<<<gridDim,blockDim>>>(vol, depth, norm, T_cw, K, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();
}

//////////////////////////////////////////////////////
// Grey Truncated SDF Fusion
// Similar extension to KinectFusion as described by:
// Robust Tracking for Real-Time Dense RGB-D Mapping with Kintinous
// Whelan et. al.
//////////////////////////////////////////////////////

__global__ void KernSdfFuse(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics K,
    Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
    float trunc_dist, float max_w, float mincostheta
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;
  for(int z=0; z < vol.d; ++z) {
    const float3 P_w = vol.VoxelPositionInUnits(x,y,z);
    const float3 P_c = T_cw * P_w;
    const float2 p_c = K.Project(P_c);
    const float3 P_i = T_iw * P_w;
    const float2 p_i = Kimg.Project(P_i);

    if( depth.InBounds(p_c, 2) && img.InBounds(p_i,2) )
    {
      const float vd = P_c.z;
      const float md = depth.GetBilinear<float>(p_c);
      const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));
      const float c = ConvertPixel<float,float3>( img.GetBilinear<float3>(p_i) ) / 255.0;

      const float costheta = dot(mdn, P_c) / -length(P_c);
      const float sd = costheta * (md - vd);
      const float w = costheta * 1.0f/vd;

      if(sd <= -trunc_dist) {
        // Further than truncation distance from surface
        // We do nothing.
      }else{
        //        }else if(sd < 5*trunc_dist) {
        if(isfinite(md) && isfinite(w) && costheta > mincostheta ) {
          const SDF_t curvol = vol(x,y,z);
          SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
          sdf += curvol;
          sdf.LimitWeight(max_w);
          vol(x,y,z) = sdf;
          colorVol(x,y,z) = (w*c + colorVol(x,y,z) * curvol.w) / (w + curvol.w);
        }
      }
    }
  }
}

void SdfFuse(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K,
    Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
    float trunc_dist, float max_w, float mincostheta
    )
{
  dim3 blockDim(16,16);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y);
  KernSdfFuse<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, K, img, T_iw, Kimg, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();
}


//--the following add by luma-----------------------------------------------------------------------------------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfFuseDirectGrey(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < vol.d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate
    const float3 P_w = vol.VoxelPositionInUnits(x,y,z);

    // Get voxel position in camera coordinate
    const float3 P_c = T_cw * P_w;

    // Project a 3D voxel point to 2D depth an grey image coordinate
    const float2 p_c = Kdepth.Project(P_c);
    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
    {
      // prepare to fuse a grey pixel into this voxel
      const float c =  grey.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c!=0)
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

          /// here 0.5 is for kinect sensor
          if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta )
          {
            //            printf("md %f,", md);
            const SDF_t curvol = vol(x,y,z);
            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            sdf += curvol;
            sdf.LimitWeight(max_w);
            vol(x,y,z) = sdf;
            colorVol(x,y,z) = (w*c + colorVol(x,y,z) * curvol.w) / (w + curvol.w);
          }
        }
      }
    }
  }
}

void SdfFuseDirectGrey(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    ) {
  dim3 blockDim(16,16);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y);
  KernSdfFuseDirectGrey<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// For Grid SDF Fusion /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>  g_vol;
__device__ BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>  g_colorVol;

// have a large size of array to save index of grid sdf that need to init
__device__ int                                                       g_NextInitSDFs[102400];

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfInitGreyGrid(Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                                    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                                    float trunc_dist, float max_w, float mincostheta
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

    // Project a 3D voxel point to 2D depth an grey image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
    {
      // prepare to fuse a grey pixel into this voxel
      const float c =  grey.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c>0)
      {
        // depth value at camera coorniate
        const float vd   = P_c.z;

        // depth value at image coordinate
        const float md   = depth.GetBilinear<float>(p_c);

        if(md>0.5)
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
            /// here 0.5 is for kinect sensor
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


void SDFInitGreyGrid( int* pNextInitSDFs,
                      BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                      BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                      Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                      Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                      float trunc_dist, float max_w, float mincostheta
                      )
{
  if(vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes>102400)
  {
    printf("[SDFInitGreyGrid.cu] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  // load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_colorVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfInitGreyGrid<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();

  //  printf("[SDFInitGreyGrid.cu] Finished kernel.\n");

  int nNextInitSDFs[102400];
  cudaMemcpyFromSymbol(nNextInitSDFs, g_NextInitSDFs, sizeof(g_NextInitSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  //  printf("[SDFInitGreyGrid.cu] Finished copy.\n");

  // copy array back
  for(int i=0;i!=vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes;i++)
  {
    pNextInitSDFs[i] = nNextInitSDFs[i];
    nNextInitSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextInitSDFs,nNextInitSDFs,sizeof(nNextInitSDFs),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_colorVol.FreeMemory();
  GpuCheckErrors();

  //  free(nNextInitSDFs);

  //  printf("[SDFInitGreyGrid.cu] Finished.\n");
}

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
// the following must be used with SDFInitGreyGrid
__global__ void KernSdfFuseDirectGreyGrid(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
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

    // Project a 3D voxel point to 2D depth an grey image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
    {
      // prepare to fuse a grey pixel into this voxel
      const float c =  grey.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c!=0)
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

          /// here 0.5 is for kinect sensor
          if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta )
          {
            const SDF_t curvol = g_vol(x,y,z);

            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            sdf += curvol;
            sdf.LimitWeight(max_w);

            /// set val
            g_vol(x, y, z) = sdf;
            g_colorVol(x,y,z) = (w*c + g_colorVol(x,y,z) * curvol.w) / (w + curvol.w);
          }
        }
      }
    }
  }
}


void SdfFuseDirectGreyGrid(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    )
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_colorVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGreyGrid<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_colorVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_colorVol.FreeMemory();
  GpuCheckErrors();
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
// the following must be used with SDFInitGreyGrid
__global__ void KernSdfFuseDirectGreyGridSafe(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
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

    // Project a 3D voxel point to 2D depth an grey image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
    {
      // prepare to fuse a grey pixel into this voxel
      const float c =  grey.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c!=0)
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

          /// here 0.5 is for kinect sensor
          int nIndex = g_vol.GetIndex(int(floorf(x/g_vol.m_nVolumeGridRes)),
                                      int(floorf(y/g_vol.m_nVolumeGridRes)),
                                      int(floorf(z/g_vol.m_nVolumeGridRes)) );

          if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta && g_vol.CheckIfBasicSDFActive(nIndex)==true )
          {
            const SDF_t curvol = g_vol(x,y,z);

            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            sdf += curvol;
            sdf.LimitWeight(max_w);

            /// set val
            g_vol(x, y, z) = sdf;
            g_colorVol(x,y,z) = (w*c + g_colorVol(x,y,z) * curvol.w) / (w + curvol.w);
          }
        }
      }
    }
  }
}


void SdfFuseDirectGreyGridSafe(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    )
{
  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_colorVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGreyGridSafe<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_colorVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_colorVol.FreeMemory();
  GpuCheckErrors();
}



// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion for certain index without consideing void (zero intensity) pixels
// notice that in this function, we check each voxel we have and see if we need
// to fuse any information into it. This is different from check each pixel we have
// and try to fuse it into the sdf.
__global__ void KernSdfFuseDirectGreyGridDesireIndex(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
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

      // Project a 3D voxel point to 2D depth an grey image coordinate
      const float2 p_c = Kdepth.Project(P_c);

      const float3 P_i = T_iw * P_w;
      const float2 p_i = Krgb.Project(P_i);

      // If the voxel is in image coordinate (inside of image boundary), then we
      // see if we should fuse this voxel
      if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
      {
        // prepare to fuse a grey pixel into this voxel
        const float c =  grey.GetBilinear<float>(p_i);

        // discard pixel value equals 0
        if(c!=0)
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

            /// here 0.5 is for kinect sensor
            if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta )
            {
              const SDF_t curvol = g_vol(x,y,z);

              // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
              SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
              sdf += curvol;
              sdf.LimitWeight(max_w);

              /// set val
              g_vol(x, y, z) = sdf;
              g_colorVol(x,y,z) = (w*c + g_colorVol(x,y,z) * curvol.w) / (w + curvol.w);
            }
          }
        }
      }
    }
  }
}


void SdfFuseDirectGreyGridDesireIndex(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    )
{
  if(vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes>102400)
  {
    printf("[SdfFuseDirectGreyGridAutoInit] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_colorVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // copy array back
  int nNextInitSDFs[102400];

  for(int i=0;i!=vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes;i++)
  {
    nNextInitSDFs[i] = pNextInitSDFs[i] ;
  }

  cudaMemcpyToSymbol(g_NextInitSDFs, nNextInitSDFs, sizeof(nNextInitSDFs), 0, cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGreyGridDesireIndex<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_colorVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_colorVol.FreeMemory();
  GpuCheckErrors();

  printf("[SdfFuseDirectGreyGridDesireIndex/cu] Finished all.\n");
}

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// the following do Grid SDF fusion and use mark voxels that cannot be fused in
// due to uninitialized Grid SDF.
__global__ void KernSdfFuseDirectGreyGridAutoInit(
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
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

    // Project a 3D voxel point to 2D depth an grey image coordinate
    const float2 p_c = Kdepth.Project(P_c);

    const float3 P_i = T_iw * P_w;
    const float2 p_i = Krgb.Project(P_i);

    // If the voxel is in image coordinate (inside of image boundary), then we
    // see if we should fuse this voxel
    if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
    {
      // prepare to fuse a grey pixel into this voxel
      const float c =  grey.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c!=0)
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

          /// here 0.5 is for kinect sensor
          if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta )
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
              g_colorVol(x,y,z) = (w*c + g_colorVol(x,y,z) * curvol.w) / (w + curvol.w);
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


void SdfFuseDirectGreyGridAutoInit(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    )
{
  if(vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes>102400)
  {
    printf("[SdfFuseDirectGreyGridAutoInit] Fatal Error! Array size overflow!\n");
    exit(-1);
  }

  /// load grid sdf to golbal memory. We do this because there is a size limit of
  // the parameters that we can send the the kernel function.
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_colorVol, &colorVol, sizeof(colorVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel for SDF fusion
  dim3 blockDim(32,32);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGreyGridAutoInit<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();

  // check if need to init new grid sdf
  int nNextInitSDFs[102400];
  cudaMemcpyFromSymbol(nNextInitSDFs, g_NextInitSDFs, sizeof(g_NextInitSDFs), 0, cudaMemcpyDeviceToHost);
  GpuCheckErrors();

  // copy array back
  for(int i=0;i!=vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes;i++)
  {
    pNextInitSDFs[i] = nNextInitSDFs[i];
    nNextInitSDFs[i] = 0;
  }

  // reset index
  cudaMemcpyToSymbol(g_NextInitSDFs,nNextInitSDFs,sizeof(nNextInitSDFs),0,cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // copy data back after launch the kernel
  vol.CopyFrom(g_vol);
  colorVol.CopyFrom(g_colorVol);
  GpuCheckErrors();

  // cuda free memory
  g_vol.FreeMemory();
  g_colorVol.FreeMemory();
  GpuCheckErrors();
}


// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
//__global__ void KernSdfFuseDirectGreyGrid(float* depth, float4* normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
//                                          Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
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

//    // Project a 3D voxel point to 2D depth an grey image coordinate
//    const float2 p_c = Kdepth.Project(P_c);

//    const float3 P_i = T_iw * P_w;
//    const float2 p_i = Krgb.Project(P_i);

//    // If the voxel is in image coordinate (inside of image boundary), then we
//    // see if we should fuse this voxel
//    if( depth.InBounds(p_c, 2) && grey.InBounds(p_i,2) )
//    {
//      // prepare to fuse a grey pixel into this voxel
//      const float c =  grey.GetBilinear<float>(p_i);

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


//void SdfFuseDirectGreyGrid(
//    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
//    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
//    float* depth, float4* norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
//    Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
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
//  KernSdfFuseDirectGreyGrid<<<gridDim,blockDim>>>(depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
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


// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfFuseColor(
    BoundedVolume<SDF_t> vol, BoundedVolume<uchar3> colorVol,
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics K,
    Image<float> img, Image<uchar3> Imgrgb, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
    float trunc_dist, float max_w, float mincostheta
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;
  for(int z=0; z < vol.d; ++z)
  {
    const float3 P_w = vol.VoxelPositionInUnits(x,y,z);
    const float3 P_c = T_cw * P_w;
    const float2 p_c = K.Project(P_c);
    const float3 P_i = T_iw * P_w;
    const float2 p_i = Kimg.Project(P_i);

    if( depth.InBounds(p_c, 2) && img.InBounds(p_i,2) )
    {
      const float c =  img.GetBilinear<float>(p_i);

      // discard pixel value equals 0
      if(c!=0)
      {
        const float vd = P_c.z;
        const float md = depth.GetBilinear<float>(p_c);
        const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));

        const float costheta = dot(mdn, P_c) / -length(P_c);
        const float sd = costheta * (md - vd);
        //                const float w = 1;
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
          if(/*sd < 5*trunc_dist && */isfinite(md) && md>0.5 && costheta > mincostheta )
          {
            const SDF_t curvol = vol(x,y,z);
            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            sdf += curvol;
            sdf.LimitWeight(max_w);
            vol(x,y,z) = sdf;

            //            printf("(u,v)=(%d,%d),(r,g,b)=(%d,%d,%d),(x,y,z)=(%d,%d,%d)",int(p_i.x),int(p_i.y),
            //                   int(Imgrgb.Get(int(p_i.x),int(p_i.y)).x),
            //                   int(Imgrgb.Get(int(p_i.x),int(p_i.y)).y),
            //                   int(Imgrgb.Get(int(p_i.x),int(p_i.y)).z),
            //                   x,y,z);

            colorVol(x,y,z) = make_uchar3( Imgrgb.Get(int(p_i.x),int(p_i.y)).x,
                                           Imgrgb.Get(int(p_i.x),int(p_i.y)).y,
                                           Imgrgb.Get(int(p_i.x),int(p_i.y)).z) ;
          }
        }
      }
    }
  }
}

void SdfFuseColor(
    BoundedVolume<SDF_t> vol, BoundedVolume<uchar3> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K,
    Image<float> img, Image<uchar3> Imgrgb, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
    float trunc_dist, float max_w, float mincostheta
    ) {
  dim3 blockDim(16,16);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y);
  KernSdfFuseColor<<<gridDim,blockDim>>>(vol, colorVol, depth, norm,
                                         T_cw, K, img, Imgrgb, T_iw, Kimg,
                                         trunc_dist, max_w, mincostheta);
  GpuCheckErrors();
}


//////////////////////////////////////////////////////
// Reset SDF
//////////////////////////////////////////////////////

void SdfReset(BoundedVolume<SDF_t> vol, float trunc_dist)
{
  vol.Fill(SDF_t(0.0/0.0, 0));
}

void SdfReset(BoundedVolume<float> vol)
{
  vol.Fill(0.5);
}

void SdfReset(BoundedVolume<uchar3> vol)
{
  vol.Fill(make_uchar3( 0,0,0 ));
}

void SdfReset(VolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol)
{
  vol.Fill(SDF_t(0.0/0.0, 0));
}

void SdfReset(VolumeGrid<float,roo::TargetDevice, roo::Manage> vol)
{
  vol.Fill(0.5);
}






//////////////////////////////////////////////////////
// Create SDF representation of sphere
//////////////////////////////////////////////////////

__global__ void KernSdfSphere(BoundedVolume<SDF_t> vol, float3 center, float r)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;
  const int z = blockIdx.z*blockDim.z + threadIdx.z;

  const float3 pos = vol.VoxelPositionInUnits(x,y,z);
  const float dist = length(pos - center);
  const float sdf = dist - r;

  vol(x,y,z) = SDF_t(sdf);
}

void SdfSphere(BoundedVolume<SDF_t> vol, float3 center, float r)
{
  dim3 blockDim(8,8,8);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);

  KernSdfSphere<<<gridDim,blockDim>>>(vol, center, r);
  GpuCheckErrors();
}

//////////////////////////////////////////////////////
// Take SDF Difference to depthmap
//////////////////////////////////////////////////////

__global__ void KernSdfDistance(Image<float> dist, Image<float> depth, BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float trunc_distance)
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if( u < depth.w && v < depth.h ) {
    const float z = depth(u,v);
    const float3 p_c = z * K.Unproject(u,v);
    const float3 p_w = T_wc * p_c;

    const SDF_t sdf = vol.GetUnitsTrilinearClamped(p_w);
    dist(u,v) = sdf.val; //(sdf.val + trunc_distance) / (2* trunc_distance);
  }
}


void SdfDistance(Image<float> dist, Image<float> depth, BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float trunc_distance)
{
  dim3 blockDim, gridDim;
  InitDimFromOutputImageOver(blockDim, gridDim, depth);

  KernSdfDistance<<<gridDim,blockDim>>>(dist, depth, vol, T_wc, K, trunc_distance);
  GpuCheckErrors();
}

}
