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
      //            const float md = depth.GetNearestNeighbour(p_c);
      //            const float3 mdn = make_float3(normals.GetNearestNeighbour(p_c));
      //            const float c = ConvertPixel<float,uchar3>( img.GetNearestNeighbour(p_i) );

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
    ) {
  //    // 3d invoke
  //    dim3 blockDim(8,8,8);
  //    dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);
  //    KernSdfFuse<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, K, img, T_iw, Kimg, trunc_dist, max_w, mincostheta);
  //    GpuCheckErrors();

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
    //        printf("loc:%f,%f,%f;",P_w.x,P_w.y,P_w.z);
    //    printf("x:%d,y:%d,z:%d,loc:%f,%f,%f;",x,y,z,P_w.x,P_w.y,P_w.z);

    // Get voxel position in camera coordinate
    const float3 P_c = T_cw * P_w;
    //    printf("P_c x:%f,y:%f,z:%f;",P_c.x,P_c.y,P_c.z);

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

__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>  g_Vol;
__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>  g_colorVol;

int GetAvailableGPUMemory()
{
  const unsigned bytes_per_mb = 1024*1000;
  size_t cu_mem_start, cu_mem_total;
  if(cudaMemGetInfo( &cu_mem_start, &cu_mem_total ) != cudaSuccess) {
    std::cerr << "Unable to get available memory" << std::endl;
    exit(-1);
  }

  int LeftMemory = cu_mem_start/bytes_per_mb;
  return LeftMemory;
}

// -----------------------------------------------------------------------------
//--the following add by luma---------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfFuseDirectGreyGrid(VolumeGrid<SDF_t, TargetDevice, Manage> testvol,
                                          Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                                          Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                                          float trunc_dist, float max_w, float mincostheta
                                          )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;

  // For each voxel (x,y,z) we have in a bounded volume
  for(int z=0; z < g_Vol.m_d; ++z)
  {
    // See if this voxel is possible to be in the image boundary
    // Get voxel position in certain radius in world coordinate (good)
    const float3 P_w = g_Vol.VoxelPositionInUnits(x,y,z);

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

      //      printf("in bounds.");
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
            g_Vol.m_GridVolumes[0](0,0,0).val = g_Vol.m_GridVolumes[0](0,0,0).val +1;

            //            printf("val is %f",g_Vol.m_GridVolumes[0](0,0,0).val);

            printf("val is %f",testvol(0,0,0).val);


            //            printf("vol is %f, workspace is %f;",vol.m_dSingleVolumeGrid(0,0,0).val, GridSDFWorkSpace[0](0,0,0).val);

            //            printf("workspace is %f;", GridSDFWorkSpace[0](0,0,0).val);

            //            g_Vol.m_GridVolumes[0].Get(0,0,0).val = 1;

            //            printf("md %f,", md);
            //            printf("try to get vol value.");
            //            const SDF_t curvol = vol.GetElementSdf(x,y,z);

            //            const SDF_t curvol= vol.GetVal(x,y,z);

            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            //            SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
            //            sdf += curvol;
            //            sdf.LimitWeight(max_w);

            //            printf("val:%f",pTest->Get(0,0,0).val);
            //            printf("test num is:%d",testnum);

            /// set val
            //            vol.SetElement(sdf, x, y, z);
            //            vol.SetVal(sdf, x,y,z);
            //            printf("get:%f;",vol.GetVal(x,y,z).val);

            //            printf("get:%f",vol.m_hSingGridVolume->Get(0,0,0).val);

            //            vol.m_GridVolume[0][0][0]->Get(0,0,0) = sdf;
            //            g_colorVol.SetElement(x,y,z, (w*c + g_colorVol.GetElementFloat(x,y,z) * curvol.w) / (w + curvol.w));
            //            printf("fuse:%f,grey:%f,pc:%f,%f-pi:%f,%f;",sdf.val,c,p_c.x, p_c.y,p_i.x,p_i.y);
            //            printf("fuse:%f,grey:%f",sdf.val,colorVol.GetElementFloat(x,y,z));
            //            printf("fuse:%f",vol.m_GridVolume[0][0][0]->Get(0,0,0));
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
    ) {

  //  int m_res = 4;

  //  BoundedVolume<SDF_t, TargetDevice, Manage>*   d_array;

  ////  int nSize = sizeof(vol.m_GridVolume);

  //  int nSize = sizeof(vol.m_TestVol) * 64;

  //  int nSize = sizeof(vol.m_hSingGridVolume);

  //  printf("each size is %d, %d",nSize);

  //  printf("get val %f",vol.m_hSingGridVolume->Get(0,0,0));

  //  // 1. Allocate device array.
  //  cudaMalloc((void**) &(d_array), nSize);
  //  GpuCheckErrors();

  //  // 2. Copy array contents from host to device.
  //  cudaMemcpy(d_array, vol.m_hSingGridVolume, nSize, cudaMemcpyHostToDevice);
  //  GpuCheckErrors();

  // 3. Point to device pointer in host struct.
  //  for(int i=0;i!=4;i++)
  //  {
  //    for(int j=0;j!=4;j++)
  //    {
  //      for(int k=0;k!=4;k++)
  //      {
  //        vol.m_hGridVolume[i + j * m_res + k * m_res * m_res] = d_array[i + j * m_res + k * m_res * m_res];
  //      }
  //    }
  //  }

  //  vol.m_hSingGridVolume = d_array;

  //  d_array->CopyFrom

  //    BoundedVolume<SDF_t, TargetDevice, Manage>* d_array;

  //    int nSize = sizeof(vol.m_TestSDF);

  //    printf("each size is %d, %d",nSize);

  //    // 1. Allocate device array.
  //    cudaMalloc((void**) &(d_array), nSize);
  //    GpuCheckErrors();

  //    // 2. Copy array contents from host to device.
  //    cudaMemcpy(d_array, vol.m_GridVolume, nSize, cudaMemcpyHostToDevice);
  //    GpuCheckErrors();


  cudaMemcpyToSymbol(g_Vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  // launch kernel
  dim3 blockDim(16,16);
  dim3 gridDim(vol.m_w / blockDim.x, vol.m_h / blockDim.y);
  KernSdfFuseDirectGreyGrid<<<gridDim,blockDim>>>(vol.m_GridVolumes[0], depth, norm, T_cw, Kdepth, grey, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();

  // copy data back after launch the kernel
  for(int i=0;i!=64;i++)
  {
    vol.m_GridVolumes[i].MemcpyFromDevice(g_Vol.m_GridVolumes[i]);
  }
}



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

//-----------------------------------------------------------------------------------------------------------------------------------

//////////////////////////////////////////////////////
// find outline from sdf
/////////////////////////////////////////////////////
__global__ void KernSdfFuseFindOutline(
    BoundedVolume<SDF_t> vol,BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics K,
    Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
    float trunc_dist, float max_w, float mincostheta, Image<float4> dOutLine
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

    // if voxel is inside bounds
    if( depth.InBounds(p_c, 2) && img.InBounds(p_i,2) )
    {
      const float vd = P_c.z;

      const float md = depth.GetBilinear<float>(p_c);
      const float3 mdn = make_float3(normals.GetBilinear<float4>(p_c));
      const float c = ConvertPixel<float,float3>( img.GetBilinear<float3>(p_i) ) / 255.0;

      const float costheta = dot(mdn, P_c) / -length(P_c);
      const float sd = costheta * (md - vd);
      const float w = costheta * 1.0f/vd;

      if(sd <= -trunc_dist)
      {
      }
      else
      {
        SDF_t sdf( clamp(sd,-trunc_dist,trunc_dist) , w);
        sdf += vol(x,y,z);

        sdf.LimitWeight(max_w);
        vol(x,y,z) = sdf;
      }
    }
    // out of boundary
    else
    {
      //            dOutLine(p_i.x,p_i.y) = make_float4(1, 0, 1, 1);
    }
  }

}




void SdfFuseFindOutline(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K,
    Image<uchar3> img, Mat<float,3,4> T_iw, ImageIntrinsics Kimg,
    float trunc_dist, float max_w, float mincostheta, Image<float4> dOutline
    ) {

  dim3 blockDim(16,16);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y);
  KernSdfFuseFindOutline<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, K, img, T_iw, Kimg, trunc_dist, max_w, mincostheta, dOutline);
  GpuCheckErrors();
}


__global__ void KernFindBBBoundary(
    BoundedVolume<float> colorVol, int max_x, int max_y, int max_z, int min_x, int min_y, int min_z
    )
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  //    const int z = blockIdx.z*blockDim.z + threadIdx.z;
  for(int z=0; z < colorVol.d; ++z)
  {
    const float curvol = colorVol(x,y,z);

    if(isfinite(curvol))
    {
      //            printf(",vol:%f", curvol);
      if(x>max_x)
      {
        max_x = x;
      }
      else if(x<min_x)
      {
        min_x = x;
      }

      if(y>max_y)
      {
        max_y = y;
      }
      else if(y<min_y)
      {
        min_y = y;
      }

      if(z>max_z)
      {
        max_z = z;
      }
      else if(z<min_z)
      {
        min_z = z;
      }
    }
  }
}

void FindBBBoundary(
    BoundedVolume<float> colorVol, int max_x, int max_y, int max_z, int min_x, int min_y, int min_z
    ) {
  dim3 blockDim(16,16);
  dim3 gridDim(colorVol.w / blockDim.x, colorVol.h / blockDim.y);
  KernFindBBBoundary<<<gridDim,blockDim>>>(colorVol, max_x, max_y, max_z, min_x, min_y, min_z);
  GpuCheckErrors();

}

// ---------------------------------------------------------------------------------------------------------------------------------


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
  vol.Fill(SDF_t(1, 1));
}

void SdfReset(VolumeGrid<float,roo::TargetDevice, roo::Manage> vol)
{
  vol.Fill(0.5);
}

//boxmin and boxmax define the box that is to be kept intact, rest will be cleared. This approach makes if conditions inside simpler.
//TODO: Name the function better.
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

//TODO: Name the function better.
void SdfResetPartial(BoundedVolume<SDF_t> vol, float3 shift)
{
  //Initialization for GPU parallelization
  dim3 blockDim(8,8,8);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y, vol.d / blockDim.z);

  //compute the box to keep, it's conter intuitive to the name of function but more efficient.
  float3 bn = vol.bbox.boxmin, bx = vol.bbox.boxmax;//bn is box min and bx is box max.

  if(shift.x>0)
    bn.x += shift.x;
  else
    bx.x += shift.x;

  //y is -ve, but boxmax and boxmin for y are also inverse. i.e. the bottom most point is min.x,max.y,min.z
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
