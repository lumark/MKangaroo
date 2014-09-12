#include "Kangaroo/GridSDF/cu_sdffusion_extra.h"

#include "Kangaroo/MatUtils.h"
#include "Kangaroo/launch_utils.h"

namespace roo
{

//--the following add by luma-----------------------------------------------------------------------------------------------------------------------------
// do SDF fusion without consideing void (zero intensity) pixels
__global__ void KernSdfFuseDirectGray(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> normals, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
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
            // get current voxel sdf value
            const SDF_t curvol = vol(x,y,z);

            // return min of 'sd' and 'trunc_dist' as 'x', then rerurn max of 'x' and 'w'
            SDF_t sdf( clamp(sd, -trunc_dist, trunc_dist) , w);

            // upatedate SDF
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

void SdfFuseDirectGray(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    ) {
  dim3 blockDim(16,16);
  dim3 gridDim(vol.w / blockDim.x, vol.h / blockDim.y);
  KernSdfFuseDirectGray<<<gridDim,blockDim>>>(vol, colorVol, depth, norm, T_cw, Kdepth, gray, T_iw, Krgb, trunc_dist, max_w, mincostheta);
  GpuCheckErrors();
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



void SdfReset(BoundedVolume<SDF_t, roo::TargetDevice, roo::Manage> vol, float trunc_dist)
{
  vol.Fill(SDF_t(0.0/0.0, 0));
}

//void SdfReset(BoundedVolume<float> vol)
//{
//  vol.Fill(0.5);
//}

void SdfReset(BoundedVolume<uchar3> vol)
{
  vol.Fill(make_uchar3( 0,0,0 ));
}


}
