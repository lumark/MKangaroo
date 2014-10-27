#include "cu_raycast_grid.h"

namespace roo
{

//////////////////////////////////////////////////////
// Phong shading.
//////////////////////////////////////////////////////

__host__ __device__ inline
float PhongShade(const float3 p_c, const float3 n_c)
{
  const float ambient = 0.4;
  const float diffuse = 0.4;
  const float specular = 0.2;
  const float3 eyedir = -1.0f * p_c / length(p_c);
  const float3 _lightdir = make_float3(0.4,0.4,-1);
  const float3 lightdir = _lightdir / length(_lightdir);
  const float ldotn = dot(lightdir,n_c);
  const float3 lightreflect = 2*ldotn*n_c + (-1.0) * lightdir;
  const float edotr = fmaxf(0,dot(eyedir,lightreflect));
  const float spec = edotr*edotr*edotr*edotr*edotr*edotr*edotr*edotr*edotr*edotr;
  return ambient + diffuse * ldotn  + specular * spec;
}

//////////////////////////////////////////////////////
// Raycast grid gray SDF
//////////////////////////////////////////////////////
__device__ BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage>        g_vol;
__device__ BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage>  g_vol_smart;
__device__ BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>        g_grayVol;



////////////////////////////////////////////////////////////////////////////////
////////////////////////////// raycast SDF with SDF_t //////////////////////////
////////////////////////////////////////////////////////////////////////////////
// raycast grid SDF
__global__ void KernRaycastSdfGrid(
    Image<float> imgdepth, Image<float4> norm, Image<float> img,
    const Mat<float,3,4> T_wc, ImageIntrinsics K,
    float near, float far, float trunc_dist, bool subpix )
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if( u < img.w && v < img.h ) {
    // get only translation matirx
    const float3 c_w = SE3Translation(T_wc);

    //
    const float3 ray_c = K.Unproject(u,v);
    const float3 ray_w = mulSO3(T_wc, ray_c);

    // Raycast bounding box to find valid ray segment of sdf
    // http://www.cs.utah.edu/~awilliam/box/box.pdf
    const float3 tminbound = (g_vol.m_bbox.Min() - c_w) / ray_w;
    const float3 tmaxbound = (g_vol.m_bbox.Max() - c_w) / ray_w;
    const float3 tmin = fminf(tminbound,tmaxbound);
    const float3 tmax = fmaxf(tminbound,tmaxbound);
    const float max_tmin = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), near);
    const float min_tmax = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), far);

    float depth = 0.0f;

    // If ray intersects bounding box
    if(max_tmin < min_tmax ) {
      // Go between max_tmin and min_tmax
      float lambda = max_tmin;
      float last_sdf = 0.0f/0.0f;
      float min_delta_lambda = g_vol.VoxelSizeUnits().x;
      float delta_lambda = 0;

      // March through space
      while(lambda < min_tmax) {
        const float3 pos_w = c_w + lambda * ray_w;
        const float sdf = g_vol.GetUnitsTrilinearClamped(pos_w);

        if( sdf <= 0 ) {
          if( last_sdf > 0) {
            // surface!
            if(subpix) {
              lambda = lambda + delta_lambda * sdf / (last_sdf - sdf);
            }
            depth = lambda;
          }
          break;
        }
        delta_lambda = sdf > 0 ? fmaxf(sdf, min_delta_lambda) : trunc_dist;
        lambda += delta_lambda;
        last_sdf = sdf;
      }
    }

    // Compute normal
    const float3 pos_w = c_w + depth * ray_w;
    const float3 _n_w = g_vol.GetUnitsBackwardDiffDxDyDz(pos_w);
    const float len_n_w = length(_n_w);
    const float3 n_w = len_n_w > 0 ? _n_w / len_n_w : make_float3(0,0,1);
    const float3 n_c = mulSO3inv(T_wc,n_w);
    const float3 p_c = depth * ray_c;

    if(depth > 0 ) {
      //          img(u,v) = (depth - near) / (far - near);
      imgdepth(u,v) = depth;
      img(u,v) = roo::PhongShade(p_c, n_c);
      //            norm(u,v) = make_float4(0.5,0.5,0.5,1) + make_float4(n_c, 0) /2.0f;
      norm(u,v) = make_float4(n_c, 1);
    }else{
      imgdepth(u,v) = 0.0f/0.0f;
      img(u,v) = 0;
      norm(u,v) = make_float4(0,0,0,0);
    }
  }
}

void RaycastSdf(
    Image<float> depth, Image<float4> norm, Image<float> img,
    const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
    const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
    float trunc_dist, bool subpix )
{
  // load vol val to golbal memory
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  dim3 blockDim, gridDim;
  //    InitDimFromOutputImageOver(blockDim, gridDim, img, 16, 16);
  InitDimFromOutputImageOver(blockDim, gridDim, img);
  KernRaycastSdfGrid<<<gridDim,blockDim>>>(depth, norm, img, T_wc, K, near, far,
                                           trunc_dist, subpix);
  GpuCheckErrors();

  g_vol.FreeMemory();
}


// raycast grid gray SDF
__global__ void KernRaycastSdfGridGray(
    Image<float> imgdepth, Image<float4> norm, Image<float> img,
    const Mat<float,3,4> T_wc, ImageIntrinsics K,
    float near, float far, float trunc_dist, bool subpix )
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if( u < img.w && v < img.h ) {
    const float3 c_w = SE3Translation(T_wc);
    const float3 ray_c = K.Unproject(u,v);
    const float3 ray_w = mulSO3(T_wc, ray_c);

    // Raycast bounding box to find valid ray segment of sdf
    // http://www.cs.utah.edu/~awilliam/box/box.pdf
    const float3 tminbound = (g_vol.m_bbox.Min() - c_w) / ray_w;
    const float3 tmaxbound = (g_vol.m_bbox.Max() - c_w) / ray_w;
    const float3 tmin = fminf(tminbound,tmaxbound);
    const float3 tmax = fmaxf(tminbound,tmaxbound);
    const float max_tmin = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), near);
    const float min_tmax = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), far);

    float depth = 0.0f;

    // If ray intersects bounding box
    if(max_tmin < min_tmax ) {
      // Go between max_tmin and min_tmax
      float lambda = max_tmin;
      float last_sdf = 0.0f/0.0f;
      float min_delta_lambda = g_vol.VoxelSizeUnits().x;
      float delta_lambda = 0;

      // March through space
      while(lambda < min_tmax) {

        const float3 pos_w = c_w + lambda * ray_w;
        const float sdf = g_vol.GetUnitsTrilinearClamped(pos_w);

        if( sdf <= 0 )
        {
          if( last_sdf > 0) {
            // surface!
            if(subpix) {
              lambda = lambda + delta_lambda * sdf / (last_sdf - sdf);
            }
            depth = lambda;
          }
          break;
        }
        delta_lambda = sdf > 0 ? fmaxf(sdf, min_delta_lambda) : trunc_dist;
        lambda += delta_lambda;
        last_sdf = sdf;
      }
    }

    // Compute normal
    const float3 pos_w = c_w + depth * ray_w;
    const float3 _n_w = g_vol.GetUnitsBackwardDiffDxDyDz(pos_w);
    const float c = g_grayVol.GetUnitsTrilinearClamped(pos_w);
    const float len_n_w = length(_n_w);
    const float3 n_w = len_n_w > 0 ? _n_w / len_n_w : make_float3(0,0,1);
    const float3 n_c = mulSO3inv(T_wc,n_w);

    // render surface
    if(depth > 0 ) {
      imgdepth(u,v) = depth;
      img(u,v) = c;
      norm(u,v) = make_float4(n_c, 1);
    }else{
      imgdepth(u,v) = 0.0f/0.0f;
      img(u,v) = 0;
      norm(u,v) = make_float4(0,0,0,0);
    }

  }
}


void RaycastSdf(
    Image<float> depth, Image<float4> norm, Image<float> img,
    const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
    const BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> grayVol,
    const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
    float trunc_dist, bool subpix )
{
  // load vol val to golbal memory
  cudaMemcpyToSymbol(g_vol, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &grayVol, sizeof(grayVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  dim3 blockDim, gridDim;
  InitDimFromOutputImageOver(blockDim, gridDim, img);
  KernRaycastSdfGridGray<<<gridDim,blockDim>>>(depth, norm, img, T_wc, K, near,
                                               far, trunc_dist, subpix);
  GpuCheckErrors();

  g_vol.FreeMemory();
  g_grayVol.FreeMemory();
}



////////////////////////////////////////////////////////////////////////////////
///////////////////////// raycast SDF with SDF_t_Smart /////////////////////////
////////////////////////////////////////////////////////////////////////////////
// raycast grid SDF
__global__ void KernRaycastSdfGridSmart(
    Image<float> imgdepth, Image<float4> norm, Image<float> img,
    const Mat<float,3,4> T_wc, ImageIntrinsics K,
    float near, float far, float trunc_dist, bool subpix )
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if( u < img.w && v < img.h ) {
    // get only translation matirx
    const float3 c_w = SE3Translation(T_wc);

    const float3 ray_c = K.Unproject(u,v);
    const float3 ray_w = mulSO3(T_wc, ray_c);

    // Raycast bounding box to find valid ray segment of sdf
    // http://www.cs.utah.edu/~awilliam/box/box.pdf
    const float3 tminbound = (g_vol_smart.m_bbox.Min() - c_w) / ray_w;
    const float3 tmaxbound = (g_vol_smart.m_bbox.Max() - c_w) / ray_w;
    const float3 tmin = fminf(tminbound,tmaxbound);
    const float3 tmax = fmaxf(tminbound,tmaxbound);
    const float max_tmin = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), near);
    const float min_tmax = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), far);

    float depth = 0.0f;

    // If ray intersects bounding box
    if(max_tmin < min_tmax ) {
      // Go between max_tmin and min_tmax
      float lambda = max_tmin;
      float last_sdf = 0.0f/0.0f;
      float min_delta_lambda = g_vol_smart.VoxelSizeUnits().x;
      float delta_lambda = 0;

      // March through space
      while(lambda < min_tmax) {
        const float3 pos_w = c_w + lambda * ray_w;
        const float sdf = g_vol_smart.GetUnitsTrilinearClamped(pos_w);

        if( sdf <= 0 ) {
          if( last_sdf > 0) {
            // surface!
            if(subpix) {
              lambda = lambda + delta_lambda * sdf / (last_sdf - sdf);
            }
            depth = lambda;
          }
          break;
        }
        delta_lambda = sdf > 0 ? fmaxf(sdf, min_delta_lambda) : trunc_dist;
        lambda += delta_lambda;
        last_sdf = sdf;
      }
    }

    // Compute normal
    const float3 pos_w = c_w + depth * ray_w;
    const float3 _n_w = g_vol_smart.GetUnitsBackwardDiffDxDyDz(pos_w);
    const float len_n_w = length(_n_w);
    const float3 n_w = len_n_w > 0 ? _n_w / len_n_w : make_float3(0,0,1);
    const float3 n_c = mulSO3inv(T_wc,n_w);
    const float3 p_c = depth * ray_c;

    if(depth > 0 ) {
      //          img(u,v) = (depth - near) / (far - near);
      imgdepth(u,v) = depth;
      img(u,v) = roo::PhongShade(p_c, n_c);
      //            norm(u,v) = make_float4(0.5,0.5,0.5,1) + make_float4(n_c, 0) /2.0f;
      norm(u,v) = make_float4(n_c, 1);
    }else{
      imgdepth(u,v) = 0.0f/0.0f;
      img(u,v) = 0;
      norm(u,v) = make_float4(0,0,0,0);
    }
  }
}


void RaycastSdf(
    Image<float> depth, Image<float4> norm, Image<float> img,
    const BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol, const Mat<float,3,4> T_wc,
    ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix )
{
  // load vol val to golbal memory
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  dim3 blockDim, gridDim;
  //    InitDimFromOutputImageOver(blockDim, gridDim, img, 16, 16);
  InitDimFromOutputImageOver(blockDim, gridDim, img);
  KernRaycastSdfGridSmart<<<gridDim,blockDim>>>(depth, norm, img, T_wc, K, near,
                                                far, trunc_dist, subpix);
  GpuCheckErrors();

  g_vol_smart.FreeMemory();
}


// raycast grid gray SDF
__global__ void KernRaycastSdfGridGraySmart(
    Image<float> imgdepth, Image<float4> norm, Image<float> img,
    const Mat<float,3,4> T_wc, ImageIntrinsics K,
    float near, float far, float trunc_dist, bool subpix )
{
  const int u = blockIdx.x*blockDim.x + threadIdx.x;
  const int v = blockIdx.y*blockDim.y + threadIdx.y;

  if( u < img.w && v < img.h ) {
    const float3 c_w = SE3Translation(T_wc);
    const float3 ray_c = K.Unproject(u,v);
    const float3 ray_w = mulSO3(T_wc, ray_c);

    // Raycast bounding box to find valid ray segment of sdf
    // http://www.cs.utah.edu/~awilliam/box/box.pdf
    const float3 tminbound = (g_vol_smart.m_bbox.Min() - c_w) / ray_w;
    const float3 tmaxbound = (g_vol_smart.m_bbox.Max() - c_w) / ray_w;
    const float3 tmin = fminf(tminbound,tmaxbound);
    const float3 tmax = fmaxf(tminbound,tmaxbound);
    const float max_tmin = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), near);
    const float min_tmax = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), far);

    float depth = 0.0f;

    // If ray intersects bounding box
    if(max_tmin < min_tmax ) {
      // Go between max_tmin and min_tmax
      float lambda = max_tmin;
      float last_sdf = 0.0f/0.0f;
      float min_delta_lambda = g_vol_smart.VoxelSizeUnits().x;
      float delta_lambda = 0;

      // March through space
      while(lambda < min_tmax) {

        const float3 pos_w = c_w + lambda * ray_w;
        const float sdf = g_vol_smart.GetUnitsTrilinearClamped(pos_w);

        if( sdf <= 0 )
        {
          if( last_sdf > 0) {
            // surface!
            if(subpix) {
              lambda = lambda + delta_lambda * sdf / (last_sdf - sdf);
            }
            depth = lambda;
          }
          break;
        }
        delta_lambda = sdf > 0 ? fmaxf(sdf, min_delta_lambda) : trunc_dist;
        lambda += delta_lambda;
        last_sdf = sdf;
      }
    }

    // Compute normal
    const float3 pos_w = c_w + depth * ray_w;
    const float3 _n_w = g_vol_smart.GetUnitsBackwardDiffDxDyDz(pos_w);
    const float c = g_grayVol.GetUnitsTrilinearClamped(pos_w);
    const float len_n_w = length(_n_w);
    const float3 n_w = len_n_w > 0 ? _n_w / len_n_w : make_float3(0,0,1);
    const float3 n_c = mulSO3inv(T_wc,n_w);

    // render surface
    if(depth > 0 ) {
      imgdepth(u,v) = depth;
      img(u,v) = c;
      norm(u,v) = make_float4(n_c, 1);
    }else{
      imgdepth(u,v) = 0.0f/0.0f;
      img(u,v) = 0;
      norm(u,v) = make_float4(0,0,0,0);
    }

  }
}


void RaycastSdf(
    Image<float> depth, Image<float4> norm, Image<float> img,
    const BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
    const BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> grayVol,
    const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
    float trunc_dist, bool subpix )
{
  // load vol val to golbal memory
  cudaMemcpyToSymbol(g_vol_smart, &vol, sizeof(vol), size_t(0), cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(g_grayVol, &grayVol, sizeof(grayVol), size_t(0), cudaMemcpyHostToDevice);
  GpuCheckErrors();

  dim3 blockDim, gridDim;
  InitDimFromOutputImageOver(blockDim, gridDim, img);
  KernRaycastSdfGridGraySmart<<<gridDim,blockDim>>>(depth, norm, img, T_wc, K,
                                                    near, far, trunc_dist, subpix);
  GpuCheckErrors();

  g_vol_smart.FreeMemory();
  g_grayVol.FreeMemory();
}

}
