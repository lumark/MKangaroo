#pragma once

#include "Mat.h"
#include "Image.h"
#include "BoundedVolume.h"
#include "BoundedVolumeGrid.h"
#include "ImageIntrinsics.h"
#include "Sdf.h"

namespace roo
{

// --------------------------------------------------------------------------------------------------------------------------
// raycast sdf
void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolume<SDF_t> vol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix = true);

void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolume<SDF_t> vol, const BoundedVolume<float> colorVol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix = true);

void RaycastSdf(Image<float> depth, Image<float4> norm, Image<uchar3> imgrgb,
                const BoundedVolume<SDF_t> vol, const BoundedVolume<uchar3> colorVol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix = true);

// --------------------------------------------------------------------------------------------------------------------------
// raycast grid sdf
void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
                const BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> colorVol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far, float trunc_dist, bool subpix );


void RaycastBox(Image<float> depth, const Mat<float,3,4> T_wc, ImageIntrinsics K, const BoundingBox bbox );

void RaycastSphere(Image<float> depth, Image<float> img, const Mat<float,3,4> T_wc, ImageIntrinsics K, float3 center, float r);

void RaycastPlane(Image<float> depth, Image<float> img, const Mat<float,3,4> T_wc, ImageIntrinsics K, const float3 n_w );

}
