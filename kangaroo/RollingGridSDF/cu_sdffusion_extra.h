#pragma once

#include "kangaroo/platform.h"
#include "kangaroo/Mat.h"
#include "kangaroo/Image.h"
#include "kangaroo/ImageIntrinsics.h"
#include "kangaroo/BoundedVolume.h"
#include "kangaroo/Sdf.h"
#include "SdfSmart.h"

namespace roo
{
KANGAROO_EXPORT
void SdfFuseDirectGray(
    BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
    ImageIntrinsics Kdepth, Image<float> gray, Mat<float,3,4> T_iw,
    ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta,float min_depth);

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
KANGAROO_EXPORT
void SdfFuseColor(BoundedVolume<SDF_t> vol, BoundedVolume<uchar3> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
    ImageIntrinsics K, Image<float> img, Image<uchar3> Imgrgb,
    Mat<float,3,4> T_iw, ImageIntrinsics Kimg, float trunc_dist,
    float max_w, float mincostheta, float min_depth);

KANGAROO_EXPORT
void SdfReset(BoundedVolume<SDF_t,  roo::TargetDevice, roo::Manage> vol, float trunc_dist = 0);

//void SdfReset(BoundedVolume<float> vol);

KANGAROO_EXPORT
void SdfReset(BoundedVolume<uchar3> vol);

}
