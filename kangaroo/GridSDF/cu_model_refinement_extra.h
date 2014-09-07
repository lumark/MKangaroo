#pragma once

#include <kangaroo/platform.h>
#include <kangaroo/Mat.h>
#include <kangaroo/Image.h>

namespace roo
{

KANGAROO_EXPORT
LeastSquaresSystem<float,6> PoseRefinementFromDepthESMNormal(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr,
    const Image<float> dDepth,
    const Mat<float,3,3> Klg, const Mat<float,3,3> Krg, const Mat<float,3,3> Krd, const Mat<float,4,4> Tgd,
    const Mat<float,4,4> Tlr, const Mat<float,3,4> KlgTlr,
    Image<unsigned char> dWorkspace, Image<float4> dDebug,
    const float c, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
);

}
