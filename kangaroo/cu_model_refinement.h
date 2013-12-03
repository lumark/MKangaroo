#pragma once

#include "Mat.h"
#include "Image.h"

namespace roo
{

LeastSquaresSystem<float,6> PoseRefinementFromVbo(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr, const Image<float4> dPr,
    const Mat<float,3,4> KT_lr, float c,
    Image<unsigned char> dWorkspace, Image<float4> dDebug
);

LeastSquaresSystem<float,6> PoseRefinementFromDisparity(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr, const Image<float> dDispr,
    const Mat<float,3,4> KT_lr, float c,
    float baseline, float fu, float fv, float u0, float v0,
    Image<unsigned char> dWorkspace, Image<float4> dDebug
);

LeastSquaresSystem<float,6> PoseRefinementFromDisparityESM(
    const Image<unsigned char> dImgl, const Image<unsigned char> dImgr,
    const Image<float> dDisp, const float baseline,
    const Mat<float,3,3> Kg, const Mat<float,3,3> Kd, const Mat<float,4,4> Tgd,
    const Mat<float,4,4> Tlr, const Mat<float,3,4> KgTlr,
    Image<LeastSquaresSystem<float,6> > dSum, Image<float4> dDebug,
    const float c, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
);

LeastSquaresSystem<float,6> PoseRefinementFromDepthESM(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr,
    const Image<float> dDepth,
    const Mat<float,3,3> Klg, const Mat<float,3,3> Krg, const Mat<float,3,3> Krd, const Mat<float,4,4> Tgd,
    const Mat<float,4,4> Tlr, const Mat<float,3,4> KlgTlr,
    Image<unsigned char> dWorkspace, Image<float4> dDebug,
    const float c, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
);

// -------------------------------------------------------------------------------------------------- add by luma
LeastSquaresSystem<float,6> ESMDetectOutline(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr,
    const Image<float> dDepth,
    const Mat<float,3,3> Klg, const Mat<float,3,3> Krg, const Mat<float,3,3> Krd, const Mat<float,4,4> Tgd,
    const Mat<float,4,4> Tlr, const Mat<float,3,4> KlgTlr,
    Image<unsigned char> dWorkspace, Image<float4> dDebug,
    const float c, const float rate, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
);
// -------------------------------------------------------------------------------------------------- add by luma

LeastSquaresSystem<float,6> CalibrationRgbdFromDepthESM(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr, const Image<float> dDepth,
    const Mat<float,3,3>& K,const Mat<float,3,4>& T_di,const Mat<float,3,4>& T_lr, float c,
    float fu, float fv, float u0, float v0,
    Image<unsigned char> dWorkspace, Image<float4> dDebug,
    const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
);

// ------------------------------------------------------------------------------------------------- add by luma
LeastSquaresSystem<float,6> ICPDetOutLine(
    const Image<float4> dPl,
    const Image<float4> dPr, const Image<float4> dNr,
    const Mat<float,3,4> KT_lr, const Mat<float,3,4> T_rl, float c, float rate,
    Image<unsigned char> dWorkspace, Image<float4> dDebug
);
// ------------------------------------------------------------------------------------------------- add by luma


LeastSquaresSystem<float,6> PoseRefinementProjectiveIcpPointPlane(
    const Image<float4> dPl,
    const Image<float4> dPr, const Image<float4> dNr,
    const Mat<float,3,4> KT_lr, const Mat<float,3,4> T_rl, float c,
    Image<unsigned char> dWorkspace, Image<float4> dDebug
);

LeastSquaresSystem<float,2*6> KinectCalibration(
    const Image<float4> dPl, const Image<uchar3> dIl,
    const Image<float4> dPr, const Image<uchar3> dIr,
    const Mat<float,3,4> KcT_cd, const Mat<float,3,4> T_lr,
    float c, Image<unsigned char> dWorkspace, Image<float4> dDebug
);

void SumSpeedTest(
    Image<unsigned char> dWorkspace, int w, int h, int blockx, int blocky
);

}
