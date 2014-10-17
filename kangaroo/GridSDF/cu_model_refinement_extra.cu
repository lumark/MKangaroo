#include "cu_model_refinement_extra.h"
#include "kangaroo/MatUtils.h"
#include "kangaroo/launch_utils.h"
#include "kangaroo/reweighting.h"
#include "kangaroo/disparity.h"
#include "kangaroo/LeastSquareSum.h"

namespace roo {

//////////////////////////////////////////////////////
// Pose refinement from depthmap
//////////////////////////////////////////////////////

template<typename Ti>
__device__ inline
void BuildPoseRefinementFromDepthmapSystemESMNormal(
    const unsigned int u,  const unsigned int v, const float depth,
    const Image<Ti>& dImgl, const Image<Ti>& dImgr,
    const Mat<float,3,3>& Klg, const Mat<float,3,3>& Krg, const Mat<float,3,3>& Krd, const Mat<float,4,4>& Tgd,
    const Mat<float,4,4>& Tlr, const Mat<float,3,4>& KlgTlr,
    LeastSquaresSystem<float,6>& lss, Image<float4> dDebug,
    const float c, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
    ) {
  // normalize pixel rgb value (for help when we need to combine ESM with ICP)
  float NormalRate = 0.0001;

  // 3d point from reference depth camera
  Mat<float,4> Pr_d;
  Pr_d(0) = depth * (u - Krd(0,2)) / Krd(0,0);
  Pr_d(1) = depth * (v - Krd(1,2)) / Krd(1,1);
  Pr_d(2) = depth;
  Pr_d(3) = 1;

  // 3d point from reference grey camera
  Mat<float,4> Pr_g = Tgd * Pr_d;

  // projected point of reference grey camera
  Mat<float,3> KrPr;
  KrPr(0) = Krg(0,0)*Pr_g(0) + Krg(0,2)*Pr_g(2);
  KrPr(1) = Krg(1,1)*Pr_g(1) + Krg(1,2)*Pr_g(2);
  KrPr(2) = Pr_g(2);

  // de-homogenized point in reference grey camera
  const Mat<float,2> pr = {{KrPr(0)/KrPr(2), KrPr(1)/KrPr(2)}};

  // 3d point in live grey camera
  const Mat<float,4> Pl = Tlr * Pr_g;

  // projected point of live grey camera
  Mat<float,3> KlPl;
  KlPl(0) = Klg(0,0)*Pl(0) + Klg(0,2)*Pl(2);
  KlPl(1) = Klg(1,1)*Pl(1) + Klg(1,2)*Pl(2);
  KlPl(2) = Pl(2);

  // de-homogenized point in live grey camera
  const Mat<float,2> pl = {{KlPl(0)/KlPl(2), KlPl(1)/KlPl(2)}};

  if(isfinite(depth) && depth > fMinDepth && depth < fMaxDepth) {
    if( dImgr.InBounds(pr(0), pr(1), 2) &&  dImgl.InBounds(pl(0), pl(1), 2) ) {

      float Il = NormalRate * dImgl.template GetBilinear<float>(pl(0), pl(1));
      float Ir = NormalRate * dImgr.template GetBilinear<float>(pr(0), pr(1));

      if( bDiscardMaxMin && ( Il == 0 || Il == 255 || Ir == 0 || Ir == 255 ) ) {
        dDebug(u, v) = make_float4(1, 1, 0, 1);
      } else {

        // image error
        const float y = Il - Ir;

        //----- Forward Compositional Approach

        // calculate image derivative
        const Mat<float,1,2> dIl =NormalRate *  dImgl.template GetCentralDiff<float>(pl(0), pl(1));

        // derivative of projection (L) and dehomogenization
        const Mat<float,2,3> dPl_by_dpl = {{
                                             1.0/KlPl(2), 0, -KlPl(0)/(KlPl(2)*KlPl(2)),
                                             0, 1.0/KlPl(2), -KlPl(1)/(KlPl(2)*KlPl(2))
                                           }};

        const Mat<float,1,4> dIldPlKlgTlr = dIl * dPl_by_dpl * KlgTlr;

        // Sparse Jl = dIldPlKT_lr * gen_i * Pr
        const Mat<float,1,6> Jl = {{
                                     dIldPlKlgTlr(0),
                                     dIldPlKlgTlr(1),
                                     dIldPlKlgTlr(2),
                                     -dIldPlKlgTlr(1)*Pr_g(2) + dIldPlKlgTlr(2)*Pr_g(1),
                                     +dIldPlKlgTlr(0)*Pr_g(2) - dIldPlKlgTlr(2)*Pr_g(0),
                                     -dIldPlKlgTlr(0)*Pr_g(1) + dIldPlKlgTlr(1)*Pr_g(0)
                                   }};



        //----- Inverse Compositional Approach

        /*
                const Mat<float,1,2> dIr = dImgl.template GetCentralDiff<float>(pr(0), pr(1));

                //derivative of projection (L) and dehomogenization for inverse decompositional
                const Mat<float,2,3> dPr = {
                  1.0/KPr(2), 0, -KPr(0)/(KPr(2)*KPr(2)),
                  0, 1.0/KPr(2), -KPr(1)/(KPr(2)*KPr(2))
                };

                const Mat<float,1,3> dIrdPrKg = dIr * dPr * Kg;

                // Sparse Jr = dIrdPrK * gen_i * Pr
                const Mat<float,1,6> Jr = {
                    dIrdPrKg(0),
                    dIrdPrKg(1),
                    dIrdPrKg(2),
                    -dIrdPrKg(1)*Pr_g(2) + dIrdPrKg(2)*Pr_g(1),
                    +dIrdPrKg(0)*Pr_g(2) - dIrdPrKg(2)*Pr_g(0),
                    -dIrdPrKg(0)*Pr_g(1) + dIrdPrKg(1)*Pr_g(0)
                };


                //----- ESM Jacobian
                const Mat<float,1,6> J = {
                    (Jr(0) + Jl(0))/2,
                    (Jr(1) + Jl(1))/2,
                    (Jr(2) + Jl(2))/2,
                    (Jr(3) + Jl(3))/2,
                    (Jr(4) + Jl(4))/2,
                    (Jr(5) + Jl(5))/2
                };
                */

        const float w = LSReweightTukey(y, NormalRate *c);
        lss.JTJ = OuterProduct(Jl, w);
        lss.JTy = mul_aTb(Jl, y*w);
        lss.obs = 1;
        lss.sqErr = y * y;

        const float debug = ( fabs(y) + 128 ) / 255.0f;
        dDebug(u,v) = make_float4(debug, 0, w, 1);
      }
    } else {
      dDebug(u,v) = make_float4(0, 1, 0, 1);
    }
  }else{
    dDebug(u,v) = make_float4(0, 0, 0, 1);
  }
}


template<typename Ti>
__global__ void KernPoseRefinementFromDepthESMNormal(
    const Image<Ti> dImgl, const Image<Ti> dImgr, const Image<float> dDepth,
    const Mat<float,3,3> Klg, const Mat<float,3,3> Krg, const Mat<float,3,3> Krd, const Mat<float,4,4> Tgd,
    const Mat<float,4,4> Tlr, const Mat<float,3,4> KlgTlr,
    Image<LeastSquaresSystem<float,6> > dSum, Image<float4> dDebug,
    const float c, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
    ) {
  const unsigned int u = blockIdx.x*blockDim.x + threadIdx.x;
  const unsigned int v = blockIdx.y*blockDim.y + threadIdx.y;

  __shared__ SumLeastSquaresSystem<float,6,16,16> lss;

  float depth = dDepth(u,v);

  BuildPoseRefinementFromDepthmapSystemESMNormal( u, v, depth, dImgl, dImgr, Klg, Krg, Krd, Tgd, Tlr, KlgTlr, lss.ZeroThisObs(), dDebug, c, bDiscardMaxMin, fMinDepth, fMaxDepth );

  lss.ReducePutBlock(dSum);
}

LeastSquaresSystem<float,6> PoseRefinementFromDepthESMNormal(
    const Image<unsigned char> dImgl,
    const Image<unsigned char> dImgr,
    const Image<float> dDepth,
    const Mat<float,3,3> Klg, const Mat<float,3,3> Krg, const Mat<float,3,3> Krd, const Mat<float,4,4> Tgd,
    const Mat<float,4,4> Tlr, const Mat<float,3,4> KlgTlr,
    Image<unsigned char> dWorkspace, Image<float4> dDebug,
    const float c, const bool bDiscardMaxMin, const float fMinDepth, const float fMaxDepth
    ){
  dim3 blockDim, gridDim;
  InitDimFromOutputImage(blockDim, gridDim, dImgr, 16, 16);

  HostSumLeastSquaresSystem<float,6> lss(dWorkspace, blockDim, gridDim);
  KernPoseRefinementFromDepthESMNormal<unsigned char><<<gridDim,blockDim>>>(dImgl, dImgr, dDepth, Klg, Krg, Krd, Tgd, Tlr, KlgTlr, lss.LeastSquareImage(), dDebug, c, bDiscardMaxMin, fMinDepth, fMaxDepth );
  return lss.FinalSystem();
}

}
