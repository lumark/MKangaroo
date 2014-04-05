#ifndef BASICSDF_H
#define BASICSDF_H


#include "VolumeGrid.h"
#include "BoundingBox.h"
#include "BoundedVolume.h"
#include "Sdf.h"
#include "cu_sdffusion.h"


template<typename T, typename Target = TargetDevice, typename Management = Manage>
class BasicSDF
{
public:

  inline __host__ __device__
  BasicSDF(T, Target, Management):
    m_bbox(make_float3(-256,-256,0.5),
           make_float3(256,256,0.5+2*256))
  {
    m_bInit = true;
    m_Vol.InitVolume(32,32,32);
    SdfReset(m_Vol);
    printf("[BasicSDF/1] init basic bounded volume grid success. bbox min x is %f, input min is %f, res is %d\n", m_bbox.boxmin.x, m_bbox.boxmin.x, 32);
  }

  bool                                              m_bInit;
  roo::BoundingBox                                  m_bbox; // bounding box for volume
  roo::BoundedVolume<T, Target, Management>         m_Vol;          // volume
  roo::BoundedVolume<T, Target, Management>         m_TestVol[16];          // volume
};




#endif // BASICSDF_H
