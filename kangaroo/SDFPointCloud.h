#ifndef SDFPOINTCLOUD_H
#define SDFPOINTCLOUD_H

#include "Volume.h"
#include "BoundingBox.h"
#include "Sdf.h"

namespace roo
{

class SDFPointCloud
{
public:
int         m_Index_x;
int         m_Index_y;
int         m_Index_z;
roo::SDF_t  m_SDF;
float       m_Intensity;

__device__ __host__
void Save(int X, int Y, int Z, roo::SDF_t& rSDF, float fIntensity)
{
  m_Index_x = X;
  m_Index_y = Y;
  m_Index_z = Z;
  m_SDF = rSDF;
  m_Intensity = fIntensity;
}

};

}


#endif // SDFPOINTCLOUD_H
