#pragma once

#include "BoundedVolumeGrid.h"

namespace roo {

//////////////////////////////////////////
// Save SDF
//////////////////////////////////////////

template<typename T, typename TColor>
void SaveMeshGrid(std::string filename, const BoundedVolumeGrid<T,TargetHost,Manage> vol, const BoundedVolumeGrid<TColor,TargetHost,Manage> volColor );

template<typename T, typename Manage>
void SaveMeshGrid(std::string filename, BoundedVolumeGrid<T,TargetDevice,Manage>& vol )
{
    roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
    hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);

    roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
    hvolcolor.init(1,1,1, vol.m_nVolumeGridRes,vol.m_bbox );

    hvol.CopyAndInitFrom(vol);
    SaveMeshGrid<T,float>(filename, hvol, hvolcolor);
}

template<typename T, typename TColor, typename Manage>
void SaveMeshGrid(std::string filename, BoundedVolumeGrid<T,TargetDevice,Manage>& vol, BoundedVolumeGrid<TColor,TargetDevice,Manage>& volColor )
{
    roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
    hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);

    roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
    hvolcolor.init(volColor.m_w, volColor.m_h, volColor.m_d, vol.m_nVolumeGridRes,vol.m_bbox);

    hvol.CopyAndInitFrom(vol);
    hvolcolor.CopyAndInitFrom(volColor);

    // save
    SaveMeshGrid<T,TColor>(filename, hvol, hvolcolor);
}

}
