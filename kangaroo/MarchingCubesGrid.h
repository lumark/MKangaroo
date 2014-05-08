#pragma once

#include "BoundedVolumeGrid.h"
#include "kangaroo/extra/SavePPMGrid.h"
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
  hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes, vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.init(1,1,1, vol.m_nVolumeGridRes,vol.m_bbox );

  SaveMeshGrid<T,float>(filename, hvol, hvolcolor);
}

template<typename T, typename TColor, typename Manage>
void SaveMeshGrid(std::string filename, BoundedVolumeGrid<T,TargetDevice,Manage>& vol, BoundedVolumeGrid<TColor,TargetDevice,Manage>& volColor )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.init(volColor.m_w, volColor.m_h, volColor.m_d, volColor.m_nVolumeGridRes,volColor.m_bbox);
  hvolcolor.CopyAndInitFrom(volColor);

  // save
  SaveMeshGrid<T,TColor>(filename, hvol, hvolcolor);
}


//template<typename T, typename TColor, typename Manage>
//void GenMeshGrid(std::string filename )
//{

//  LoadPXMGrid();

//  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
//  hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);
//  hvol.CopyAndInitFrom(vol);

//  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
//  hvolcolor.init(volColor.m_w, volColor.m_h, volColor.m_d, volColor.m_nVolumeGridRes,volColor.m_bbox);
//  hvolcolor.CopyAndInitFrom(volColor);

//  // save
//  SaveMeshGrid<T,TColor>(filename, hvol, hvolcolor);
//}

template<typename T, typename TColor>
void SaveMeshGridSepreate(std::string filename, const BoundedVolumeGrid<T,TargetHost,Manage> vol, const BoundedVolumeGrid<TColor,TargetHost,Manage> volColor );

template<typename T, typename Manage>
void SaveMeshGridSepreate(std::string filename, BoundedVolumeGrid<T,TargetDevice,Manage>& vol )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes, vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.init(1,1,1, vol.m_nVolumeGridRes,vol.m_bbox );

  SaveMeshGridSepreate<T,float>(filename, hvol, hvolcolor);
}

template<typename T, typename TColor, typename Manage>
void SaveMeshGridSepreate(std::string filename, BoundedVolumeGrid<T,TargetDevice,Manage>& vol, BoundedVolumeGrid<TColor,TargetDevice,Manage>& volColor )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.init(volColor.m_w, volColor.m_h, volColor.m_d, volColor.m_nVolumeGridRes,volColor.m_bbox);
  hvolcolor.CopyAndInitFrom(volColor);

  // save
  SaveMeshGridSepreate<T,TColor>(filename, hvol, hvolcolor);
}



}
