#ifndef SAVEPPMGRID_H
#define SAVEPPMGRID_H

#include "SavePPM.h"
#include <kangaroo/BoundedVolumeGrid.h>

// P1	Portable bitmap	ASCII
// P2	Portable graymap	ASCII
// P3	Portable pixmap	ASCII
// P4	Portable bitmap	Binary
// P5	Portable graymap	Binary
// P6	Portable pixmap	Binary

/////////////////////////////////////////////////////////////////////////////
// Save Volume types
/////////////////////////////////////////////////////////////////////////////

template<typename T, typename Manage>
void SavePXM(std::ofstream& bFile, const roo::VolumeGrid<T,roo::TargetHost,Manage>& vol, std::string ppm_type = "P5", int num_colors = 255)
{
  bFile << ppm_type << std::endl;
  bFile << vol.w << " " << vol.h << " " << vol.d << '\n';
  bFile << num_colors << '\n';
  for(unsigned int d=0; d<vol.d; ++d) {
    for(unsigned int r=0; r<vol.h; ++r) {
      bFile.write( (const char*)vol.RowPtr(r,d), vol.w * sizeof(T) );
    }
  }
  bFile.close();
}

template<typename T, typename Manage>
void SavePXM(const std::string filename, const roo::VolumeGrid<T,roo::TargetHost,Manage>& vol, std::string ppm_type = "P5", int num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile, vol, ppm_type, num_colors);
}

template<typename T, typename Manage>
void SavePXM(std::ofstream& bFile, const roo::VolumeGrid<T,roo::TargetDevice,Manage>& vol, std::string ppm_type = "P5", int num_colors = 255)
{
  roo::VolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.InitVolume(vol.w, vol.h, vol.d);
  hvol.CopyFrom(vol);
  SavePXM(bFile, hvol, ppm_type, num_colors);
}



template<typename T, typename Manage>
void SavePXM(const std::string filename, const roo::VolumeGrid<T,roo::TargetDevice,Manage>& vol, std::string ppm_type = "P5", int num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile,vol,ppm_type,num_colors);
}



template<typename T, typename Manage>
void SavePXM(const std::string filename, const roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>& vol, std::string ppm_type = "P5", int num_colors = 255)
{
  // save each active volume in BoundedVolumeGrid
  for(int i=0;i!=vol.m_nWholeGridRes*vol.m_nWholeGridRes*vol.m_nWholeGridRes;i++)
  {
    if(vol.CheckIfBasicSDFActive(i)==true)
    {
      // save
      string sFileName = filename + "-" + std::to_string(i);
      std::ofstream bFile( sFileName.c_str(), std::ios::out | std::ios::binary );
      SavePXM<T,Manage>(bFile,vol.m_GridVolumes[i],ppm_type,num_colors);
      std::cout<<"[Kangaroo/SavePXMGrid] save grid sdf "<<sFileName<<" success."<<endl;
    }
  }
}




/////////////////////////////////////////////////////////////////////////////
// Load Volume types
/////////////////////////////////////////////////////////////////////////////

template<typename T>
bool LoadPXM(std::ifstream& bFile, roo::VolumeGrid<T,roo::TargetHost,roo::Manage>& vol)
{
  // Parse header

  std::string ppm_type = "";
  int num_colors = 0;
  int w = 0;
  int h = 0;
  int d = 0;

  bFile >> ppm_type;
  bFile >> w;
  bFile >> h;
  bFile >> d;
  bFile >> num_colors;
  bFile.ignore(1,'\n');

  bool success = !bFile.fail() && w > 0 && h > 0 && d > 0;

  if(success) {
    // Make sure vol is empty
    roo::Manage::Cleanup<T,roo::TargetHost>(vol.ptr);

    // Allocate memory
    roo::TargetHost::AllocatePitchedMem<T>(&vol.ptr,&vol.pitch,&vol.img_pitch,w,h,d);
    vol.w = w; vol.h = h; vol.d = d;

    // Read in data
    for(size_t d=0; d<vol.d; ++d) {
      for(size_t r=0; r<vol.h; ++r) {
        bFile.read( (char*)vol.RowPtr(r,d), vol.w * sizeof(T) );
      }
    }
    success = !bFile.fail();
  }
  bFile.close();

  return success;
}

template<typename T>
bool LoadPXM(const std::string filename, roo::VolumeGrid<T,roo::TargetHost,roo::Manage>& vol)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  return LoadPXM<T>(bFile,vol);
}

template<typename T>
bool LoadPXM(const std::string filename, roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage>& vol)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  //read in the bounding volume bounds
  bFile >> vol.m_bbox.boxmin.x;
  bFile >> vol.m_bbox.boxmin.y;
  bFile >> vol.m_bbox.boxmin.z;
  bFile >> vol.m_bbox.boxmax.x;
  bFile >> vol.m_bbox.boxmax.y;
  bFile >> vol.m_bbox.boxmax.z;
  bFile.ignore(1,'\n');
  return LoadPXM<T>(bFile,vol);
}

template<typename T>
bool LoadPXM(const std::string filename, roo::VolumeGrid<T,roo::TargetDevice,roo::Manage>& vol)
{
  roo::VolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  bool success = LoadPXM(filename, hvol);

  if(success) {
    roo::Manage::Cleanup<T,roo::TargetDevice>(vol.ptr);

    roo::TargetDevice::AllocatePitchedMem<T>(&vol.ptr,&vol.pitch,&vol.img_pitch,hvol.w,hvol.h,hvol.d);
    vol.w = hvol.w; vol.h = hvol.h; vol.d = hvol.d;

    vol.CopyFrom(hvol);
  }
  return success;
}

template<typename T>
bool LoadPXM(const std::string filename, roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>& vol)
{
  roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage> hvol;
  bool success = LoadPXM(filename, hvol);

  if(success) {
    //        roo::Manage::Cleanup<T,roo::TargetDevice>(vol.ptr);

    //        roo::TargetDevice::AllocatePitchedMem<T>(&vol.ptr,&vol.pitch,&vol.img_pitch,hvol.m_w,hvol.m_h,hvol.m_d);
    vol.m_w = hvol.m_w; vol.m_h = hvol.m_h; vol.m_d = hvol.m_d;
    printf("Try to load BoundedVolumeGrid\n");
    vol.CopyAndInitFrom(hvol);
    vol.m_bbox = hvol.m_bbox;
  }
  return success;
}


#endif // SAVEPPMGRID_H
