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
void SavePXMGrid(const std::string filename, const roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>& vol, std::string ppm_type = "P5", int num_colors = 255)
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
    }
  }
}


#endif // SAVEPPMGRID_H
