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
  printf("[SavePXM] saving file...\n");
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
bool LoadPXMSingleGrid(const std::string filename, roo::VolumeGrid<T,roo::TargetHost,roo::Manage>& vol)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );

  // Parse header
  std::string ppm_type = "P5";
  int num_colors = 255;
  int w = 32;
  int h = 32;
  int d = 32;

  bFile >> ppm_type;
  bFile >> w;
  bFile >> h;
  bFile >> d;
  bFile >> num_colors;
  bFile.ignore(1,'\n');

  bool success = !bFile.fail() && w > 0 && h > 0 && d > 0;

  if(success) {
//    vol.w = w; vol.h = h; vol.d = d;

    // Read in data
    for(size_t d=0; d<vol.d; ++d) {
      for(size_t r=0; r<vol.h; ++r) {
        bFile.read( (char*)vol.RowPtr(r,d), vol.w * sizeof(T) );
      }
    }
    success = !bFile.fail();
  }
  else
  {
    bFile.close();
    return false;
  }

  bFile.close();
  return true;
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
bool LoadPXMGrid(std::string sDirName, const std::vector<std::string>&    vfilename,
                 roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>& vol,
                 roo::BoundingBox&                                        rBBox)
{
  // to load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;

  // init sdf
  hvol.init(256,256,256,32,rBBox);

  // read bb box..
  int nNum = 0;

  printf("[LoadPXMGrid] Try to copy data from disk to host.., available gpu memory is %d.\n", GetAvailableGPUMemory());

  // load each single VolumeGrid
  for(int i=0;i!=vfilename.size();i++)
  {
    // get index from file name
    std::string sFileName = vfilename[i];
    std::string sIndex = sFileName.substr(sFileName.find_last_of("-")+1, sFileName.size() - sFileName.find_last_of("-"));

    int nIndex = std::atoi(sIndex.c_str());

    // init and read grid sdf
    if(hvol.InitSingleBasicSDFWithIndex(nIndex) == true)
    {
      if(LoadPXMSingleGrid(sDirName+ sFileName, hvol.m_GridVolumes[i]) == false)
      {
        std::cout<<"[LoadPXMGrid] Fatal error! cannot read single volume grid "<<sFileName<<" with index "<<nIndex<<" from hard disk."<<endl;
        exit(-1);
      }
      else
      {
        nNum ++;
      }
    }
    else
    {
      std::cout<<"fatal error! cannot init single volume grid "<<sFileName<<" with index "<<nIndex<<" whole grid res is "<<hvol.m_nWholeGridRes<<endl;
      exit(-1);
    }
  }

  //
  printf("finish read data to host. Total loading sdf num is %d, Available GPU memory is %d, Now copy it to device..\n",nNum, GetAvailableGPUMemory());

  // copy data from host to device
  vol.CopyAndInitFrom(hvol);
  GpuCheckErrors();

  printf("finish read data to device. Available memory is %d\n",GetAvailableGPUMemory());

  return true;
}


#endif // SAVEPPMGRID_H
