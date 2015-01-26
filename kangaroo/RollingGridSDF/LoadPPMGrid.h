#ifndef LOADPPMGRID_H
#define LOADPPMGRID_H

#include <fstream>
#include <iostream>
#include <kangaroo/Image.h>
#include <kangaroo/Volume.h>
#include <kangaroo/BoundedVolume.h>
#include "BoundedVolumeGrid.h"

KANGAROO_EXPORT
roo::BoundingBox LoadPXMBoundingBox(std::string filename);

KANGAROO_EXPORT
template<typename T>
bool LoadPXMSingleGrid(
    const std::string                                      sFilename,
    roo::VolumeGrid<T,roo::TargetHost,roo::Manage>&        vol)
{
  std::ifstream bFile( sFilename.c_str(), std::ios::in | std::ios::binary );

  // Parse header
  std::string ppm_type = "P5";
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

  if(success)
  {
    // Make sure the vol is empty
    //    roo::Manage::Cleanup<T,roo::TargetHost>(vol.ptr);
    GpuCheckErrors();

    // init volume grid
    vol.InitVolume(w, h, d);
    GpuCheckErrors();

    // Read in data
    for(size_t d=0; d<vol.d; ++d)
    {
      for(size_t r=0; r<vol.h; ++r)
      {
        bFile.read( (char*)vol.RowPtr(r,d), vol.w * sizeof(T) );
      }
    }
    success = !bFile.fail();
  }
  else
  {
    std::cerr<<"[LoadPXMSingleGrid] Error! Read "<<sFilename<<" Fail!"<<std::endl;
    exit(-1);
  }

  bFile.close();
  return success;
}

KANGAROO_EXPORT
template<typename T>
bool LoadPXMGrid(
    std::string                                               sDirName,
    const std::vector<std::string>&                           vfilename,
    std::string                                               sBBFileName,
    roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>&  rDVol)
{
  // ---------------------------------------------------------------------------
  // to load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hVol;
  roo::BoundingBox BBox = LoadPXMBoundingBox(sDirName + sBBFileName);

  // ---------------------------------------------------------------------------
  // init sdf
  hVol.Init(rDVol.m_w, rDVol.m_h, rDVol.m_d, rDVol.m_nVolumeGridRes, BBox);
  std::cout<<"[LoadPXMGrid] Finish init sdf. loading model files from the hard disk."<<std::endl;

  // ---------------------------------------------------------------------------
  // load each single VolumeGrid
  for(unsigned int i=0;i!=vfilename.size();i++)
  {
    // get index from file name
    std::string sFileName = vfilename[i];
    std::string sIndex = sFileName.substr(
          sFileName.find_last_of("-")+1,
          sFileName.size() - sFileName.find_last_of("-"));

    if(sIndex!="BB")
    {
      int nIndex = std::atoi(sIndex.c_str());

      if(LoadPXMSingleGrid(sDirName+ sFileName, hVol.m_GridVolumes[nIndex]) == false)
      {
        std::cout<<"[LoadPXMGrid] Error! cannot read single volume grid "<<
                   sFileName<< " with index "<<nIndex<<" from hard disk."<<std::endl;

        return false;
      }
    }
  }

  std::cout<<"[LoadPXMGrid] Copying data from the host memory to the device memory."<<std::endl;

  // ---------------------------------------------------------------------------
  // copy data from host to device.
  /// somehow we need to copy the host data to a empty target device vol first, and
  /// copy data from the temp device vol to the target device vol
  roo::BoundedVolumeGrid<T, roo::TargetDevice, roo::Manage> NewDVol;
  NewDVol.Init(rDVol.m_w, rDVol.m_h, rDVol.m_d, rDVol.m_nVolumeGridRes, BBox);
  NewDVol.CopyAndInitFrom(hVol);

  std::cout<<"copying data to the device memory.."<<std::endl;
  // copy data from device to device
  rDVol.CopyAndInitFrom(NewDVol);
  std::cout<<"[LoadPXMGrid] Finish Copy data from host to device."<<std::endl;
  return true;
}

#endif // LOADPPMGRID_H
