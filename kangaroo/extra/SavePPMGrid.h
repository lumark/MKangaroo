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
void SavePXM(std::ofstream& bFile, const roo::VolumeGrid<T,roo::TargetHost,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
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
void SavePXM(const std::string filename, const roo::VolumeGrid<T,roo::TargetHost,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile, vol, ppm_type, num_colors);
}


template<typename T, typename Manage>
void SavePXM(std::ofstream& bFile, const roo::VolumeGrid<T,roo::TargetDevice,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  roo::VolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.InitVolume(vol.w, vol.h, vol.d);
  hvol.CopyFrom(vol);
  SavePXM(bFile, hvol, ppm_type, num_colors);
}


template<typename T, typename Manage>
void SavePXM(const std::string filename, const roo::VolumeGrid<T,roo::TargetDevice,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile,vol,ppm_type,num_colors);
}


inline void SavePXMBoundingBox(const std::string filename, roo::BoundingBox BBox)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  bFile << BBox.boxmin.x << " " <<  BBox.boxmin.y << " " << BBox.boxmin.z << std::endl;
  bFile << BBox.boxmax.x << " " <<  BBox.boxmax.y << " " << BBox.boxmax.z << std::endl;

  printf("save bb success File: %s\n",filename.c_str());
  bFile.close();
}


template<typename T, typename Manage>
void SavePXM(const std::string                                      filename,
             roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   rDVol,
             bool                                                   bGlobalPose = false,
             std::string                                            ppm_type = "P5",
             int                                                    num_colors = 255)
{
  if(rDVol.GetActiveGridVolNum()==0)
  {
    std::cerr<<"[Kangaroo/SavePXMGrid] Cannot save model for void volume. Empty vol."<<std::endl;
  }
  else
  {
    // load data from device to host
    roo::BoundedVolumeGrid<T,roo::TargetHost, Manage> hvol;
    hvol.init(rDVol.m_w,rDVol.m_h,rDVol.m_d,rDVol.m_nVolumeGridRes,rDVol.m_bbox);
    hvol.CopyAndInitFrom(rDVol);
    hvol.m_global_shift = rDVol.m_global_shift;

    // First save bounding box to HardDisk
    std::string sBBFileName = filename+"-BB";
    SavePXMBoundingBox(sBBFileName, rDVol.m_bbox);

    // Second save each active volume in BoundedVolumeGrid to HardDisk
    int nNum =0;

    for(int i=0;i!=rDVol.m_nWholeGridRes_w;i++)
    {
      for(int j=0;j!=rDVol.m_nWholeGridRes_h;j++)
      {
        for(int k=0;k!=rDVol.m_nWholeGridRes_d;k++)
        {
          if(hvol.CheckIfBasicSDFActive(hvol.GetIndex(i,j,k))==true)
          {
            // save local index (without rolling)
            std::string sFileName;
            if(bGlobalPose==false)
            {
              // only save local grid
              sFileName = filename+"-"+std::to_string(i)+"-"+std::to_string(j)+"-"+std::to_string(k);
            }
            else
            {
              // save global index (with rolling)
              sFileName =filename+"-"+std::to_string(hvol.m_global_shift.x)+
                  "-"+std::to_string(hvol.m_global_shift.y)+
                  "-"+std::to_string(hvol.m_global_shift.z)+
                  "-"+std::to_string(i)+"-"+std::to_string(j)+"-"+std::to_string(k);
            }

            std::ofstream bFile( sFileName.c_str(), std::ios::out | std::ios::binary );
            SavePXM<T,Manage>(bFile,hvol.m_GridVolumes[i*j*k],ppm_type,num_colors);

            std::cout<<"[Kangaroo/SavePXMGrid] Save "<<sFileName<<" success."<<std::endl;
            nNum++;
          }
        }
      }
    }

    printf("[Kangaroo/SavePXMGrid] Save %d grid sdf.\n", nNum);
  }

}





/////////////////////////////////////////////////////////////////////////////
//                           Load Volume types.
/////////////////////////////////////////////////////////////////////////////

template<typename T>
bool LoadPXMSingleGrid(const std::string filename,
                       roo::VolumeGrid<T,roo::TargetHost,roo::Manage>& vol)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );

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

  if(success) {
    // init volume grid
    vol.InitVolume(w,h,d);
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
    bFile.close();
    return false;
  }

  bFile.close();
  return true;
}

inline roo::BoundingBox LoadPXMBoundingBox(std::string filename)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  if(bFile.fail()==true)
  {
    std::cerr<<"fatal error! file "<<filename<<" does not exist."<<std::endl;
    exit(-1);
  }

  //read in the bounding volume bounds
  roo::BoundingBox BBox;

  bFile >> BBox.boxmin.x;
  bFile >> BBox.boxmin.y;
  bFile >> BBox.boxmin.z;
  bFile >> BBox.boxmax.x;
  bFile >> BBox.boxmax.y;
  bFile >> BBox.boxmax.z;
  bFile.ignore(1,'\n');

  printf("load bounding box success. Min_x:%f,Max_x: %f\n",BBox.boxmin.x,BBox.boxmax.x);
  bFile.close();
  return BBox;
}


template<typename T>
bool LoadPXMGrid(std::string                        sDirName,
                 const std::vector<std::string>&    vfilename,
                 std::string                        sBBFileName,
                 roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>& vol)
{
  // to load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;

  roo::BoundingBox BBox = LoadPXMBoundingBox(sDirName+sBBFileName);

  // init sdf
  hvol.init(vol.m_w, vol.m_h,vol.m_d,vol.m_nVolumeGridRes,BBox);

  // read bb box..
  int nNum = 0;

  // load each single VolumeGrid
  for(int i=0;i!=vfilename.size();i++)
  {
    // get index from file name
    std::string sFileName = vfilename[i];
    std::string sIndex = sFileName.substr(sFileName.find_last_of("-")+1,
                                          sFileName.size() - sFileName.find_last_of("-"));

    if(sIndex!="BB")
    {
      int nIndex = std::atoi(sIndex.c_str());

      if(LoadPXMSingleGrid(sDirName+ sFileName, hvol.m_GridVolumes[nIndex]) == false)
      {
        std::cout<<"[LoadPXMGrid] Fatal error! cannot read single volume grid "<<sFileName<<
                   " with index "<<nIndex<<" from hard disk."<<std::endl;
        exit(-1);
      }
      else
      {
        nNum ++;
      }
    }
  }

  // copy data from host to device
  vol.CopyAndInitFrom(hvol);
  GpuCheckErrors();

  printf("[LoadPXMGrid] Finish load %d data to device. Available memory is %d\n",nNum, GetAvailableGPUMemory());

  return true;
}


#endif // SAVEPPMGRID_H
