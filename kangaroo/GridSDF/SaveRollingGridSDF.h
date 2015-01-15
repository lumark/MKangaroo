#ifndef SAVEROLLINGGRIDSDF_H
#define SAVEROLLINGGRIDSDF_H

#include "MarchingCubesGrid.h"
#include "LoadPPMGrid.h"
#include "SavePPMGrid.h"
#include "SaveMeshGrid.h"
#include "cu_sdf_reset.h"

namespace roo {

//////////////////////////////////////////
// save grid mesh
//////////////////////////////////////////
/// \brief The SingleVolume class
/// a single volume in a global pos, may contain several grids
class SingleVolume
{
public:
  int3                            GlobalIndex;
  std::vector<int3>               vLocalIndex;
  std::vector<std::string>        vFileName;
  std::string                     sBBoxFileName;
};

bool GetIndexFromFileName(
    std::string                   sFileName,
    int3&                         GlobalIndex,
    int3&                         LocalIndex );

std::vector<SingleVolume> GetFilesNeedSaving(
    std::vector<std::string>&     vfilename);

bool SaveMeshFromPXMs(
    std::string                   sDirName,
    std::string                   sBBFileHead,
    int3                          nVolRes,
    int                           nGridRes,
    std::vector<std::string>      vfilename,
    std::string                   sMeshFileName);

}

#endif // SAVEROLLINGGRIDSDF_H
