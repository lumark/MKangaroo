#pragma once

#include <sophus/se3.hpp>
#include <kangaroo/Sdf.h>
#include <kangaroo/extra/SaveGIL.h>
#include <kangaroo/GridSDF/MarchingCubesGrid.h>
#include <kangaroo/GridSDF/SavePPMGrid.h>
#include <boost/ptr_container/ptr_vector.hpp>

KANGAROO_EXPORT
inline void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t, roo::TargetDevice, roo::Manage>& vol)
{
  Eigen::Matrix3d RDFvision;  RDFvision  << 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFmeshlab; RDFmeshlab << 1,0,0,  0,-1,0, 0,0,-1;
  Eigen::Matrix4d T_vis_ml = Eigen::Matrix4d::Identity();
  T_vis_ml.block<3,3>(0,0) = RDFvision.transpose() * RDFmeshlab;
  Eigen::Matrix4d T_ml_vis = Eigen::Matrix4d::Identity();
  T_ml_vis.block<3,3>(0,0) = RDFmeshlab.transpose() * RDFvision;

  std::string mesh_filename = "mesh";
  std::ofstream of("project.mlp");

  of << "<!DOCTYPE MeshLabDocument>" << std::endl;
  of << "<MeshLabProject>" << std::endl;

  of << " <MeshGroup>" << std::endl;

  roo::SaveMeshGrid(mesh_filename, vol);

  of << "  <MLMesh label=\"mesh.ply\" filename=\"" << mesh_filename <<
        ".ply\">" << std::endl;

  of << "   <MLMatrix44>" << std::endl;
  of << "1 0 0 0 " << std::endl;
  of << "0 1 0 0 " << std::endl;
  of << "0 0 1 0 " << std::endl;
  of << "0 0 0 1 " << std::endl;
  of << "</MLMatrix44>" << std::endl;
  of << "  </MLMesh>" << std::endl;

  of << " </MeshGroup>" << std::endl;

  of << " <RasterGroup>" << std::endl;

  of << " </RasterGroup>" << std::endl;

  of << "</MeshLabProject> " << std::endl;

  of.close();
}

KANGAROO_EXPORT
inline void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t_Smart, roo::TargetDevice, roo::Manage>& vol)
{
  Eigen::Matrix3d RDFvision;  RDFvision  << 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFmeshlab; RDFmeshlab << 1,0,0,  0,-1,0, 0,0,-1;
  Eigen::Matrix4d T_vis_ml = Eigen::Matrix4d::Identity();
  T_vis_ml.block<3,3>(0,0) = RDFvision.transpose() * RDFmeshlab;
  Eigen::Matrix4d T_ml_vis = Eigen::Matrix4d::Identity();
  T_ml_vis.block<3,3>(0,0) = RDFmeshlab.transpose() * RDFvision;

  std::string mesh_filename = "mesh";
  std::ofstream of("project.mlp");

  of << "<!DOCTYPE MeshLabDocument>" << std::endl;
  of << "<MeshLabProject>" << std::endl;

  of << " <MeshGroup>" << std::endl;

  roo::SaveMeshGrid(mesh_filename, vol);

  of << "  <MLMesh label=\"mesh.ply\" filename=\"" << mesh_filename <<
        ".ply\">" << std::endl;

  of << "   <MLMatrix44>" << std::endl;
  of << "1 0 0 0 " << std::endl;
  of << "0 1 0 0 " << std::endl;
  of << "0 0 1 0 " << std::endl;
  of << "0 0 0 1 " << std::endl;
  of << "</MLMatrix44>" << std::endl;
  of << "  </MLMesh>" << std::endl;

  of << " </MeshGroup>" << std::endl;

  of << " <RasterGroup>" << std::endl;

  of << " </RasterGroup>" << std::endl;

  of << "</MeshLabProject> " << std::endl;

  of.close();
}

KANGAROO_EXPORT
inline void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t, roo::TargetDevice, roo::Manage>& vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>& GreyVol)
{
  Eigen::Matrix3d RDFvision;  RDFvision  << 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFmeshlab; RDFmeshlab << 1,0,0,  0,-1,0, 0,0,-1;
  Eigen::Matrix4d T_vis_ml = Eigen::Matrix4d::Identity();
  T_vis_ml.block<3,3>(0,0) = RDFvision.transpose() * RDFmeshlab;
  Eigen::Matrix4d T_ml_vis = Eigen::Matrix4d::Identity();
  T_ml_vis.block<3,3>(0,0) = RDFmeshlab.transpose() * RDFvision;

  std::string mesh_filename = "mesh";
  std::ofstream of("project.mlp");

  of << "<!DOCTYPE MeshLabDocument>" << std::endl;
  of << "<MeshLabProject>" << std::endl;

  of << " <MeshGroup>" << std::endl;

  roo::SaveMeshGrid(mesh_filename, vol, GreyVol);

  of << "  <MLMesh label=\"mesh.ply\" filename=\"" << mesh_filename <<
        ".ply\">" << std::endl;

  of << "   <MLMatrix44>" << std::endl;
  of << "1 0 0 0 " << std::endl;
  of << "0 1 0 0 " << std::endl;
  of << "0 0 1 0 " << std::endl;
  of << "0 0 0 1 " << std::endl;
  of << "</MLMatrix44>" << std::endl;
  of << "  </MLMesh>" << std::endl;

  of << " </MeshGroup>" << std::endl;

  of << " <RasterGroup>" << std::endl;

  of << " </RasterGroup>" << std::endl;

  of << "</MeshLabProject> " << std::endl;

  of.close();
}

KANGAROO_EXPORT
inline void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t_Smart, roo::TargetDevice, roo::Manage>& vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>& GreyVol)
{
  Eigen::Matrix3d RDFvision;  RDFvision  << 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFmeshlab; RDFmeshlab << 1,0,0,  0,-1,0, 0,0,-1;
  Eigen::Matrix4d T_vis_ml = Eigen::Matrix4d::Identity();
  T_vis_ml.block<3,3>(0,0) = RDFvision.transpose() * RDFmeshlab;
  Eigen::Matrix4d T_ml_vis = Eigen::Matrix4d::Identity();
  T_ml_vis.block<3,3>(0,0) = RDFmeshlab.transpose() * RDFvision;

  std::string mesh_filename = "mesh";
  std::ofstream of("project.mlp");

  of << "<!DOCTYPE MeshLabDocument>" << std::endl;
  of << "<MeshLabProject>" << std::endl;

  of << " <MeshGroup>" << std::endl;

  double dTime = roo::_Tic();
  roo::SaveMeshGrid(mesh_filename, vol, GreyVol);
  std::cout<<"save model use time "<<roo::_Toc(dTime)<<std::endl;

  of << "  <MLMesh label=\"mesh.ply\" filename=\"" << mesh_filename <<
        ".ply\">" << std::endl;

  of << "   <MLMatrix44>" << std::endl;
  of << "1 0 0 0 " << std::endl;
  of << "0 1 0 0 " << std::endl;
  of << "0 0 1 0 " << std::endl;
  of << "0 0 0 1 " << std::endl;
  of << "</MLMatrix44>" << std::endl;
  of << "  </MLMesh>" << std::endl;

  of << " </MeshGroup>" << std::endl;

  of << " <RasterGroup>" << std::endl;

  of << " </RasterGroup>" << std::endl;

  of << "</MeshLabProject> " << std::endl;

  of.close();
}

KANGAROO_EXPORT
// generate a single mesh from several PPMs
inline void SaveMeshlabFromPPMs(
    std::string                 sDirName,
    std::string                 sBBFileHead,
    int3                        VolRes,
    int                         nGridRes,
    std::vector<std::string>    vfilename,
    std::string                 sFinalMeshFileName)
{
  Eigen::Matrix3d RDFvision;  RDFvision  << 1,0,0,  0,1,0,  0,0,1;
  Eigen::Matrix3d RDFmeshlab; RDFmeshlab << 1,0,0,  0,-1,0, 0,0,-1;
  Eigen::Matrix4d T_vis_ml = Eigen::Matrix4d::Identity();
  T_vis_ml.block<3,3>(0,0) = RDFvision.transpose() * RDFmeshlab;
  Eigen::Matrix4d T_ml_vis = Eigen::Matrix4d::Identity();
  T_ml_vis.block<3,3>(0,0) = RDFmeshlab.transpose() * RDFvision;

  std::string mesh_filename = "mesh";
  std::ofstream of("project.mlp");

  of << "<!DOCTYPE MeshLabDocument>" << std::endl;
  of << "<MeshLabProject>" << std::endl;

  of << " <MeshGroup>" << std::endl;

  roo::SaveMeshFromPPMs(sDirName, sBBFileHead, VolRes, nGridRes, vfilename, sFinalMeshFileName);

  of << "  <MLMesh label=\"mesh.ply\" filename=\"" << mesh_filename << ".ply\">" << std::endl;
  of << "   <MLMatrix44>" << std::endl;
  of << "1 0 0 0 " << std::endl;
  of << "0 1 0 0 " << std::endl;
  of << "0 0 1 0 " << std::endl;
  of << "0 0 0 1 " << std::endl;
  of << "</MLMatrix44>" << std::endl;
  of << "  </MLMesh>" << std::endl;

  of << " </MeshGroup>" << std::endl;

  of << " <RasterGroup>" << std::endl;

  of << " </RasterGroup>" << std::endl;

  of << "</MeshLabProject> " << std::endl;

  of.close();
}
