#include "SaveMeshGrid.h"
#include "boost/scoped_ptr.hpp"

namespace roo
{

aiMesh* MeshFromListsVector(
    const std::vector<aiVector3D>&                    verts,
    const std::vector<aiVector3D>&                    norms,
    const std::vector<aiFace>&                        faces,
    const std::vector<aiColor4D>&                     colors
    )
{
  aiMesh* mesh = new aiMesh();
  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;

  mesh->mNumVertices = verts.size();
  mesh->mVertices = new aiVector3D[verts.size()];
  for(unsigned int i=0; i < verts.size(); ++i) {
    mesh->mVertices[i] = verts[i];
  }

  if(norms.size() == verts.size()) {
    mesh->mNormals = new aiVector3D[norms.size()];
    for(unsigned int i=0; i < norms.size(); ++i) {
      mesh->mNormals[i] = norms[i];
    }
  }else{
    mesh->mNormals = 0;
  }

  mesh->mNumFaces = faces.size();
  mesh->mFaces = new aiFace[faces.size()];
  for(unsigned int i=0; i < faces.size(); ++i) {
    mesh->mFaces[i] = faces[i];
  }

  if( colors.size() == verts.size()) {
    mesh->mColors[0] = new aiColor4D[colors.size()];
    for(unsigned int i=0; i < colors.size(); ++i) {
      mesh->mColors[0][i] = colors[i];
    }
  }

  std::cout<<"[MeshFromLists] Finished. verts num "<<verts.size()<<", norms num: "<<norms.size()<<
             ", faces num "<<faces.size()<<", color num: "<< colors.size()<<std::endl;

  return mesh;
}

bool SaveMeshGridToFile(
    std::string                                       sFilename,
    aiMesh*                                           pMesh,
    std::string                                       sFormat)
{
  // Create root node which indexes first mesh
  aiNode* root = new aiNode();
  root->mNumMeshes = 1;
  root->mMeshes = new unsigned int[root->mNumMeshes];
  root->mMeshes[0] = 0;
  root->mName = "root";

  aiMaterial* material = new aiMaterial();

  // Create scene to contain root node and mesh
  aiScene scene;
  scene.mRootNode = root;
  scene.mNumMeshes = 1;
  scene.mMeshes = new aiMesh*[scene.mNumMeshes];
  scene.mMeshes[0] = pMesh;
  scene.mNumMaterials = 1;
  scene.mMaterials = new aiMaterial*[scene.mNumMaterials];
  scene.mMaterials[0] = material;

  std::cout<<"[SaveMeshGridToFile] scene has vertex color: "<< scene.mMeshes[0]->HasVertexColors(0)<<std::endl;

  sFilename = sFilename + "." + sFormat;
  aiReturn res = aiExportScene(&scene, sFormat.c_str(), sFilename.c_str(), 0);

  if(res == 0)
  {
    std::cout << "[SaveMeshGridToFile] Mesh export success. File Name "<< sFilename <<std::endl;
    return true;
  }
  else
  {
    std::cerr << "[SaveMeshGridToFile] Mesh export fail." << std::endl;
    return false;
  }

  return true;
}


bool SaveMeshGridToFileAssimp(
    std::string                                       sFilename,
    aiMesh*                                           pMesh,
    std::string                                       sFormat)
{
  // Create root node which indexes first mesh
  aiNode* root = new aiNode();
  root->mNumMeshes = 1;
  root->mMeshes = new unsigned int[root->mNumMeshes];
  root->mMeshes[0] = 0;
  root->mName = "root";

  aiMaterial* material = new aiMaterial();

  // Create scene to contain root node and mesh
  aiScene scene;
  scene.mRootNode = root;
  scene.mNumMeshes = 1;
  scene.mMeshes = new aiMesh*[scene.mNumMeshes];
  scene.mMeshes[0] = pMesh;
  scene.mNumMaterials = 1;
  scene.mMaterials = new aiMaterial*[scene.mNumMaterials];
  scene.mMaterials[0] = material;

  std::cout<<"[SaveMeshGridToFile] scene has vertex color: "<< scene.mMeshes[0]->HasVertexColors(0)<<std::endl;

  sFilename = sFilename + "." + sFormat;
  aiReturn res = aiExportScene(&scene, sFormat.c_str(), sFilename.c_str(), 0);

  if(res == 0)
  {
    std::cout << "[SaveMeshGridToFile] Mesh export success. File Name "<< sFilename <<std::endl;
    return true;
  }
  else
  {
    std::cerr << "[SaveMeshGridToFile] Mesh export fail." << std::endl;
    return false;
  }

  return true;
}

template void SaveMeshGrid<roo::SDF_t_Smart, float, Manage>
(
std::string,
BoundedVolumeGrid<SDF_t_Smart, TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor
);

}
