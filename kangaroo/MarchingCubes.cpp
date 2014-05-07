// Marching cubes ASSIMP exporter based on Marching Cubes Example Program
// by Cory Bloyd with additional source from Paul Bourke (public domain)
// http://paulbourke.net/geometry/polygonise/
//
// Marching Cubes Example Program
// by Cory Bloyd (corysama@yahoo.com)
//
// A simple, portable and complete implementation of the Marching Cubes
// and Marching Tetrahedrons algorithms in a single source file.
// There are many ways that this code could be made faster, but the
// intent is for the code to be easy to understand.
//
// For a description of the algorithm go to
// http://astronomy.swin.edu.au/pbourke/modelling/polygonise/
//
// This code is public domain.
//

#include "MarchingCubes.h"


namespace roo
{


void SaveMesh(std::string filename, aiMesh* mesh)
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
    scene.mMeshes[0] = mesh;
    scene.mNumMaterials = 1;
    scene.mMaterials = new aiMaterial*[scene.mNumMaterials];
    scene.mMaterials[0] = material;

    aiReturn res = aiExportScene(&scene, "ply", (filename + ".ply").c_str(), 0);
    std::cout << "Mesh export result: " << res << std::endl;
}

template<typename T, typename TColor>
void SaveMesh(std::string filename, const BoundedVolume<T,TargetHost> vol, const BoundedVolume<TColor,TargetHost> volColor )
{
    std::vector<aiVector3D> verts;
    std::vector<aiVector3D> norms;
    std::vector<aiFace> faces;
    std::vector<aiColor4D> colors;

    for(GLint iX = 0; iX < vol.Voxels().x-1; iX++) {
        for(GLint iY = 0; iY < vol.Voxels().y-1; iY++) {
            for(GLint iZ = 0; iZ < vol.Voxels().z-1; iZ++) {
                vMarchCube(vol, volColor, iX,iY,iZ, verts, norms, faces, colors);
            }
        }
    }

    aiMesh* mesh = MeshFromLists(verts,norms,faces,colors);
    SaveMesh(filename, mesh);
}

// Instantiate templates
template void SaveMesh<SDF_t,float>(std::string, const BoundedVolume<SDF_t,TargetHost,DontManage> vol, const BoundedVolume<float,TargetHost> volColor);

}
