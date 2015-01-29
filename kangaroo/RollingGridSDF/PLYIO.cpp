/**************************************
* Copyright (c) <2014> <Vishesh Gupta> *
**************************************

The MIT / X Window System License
=================================
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


// modify by lu.ma@colorado.edu

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include "PLYIO.h"

using namespace glm;

PLYModel::PLYModel()
{
}

void PLYModel::ReadStarnardPLY(
    const char* filename,
    bool isNormal,
    bool isColor) {

  std::cout<<"[ReadStarnardPLY] Start Reading "<<filename<<std::endl;
  struct faceData F;
  isMesh=0;

  std::string line;
  std::string s1, s2;

  std::ifstream inputPly;
  inputPly.open(filename,std::ios::binary);

  if (!inputPly.is_open()) {
    std::cerr << "Couldn't open " << filename << '\n';
    exit(1);
  }

  ifColor=0;
  ifNormal=0;

  std::getline(inputPly, line);
  while(line.compare("vertex")!=0)
  {
    std::getline(inputPly, line,' ');
    std::cout<<line<<"\n";
  }

  inputPly>>vertexCount; // 3rd line
  std::cout<<vertexCount;

  //getline(inputPly,line);
  //bool visit=0;
  while(line.compare("face")!=0)
  {
    if(line.find("red")==0 || line.find("diffuse_red") == 0) ifColor=1;
    if(line.find("nx")==0) ifNormal=1;
    std::getline(inputPly, line,' ');
    std::cout<<line<<std::endl;
  }

  inputPly>>faceCount;
  std::cout<<faceCount;
  if(faceCount>0)
    isMesh=1;

  while(line.compare("end_header")!=0)
  {
    std::getline(inputPly, line);
    std::cout<<line<<std::endl;
  }

  // This runs if the file has both COLOR and NORMAL information
  if(isNormal && isColor)
  {
    struct data Values;

    if(!ifColor)
    {
      std::cout<<"\nFile doesn't contain the Color Information, check the constructor call";
      exit(0);
    }
    if(!ifNormal)
    {
      std::cout<<"\nFile doesn't contain the Normal Information, check the constructor call";
      exit(0);
    }

    inputPly.read((char *)&Values,sizeof(Values));

    std::cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<<
               Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<<
               (int)Values._r <<"\t"<< (int)Values._g <<"\t"<< (int)Values._b <<"\t"<< (int)Values._a<<std::endl;

    min = max = vec3(Values._x, Values._y, Values._z);

    positions.push_back(vec3(Values._x, Values._y, Values._z));
    normals.push_back(vec3(Values._nx, Values._ny, Values._nz));
    colors.push_back(vec3(Values._r, Values._g, Values._b) / 255.0f);

    for (long int i = 1; i < vertexCount; i++)
    {
      inputPly.read((char *)&Values,sizeof(Values));

      //      cout<<"\n"<<Values._x <<", "<< Values._y <<", "<< Values._z <<
      //            ", "<< Values._nx <<", "<< Values._ny <<", "<< Values._nz <<
      //            ", "<< (int)Values._r <<", "<< (int)Values._g <<", "<< (int)Values._b <<
      //            ", "<< (int)Values._a<<"\n";

      if (Values._x < min.x) min.x = Values._x;
      if (Values._y < min.y) min.y = Values._y;
      if (Values._z < min.z) min.z = Values._z;

      if (Values._x > max.x) max.x = Values._x;
      if (Values._y > max.y) max.y = Values._y;
      if (Values._z > max.z) max.z = Values._z;

      positions.push_back(vec3(Values._x, Values._y, Values._z));	// -1 is been multiplied to set the correct orientation of this model....
      normals.push_back(vec3(Values._nx, Values._ny, Values._nz));
      colors.push_back(vec3((int)Values._r, (int)Values._g, (int)Values._b) / 255.0f);
    }
    center = (min + max) / 2.0f;

    //Bounding volume measurements
    bvWidth = max.x - min.x;
    bvHeight = max.y - min.y;
    bvDepth = max.z - min.z;
    bvAspectRatio = bvWidth / bvHeight;
  }

  // Runs when there is COLOR information  but no NORMAL information
  else if (isColor && !isNormal)
  {
    struct data_1 Value;
    if(!ifColor)
    {
      std::cout<<"\nFile doesn't contain the Color Information, check the constructor call";
      exit(0);
    }

    inputPly.read((char *)&Value,sizeof(Value));
    //cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<< Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<< Values._r <<"\t"<< Values._g <<"\t"<< Values._b <<"\t"<< Values._a;
    min = max = vec3(Value._x, Value._y, Value._z);

    positions.push_back(vec3(Value._x, Value._y, Value._z));
    colors.push_back(vec3(Value._r, Value._g, Value._b) / 255.0f);

    for (long int i = 1; i < vertexCount; i++)
    {
      inputPly.read((char *)&Value,sizeof(Value));
      //cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<< Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<< (int)Values._r <<"\t"<< (int)Values._g <<"\t"<< (int)Values._b <<"\t"<< (int)Values._a;

      if (Value._x < min.x) min.x = Value._x;
      if (Value._y < min.y) min.y = Value._y;
      if (Value._z < min.z) min.z = Value._z;

      if (Value._x > max.x) max.x = Value._x;
      if (Value._y > max.y) max.y = Value._y;
      if (Value._z > max.z) max.z = Value._z;

      positions.push_back(vec3(Value._x, Value._y, Value._z));	// -1 is been multiplied to set the correct orientation of this model....
      colors.push_back(vec3((int)Value._r, (int)Value._g, (int)Value._b) / 255.0f);
    }
    center = (min + max) / 2.0f;

    //Bounding volume measurements
    bvWidth = max.x - min.x;
    bvHeight = max.y - min.y;
    bvDepth = max.z - min.z;
    bvAspectRatio = bvWidth / bvHeight;

  }

  // Runs if there is NORMAL but no color Information
  else if(!isColor && isNormal)
  {
    struct data_2 Value;
    if(!ifNormal)
    {
      std::cout<<"\nFile doesn't contain the Normal Information, check the constructor call";
      exit(0);
    }
    inputPly.read((char *)&Value,sizeof(Value));
    //cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<< Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<< Values._r <<"\t"<< Values._g <<"\t"<< Values._b <<"\t"<< Values._a;
    min = max = vec3(Value._x, Value._y, Value._z);

    positions.push_back(vec3(Value._x, Value._y, Value._z));
    normals.push_back(vec3(Value._nx, Value._ny, Value._nz));

    for (long int i = 1; i < vertexCount; i++)
    {
      inputPly.read((char *)&Value,sizeof(Value));
      //cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<< Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<< (int)Values._r <<"\t"<< (int)Values._g <<"\t"<< (int)Values._b <<"\t"<< (int)Values._a;

      if (Value._x < min.x) min.x = Value._x;
      if (Value._y < min.y) min.y = Value._y;
      if (Value._z < min.z) min.z = Value._z;

      if (Value._x > max.x) max.x = Value._x;
      if (Value._y > max.y) max.y = Value._y;
      if (Value._z > max.z) max.z = Value._z;

      positions.push_back(vec3(Value._x, Value._y, Value._z));	// -1 is been multiplied to set the correct orientation of this model....
      normals.push_back(vec3(Value._nx, Value._ny, Value._nz));
    }
    center = (min + max) / 2.0f;

    //Bounding volume measurements
    bvWidth = max.x - min.x;
    bvHeight = max.y - min.y;
    bvDepth = max.z - min.z;
    bvAspectRatio = bvWidth / bvHeight;

  }

  // Runs if there is no NORMAL and no color Information, Only the vertexd information
  else if(!isColor && !isNormal)
  {
    struct data_3 Value;

    inputPly.read((char *)&Value,sizeof(Value));
    //cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<< Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<< Values._r <<"\t"<< Values._g <<"\t"<< Values._b <<"\t"<< Values._a;
    min = max = vec3(Value._x, Value._y, Value._z);

    positions.push_back(vec3(Value._x, Value._y, Value._z));

    for (long int i = 1; i < vertexCount; i++)
    {
      inputPly.read((char *)&Value,sizeof(Value));
      //cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<< Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<< (int)Values._r <<"\t"<< (int)Values._g <<"\t"<< (int)Values._b <<"\t"<< (int)Values._a;

      if (Value._x < min.x) min.x = Value._x;
      if (Value._y < min.y) min.y = Value._y;
      if (Value._z < min.z) min.z = Value._z;

      if (Value._x > max.x) max.x = Value._x;
      if (Value._y > max.y) max.y = Value._y;
      if (Value._z > max.z) max.z = Value._z;

      positions.push_back(vec3(Value._x, Value._y, Value._z));	// -1 is been multiplied to set the correct orientation of this model....
    }
    center = (min + max) / 2.0f;

    //Bounding volume measurements
    bvWidth = max.x - min.x;
    bvHeight = max.y - min.y;
    bvDepth = max.z - min.z;
    bvAspectRatio = bvWidth / bvHeight;

  }


  //face Reading Code
  if(isMesh)
  {
    unsigned char numEdges;
    for(int i=0;i<faceCount;i++)
    {
      inputPly.read((char *)&numEdges,sizeof(numEdges));
      inputPly.read((char *)&F,sizeof(F));
      faces.push_back(ivec3(F.a,F.b,F.c));
    }
  }
  inputPly.close();

  std::cout<<"[ReadStandardPLY] Finish read."<<std::endl;
}

// the ply file expolr from assimp has float r, g, b, which is not support by mashlab.
void PLYModel::ReadAssimpColorPLY(
    const char* filename)
{
  bool isNormal = true;
  bool isColor = true;

  struct faceData F;
  isMesh=0;

  std::string line;
  std::ifstream inputPly;
  inputPly.open(filename,std::ios::binary);

  if (!inputPly.is_open()) {
    std::cerr << "Couldn't open " << filename << '\n';
    exit(1);
  }

  ifColor=0;
  ifNormal=0;

  std::getline(inputPly, line);
  while(line.compare("vertex")!=0)
  {
    std::getline(inputPly, line,' ');
    std::cout<<line<<"\n";
  }
  inputPly>>vertexCount; // 3rd line
  std::cout<<vertexCount;

  //getline(inputPly,line);
  //bool visit=0;
  while(line.compare("face")!=0)
  {
    if(line.find("red")==0 || line.find("r")==0) ifColor=1;
    if(line.find("nx")==0) ifNormal=1;
    std::getline(inputPly, line,' ');
    std::cout<<line<<std::endl;
  }

  inputPly>>faceCount;
  std::cout<<faceCount;
  if(faceCount>0)
    isMesh=1;

  while(line.compare("end_header")!=0)
  {
    std::getline(inputPly, line);
    std::cout<<line<<std::endl;
  }

  // This runs if the file has both COLOR and NORMAL information
  if(isNormal && isColor)
  {
    struct data_float_color Values;

    if(!ifColor)
    {
      std::cout<<"\nFile doesn't contain the Color Information, check the constructor call";
      exit(0);
    }
    if(!ifNormal)
    {
      std::cout<<"\nFile doesn't contain the Normal Information, check the constructor call";
      exit(0);
    }

    // read the first line of the data
    std::getline(inputPly, line);

    //make a stream for the line itself
    std::istringstream in(line);
    in >> Values._x >> Values._y >> Values._z >>
        Values._nx >> Values._ny >> Values._nz >>
        Values._r >> Values._g >> Values._b >>Values._a;

    std::cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<<
               Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<<
               (int)Values._r <<"\t"<< (int)Values._g <<"\t"<< (int)Values._b <<
               "\t"<< (int)Values._a<<std::endl;

    min = max = vec3(Values._x, Values._y, Values._z);

    positions.push_back(vec3(Values._x, Values._y, Values._z));
    normals.push_back(vec3(Values._nx, Values._ny, Values._nz));
    colors.push_back(vec3(Values._r, Values._g, Values._b) / 255.0f);

    for (long int i = 1; i < vertexCount; i++)
    {
      std::getline(inputPly, line);

      //make a stream for the line itself
      std::istringstream in(line);
      in >> Values._x >> Values._y >> Values._z >>
          Values._nx >> Values._ny >> Values._nz >>
          Values._r >> Values._g >> Values._b >>Values._a;

      //      cout<<"\n"<<Values._x <<"\t"<< Values._y <<"\t"<< Values._z <<"\t"<<
      //            Values._nx <<"\t"<< Values._ny <<"\t"<< Values._nz <<"\t"<<
      //            (int)Values._r <<"\t"<< (int)Values._g <<"\t"<< (int)Values._b <<
      //            "\t"<< (int)Values._a;

      if (Values._x < min.x) min.x = Values._x;
      if (Values._y < min.y) min.y = Values._y;
      if (Values._z < min.z) min.z = Values._z;

      if (Values._x > max.x) max.x = Values._x;
      if (Values._y > max.y) max.y = Values._y;
      if (Values._z > max.z) max.z = Values._z;

      // -1 is been multiplied to set the correct orientation of this model....
      positions.push_back(vec3(Values._x, Values._y, Values._z));
      normals.push_back(vec3(Values._nx, Values._ny, Values._nz));
      colors.push_back(vec3((int)Values._r, (int)Values._g, (int)Values._b) / 255.0f);
    }

    center = (min + max) / 2.0f;

    //Bounding volume measurements
    bvWidth = max.x - min.x;
    bvHeight = max.y - min.y;
    bvDepth = max.z - min.z;
    bvAspectRatio = bvWidth / bvHeight;
  }

  //face Reading Code
  if(isMesh)
  {
    unsigned char numEdges;
    for(int i=0;i<faceCount;i++)
    {
      // read the first line of the data
      std::getline(inputPly, line);

      //make a stream for the line itself
      std::istringstream in(line);
      in >> numEdges >> F.a>> F.b >> F.c;
      //      inputPly.read((char *)&numEdges,sizeof(numEdges));
      //      inputPly.read((char *)&F,sizeof(F));
      faces.push_back(ivec3(F.a,F.b,F.c));
    }

    std::cout<<"[ReadColorPLY] Finish Read Faces."<<std::endl;
  }

  inputPly.close();
}


////////////////////////////////////////////////////////////////////////////////
void PLYModel::PLYWrite(
    const char* filename,
    bool isNormal,
    bool isColor)
{
  std::cout<<"[PLYWrite] Start."<<std::endl;

  if(!isColor && isNormal)
  {
    FILE *outputPly;
    outputPly=fopen(filename,"wb");

    fprintf(outputPly,"ply\n");
    fprintf(outputPly,"format binary_little_endian 1.0\n");
    fprintf(outputPly,"comment This contains a Splatted Point Cloud\n");
    fprintf(outputPly,"element vertex %d\n",vertexCount);
    fprintf(outputPly,"property float x\n");
    fprintf(outputPly,"property float y\n");
    fprintf(outputPly,"property float z\n");
    fprintf(outputPly,"property float nx\n");
    fprintf(outputPly,"property float ny\n");
    fprintf(outputPly,"property float nz\n");
    fprintf(outputPly,"element face %d\n",faceCount);
    fprintf(outputPly,"property list uchar int vertex_indices\n");
    fprintf(outputPly,"end_header\n");

    //write vertices and normals
    for(long int i = 0; i < vertexCount ; i++)
    {
      fwrite(&positions[i],sizeof(glm::vec3),1,outputPly);
      fwrite(&normals[i],sizeof(glm::vec3),1,outputPly);
    }
    // write faces
    unsigned char sides=3;
    for(int i=0;i<faceCount;i++)
    {
      fwrite(&sides,sizeof(unsigned char),1,outputPly);
      fwrite(&faces[i],sizeof(glm::ivec3),1,outputPly);
    }
    fclose(outputPly);
  }

  else if(isColor && isNormal)
  {
    FILE *outputPly;
    struct colorData C;
    outputPly=fopen(filename,"wb");

    fprintf(outputPly,"ply\n");
    fprintf(outputPly,"format binary_little_endian 1.0\n");
    fprintf(outputPly,"comment This contains a Splatted Point Cloud\n");
    fprintf(outputPly,"element vertex %d\n",vertexCount);
    fprintf(outputPly,"property float x\n");
    fprintf(outputPly,"property float y\n");
    fprintf(outputPly,"property float z\n");
    fprintf(outputPly,"property float nx\n");
    fprintf(outputPly,"property float ny\n");
    fprintf(outputPly,"property float nz\n");
    fprintf(outputPly,"property uchar red\n");
    fprintf(outputPly,"property uchar green\n");
    fprintf(outputPly,"property uchar blue\n");
    fprintf(outputPly,"property uchar alpha\n");
    fprintf(outputPly,"element face %d\n",faceCount);
    fprintf(outputPly,"property list uchar int vertex_indices\n");
    fprintf(outputPly,"end_header\n");

    //write vertices and normals
    for(long int i = 0; i < vertexCount ; i++)
    {
      C.a=255;
      C.r=colors[i].r*255;
      C.g=colors[i].g*255;
      C.b=colors[i].b*255;
      fwrite(&positions[i],sizeof(glm::vec3),1,outputPly);
      fwrite(&normals[i],sizeof(glm::vec3),1,outputPly);
      fwrite(&C,sizeof(struct colorData),1,outputPly);
    }
    // write faces
    unsigned char sides=3;
    for(int i=0;i<faceCount;i++)
    {
      fwrite(&sides,sizeof(unsigned char),1,outputPly);
      fwrite(&faces[i],sizeof(glm::ivec3),1,outputPly);
    }
    fclose(outputPly);
  }

  std::cout<<"[PLYModel::PLYWrite] Write file to "<<filename<<std::endl;
}

////////////////////////////////////////////////////////////////////////////////
bool PLYModel::PLYWrite(
    std::vector<aiVector3D>& r_verts,
    std::vector<aiVector3D>& r_norms,
    std::vector<aiFace>& r_face,
    std::vector<aiColor4D>& r_colors,
    const char* filename,
    bool isNormal,
    bool isColor)
{
  std::cout<<"[PLYWrite] Start."<<std::endl;

  if(!isColor && isNormal)
  {
    FILE *outputPly;
    outputPly=fopen(filename,"wb");

    fprintf(outputPly,"ply\n");
    fprintf(outputPly,"format binary_little_endian 1.0\n");
    fprintf(outputPly,"comment This contains a Splatted Point Cloud\n");
    fprintf(outputPly,"element vertex %d\n",static_cast<int>(r_verts.size()) );
    fprintf(outputPly,"property float x\n");
    fprintf(outputPly,"property float y\n");
    fprintf(outputPly,"property float z\n");
    fprintf(outputPly,"property float nx\n");
    fprintf(outputPly,"property float ny\n");
    fprintf(outputPly,"property float nz\n");
    fprintf(outputPly,"element face %d\n",static_cast<int>(r_face.size()) );
    fprintf(outputPly,"property list uchar int vertex_indices\n");
    fprintf(outputPly,"end_header\n");

    //write vertices and normals
    for(unsigned long int i = 0; i < r_verts.size() ; i++)
    {
      fwrite(&r_verts[i],sizeof(aiVector3D),1,outputPly);
      fwrite(&r_norms[i],sizeof(aiVector3D),1,outputPly);
    }
    // write faces
    unsigned char sides=3;
    glm::ivec3 tempFace;
    for(unsigned int i=0;i<r_face.size();i++)
    {
      fwrite(&sides,sizeof(unsigned char),1,outputPly);
      tempFace = ivec3(r_face[i].mIndices[0],r_face[i].mIndices[1],r_face[i].mIndices[2]);
      fwrite(&tempFace,sizeof(glm::ivec3),1,outputPly);
    }
    fclose(outputPly);
  }

  if(isColor && isNormal)
  {
    FILE *outputPly;
    struct colorData C;

    outputPly=fopen(filename,"wb");

    fprintf(outputPly,"ply\n");
    fprintf(outputPly,"format binary_little_endian 1.0\n");
    fprintf(outputPly,"comment This contains a Splatted Point Cloud\n");
    fprintf(outputPly,"element vertex %d\n",static_cast<int>(r_verts.size()));
    fprintf(outputPly,"property float x\n");
    fprintf(outputPly,"property float y\n");
    fprintf(outputPly,"property float z\n");
    fprintf(outputPly,"property float nx\n");
    fprintf(outputPly,"property float ny\n");
    fprintf(outputPly,"property float nz\n");
    fprintf(outputPly,"property uchar red\n");
    fprintf(outputPly,"property uchar green\n");
    fprintf(outputPly,"property uchar blue\n");
    fprintf(outputPly,"property uchar alpha\n");
    fprintf(outputPly,"element face %d\n",static_cast<int>(r_face.size()));
    fprintf(outputPly,"property list uchar int vertex_indices\n");
    fprintf(outputPly,"end_header\n");

    //write vertices and normals
    for(unsigned long int i = 0; i < r_verts.size() ; i++)
    {
      C.a=255;
      C.r=static_cast<unsigned char>(r_colors[i][0]);
      C.g=static_cast<unsigned char>(r_colors[i][1]);
      C.b=static_cast<unsigned char>(r_colors[i][2]);

      fwrite(&r_verts[i],sizeof(aiVector3D),1,outputPly);
      fwrite(&r_norms[i],sizeof(aiVector3D),1,outputPly);
      fwrite(&C,sizeof(struct colorData),1,outputPly);
    }

    // write faces
    unsigned char sides=3;
    glm::ivec3 tempFace;
    for(unsigned int i=0;i<r_face.size();i++)
    {
      fwrite(&sides,sizeof(unsigned char),1,outputPly);
      tempFace = ivec3(r_face[i].mIndices[0],r_face[i].mIndices[1],r_face[i].mIndices[2]);
      fwrite(&tempFace,sizeof(glm::ivec3),1,outputPly);
    }
    fclose(outputPly);
  }

  std::cout<<"[PLYModel::PLYWrite] Write file to "<<filename<<std::endl;

  return true;
}

void PLYModel::FreeMemory()
{
  positions.~vector();
  normals.~vector();
  colors.~vector();
  if(isMesh)
  {
    faces.~vector();
  }

}
