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

// the code is modified from https://github.com/vgvishesh/PLY_Loader by Lu, Ma

#ifndef PLY_LOADER_H
#define	PLY_LOADER_H

#include <glm/glm.hpp>
#include <vector>
#include <sstream>
#include <string>


struct data
{
  float _x, _y, _z;
  float _nx, _ny, _nz;
  unsigned char _r, _g, _b, _a;
  data(){
    _x= _y=_z=_nx=_ny=_nz=_r=_g=_b=_a=0;
  }
};

struct data_float_color
{
  float _x, _y, _z;
  float _nx, _ny, _nz;
  float _r, _g, _b, _a;
  data_float_color(){
    _x= _y=_z=_nx=_ny=_nz=_r=_g=_b=_a=0;
  }
};

// To read files not having the normal data.
struct data_1
{
  float _x, _y, _z;
  unsigned char _r, _g, _b, _a;
  data_1(){
    _x= _y=_z=_r=_g=_b=_a=0;
  }
};

// To read files not having the color data
struct data_2
{
  float _x, _y, _z;
  float _nx, _ny, _nz;
  data_2(){
    _x= _y=_z=_nx=_ny=_nz=0;
  }
};

// To read files not having the color data and normal data
struct data_3
{
  float _x, _y, _z;
  data_3(){
    _x= _y=_z=0;
  }
};

struct faceData
{
  int a,b,c;
  faceData()
  {
    a=b=c=0;
  }
};

struct colorData
{
  unsigned char r,g,b,a;
  colorData(){
    r=g=b=a=0;
  }
};


class PLYModel {
public:
  PLYModel();

  //To indicate if normal, color informations are present in the file respectively
  void ReadStarnardPLY(const char *filename, bool isnormal, bool iscolor);
  void ReadAssimpColorPLY(const char *filename);

  // To indicate if normal, color informations are to be written in the file respectively
  void PLYWrite(const char *filename, bool =1,bool =1);
  void FreeMemory();

private:
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec3> colors;
  std::vector<glm::ivec3> faces;

  int vertexCount; //number of vertices
  float bvWidth, bvHeight, bvDepth; //bounding volume dimensions
  float bvAspectRatio; //bounding volume aspect ratio
  int faceCount; //number of faces; if reading meshes
  bool isMesh; // To tell if this is a mesh or not
  bool ifColor,ifNormal;

  glm::vec3 min, max, center;
};

#endif	/* PLYLOADER_H */
