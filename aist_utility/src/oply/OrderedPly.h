// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef ORDERED_PLY_H
#define ORDERED_PLY_H
#include <string>
#include <vector>
#include <array>

typedef enum PhoXiControlVersion {
  PC_VER_ANY,	// Unknown version
  PC_VER_1_1,	// PhoXi Control v1.1.x
  PC_VER_1_2	// PhoXi Control v1.2.x
} PCver;

//
// OrderedPly構造体.
//
// PhoXiが出力するPLYファイルのデータ構造. 欠落した3D点の扱いが特殊なPLY.
// ・3D点の属性は, 3D座標, 法線ベクトル, RGBカラー, 輝度, 距離, 確信度.
// ・3D点の数＝画像の幅×高さ.
// ・全ての3D点がラスタスキャン順に並び, 全点にカラーの情報が付与される.
// ・欠落した3D点は3D座標が(0,0,0), 法線ベクトルが(0,0,1)となる.
//
struct OrderedPly {
  PCver version;		// PhoXi Controlのバージョン
  int size;			// 読み込んだ点の数
  int last;			// vertexの最後の属性. 次のvertexへの目印.

  // element vertex
  std::vector<std::array<float, 3> >  point;	   // 3次元座標(x,y,z)の列
  std::vector<std::array<float, 3> >  normal;	   // 法線ベクトル(nx,ny,nz)の列
  std::vector<std::array<u_char, 3> > color;	   // カラー(red,green,blue)の列
  std::vector<float>		      texture;	   // 輝度(Texture32)の列
  std::vector<float>		      depth;	   // 距離(Depth32)の列.
  std::vector<float>		      confidence;  // 確信度(Confidence32)の列.

  // element camera
  std::array<float, 3> view;
  std::array<float, 3> x_axis;
  std::array<float, 3> y_axis;
  std::array<float, 3> z_axis;

  // element phoxi_frame_params
  int   frame_width;
  int   frame_height;
  int   frame_index;
  float frame_start_time;
  float frame_duration;
  float frame_computation_duration;	// v1.2.x beta以降.
  float frame_transfer_duration;	// v1.2.x beta以降.

  // element camera_matrix
  float cm[9];				// v1.2.x beta以降.

  // element distortion_matrix
  float dm[14];				// v1.2.x beta以降.

  // element camera_resolution
  float width;				// v1.2.x beta以降.
  float height;				// v1.2.x beta以降.

  // element frame_binning
  float horizontal;			// v1.2.x beta以降.
  float vertical;			// v1.2.x beta以降.


  OrderedPly() : version(PC_VER_ANY), size(0), last(0),
    frame_width(0), frame_height(0),
    frame_index(0), frame_start_time(0), frame_duration(0),
    frame_computation_duration(0), frame_transfer_duration(0),
    width(0), height(0), horizontal(0), vertical(0)
  {
    for (int i = 0; i < 9;  i++) cm[i] = 0;
    for (int i = 0; i < 14; i++) dm[i] = 0;
  };
};

//
// OrderedPlyの読み込み
//
class OPlyReader {
 private:
  std::string filename;
  OrderedPly& data;
  PCver       guess;

 public:
  OPlyReader(std::string inputFile, OrderedPly& inputData);

  // ファイルを開く（PhoXi Ordered Ply形式）
  void read(void);
};

//
// OrderedPlyの書き込み
//
class OPlyWriter {
 private:
  std::string filename;
  OrderedPly& data;

 public:
  OPlyWriter(std::string outputFile, OrderedPly& inputData);

  // ファイルの保存（PhoXi Ordered Ply形式）
  void write(void);
};

#endif // ORDERED_PLY_H
