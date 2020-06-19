/*
The mapping algorithm is an advanced implementation of the following open source project:
  [blam](https://github.com/erik-nelson/blam). 
Modifier: livox               dev@livoxtech.com


Copyright (c) 2015, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/



#include <blam_slam/BlamSlam.h>
#include "common.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <geometry_utils/Transform3.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress (double percentage)
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf ("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush (stdout);
}

namespace gu = geometry_utils;

typedef pcl::POINT_TYPE PointT;
typedef pcl::PointCloud<PointT> PointCloud;

char framesDir[100] = "../data/Base_LiDAR_Frames";

using namespace pcl::visualization;
BlamSlam LivoxSLAM;

long long frameNumber = 0;

std::string itos(int i)
{
    std::stringstream s;
    s << i;
    return s.str();
}



int main(void)
{
  
    //================== Step.1 Reading L-LiDAR frames =====================//
    struct dirent **namelist;
    int framenumbers = scandir(framesDir, &namelist, 0, alphasort) - 2;
    int frame_count = 100000;
    int cframe_count = 0;
    cout << "Start building local map..." << endl;
    cout << "Loaded " << framenumbers << " frames from Base-LiDAR" << endl;

    char filename[] = "../data/T_Matrix.txt";
    ofstream fout(filename);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    LivoxSLAM.Initialize();

   //================== Step.2 building submap =====================//

    while (!viewer.wasStopped())  
    {
        while (frame_count < framenumbers + 100000)
        
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr frames(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(string(framesDir) + "/" + itos(frame_count) + ".pcd", *frames) == -1)
            {
                PCL_ERROR("Couldn't read H_LiDAR_Map \n");
                return (-1);
            }
            
            std::vector<int> mapping;
            
            
            LivoxSLAM.ProcessPointCloudMessage(frames);

            const gu::Transform3 estimate = LivoxSLAM.localization_.GetIntegratedEstimate();
            const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
            const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            
            tf.block(0, 0, 3, 3) = R;
            tf.block(0, 3, 3, 1) = T;
            fout << tf.matrix() << endl;

            viewer.showCloud(LivoxSLAM.mapper_.map_data_);
            
            frame_count++;
            cframe_count++;
           
            printProgress ((double)cframe_count/(double)framenumbers);
        }

        std::cout << "\nSaving submap..." << std::endl;
        pcl::PCDWriter writer;
        writer.write("../data/H-LiDAR-Map-data/H_LiDAR_Map.pcd", *LivoxSLAM.mapper_.map_data_);
        std::cout << "Mapping done！" << std::endl;
        fout.close();
        break;
       
    }

    return 0;
}
