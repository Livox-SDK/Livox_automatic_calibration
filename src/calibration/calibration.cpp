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

#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_utils/Transform3.h>
#include <point_cloud_mapper/PointCloudMapper.h>

using namespace std;
namespace gu = geometry_utils;

#define PI (3.1415926535897932346f)

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress(double percentage)
{
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

char framesDir[100] = "../data/Target-LiDAR-Frames";

std::string itos(int i)
{
    std::stringstream s;
    s << i;
    return s.str();
}

int main()
{
    std::cout << "Start calibration..." << std::endl;

    //================== Step.1 Reading H-LiDAR map data =====================//

    std::cout << "Reading H-LiDAR map data..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr H_LiDAR_Map(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/H-LiDAR-Map-data/H_LiDAR_Map.pcd", *H_LiDAR_Map) == -1)
    {
        PCL_ERROR("Couldn't read H_LiDAR_Map \n");
        return (-1);
    }
    std::cout << "Loaded " << H_LiDAR_Map->size() << " data points from H_LiDAR_Map.pcd" << std::endl;

    //put it into map
    PointCloudMapper maps;
    maps.Initialize();
    pcl::PointCloud<pcl::PointXYZ>::Ptr unused(new pcl::PointCloud<pcl::PointXYZ>);
    maps.InsertPoints(H_LiDAR_Map, unused.get());

    //================== Step.2 Reading H-LiDAR's Trajectory and init guess=====================//

    ifstream T_Mat_File("../data/T_Matrix.txt");
    Eigen::Matrix4f T_Matrix = Eigen::Matrix4f::Identity();

    ifstream initFile("../data/Init_Matrix.txt");

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f init_guess_0 = Eigen::Matrix4f::Identity();

    for (int mat_i = 0; mat_i != 4; mat_i++)
    {
        for (int mat_j = 0; mat_j != 4; mat_j++)
        {
            initFile >> init_guess(mat_i, mat_j);
        }
    }

    init_guess_0 = init_guess;
    //================== Step.3 Reading L-LiDAR frames =====================//

    struct dirent **namelist;
    int framenumbers = scandir(framesDir, &namelist, 0, alphasort) - 2;
    int frame_count = 100000;
    int cframe_count = 0;
    cout << "Loaded " << framenumbers << " frames from Target-LiDAR" << endl;

    //=================================
    //prepare ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_output_cloud(new pcl::PointCloud<pcl::PointXYZ>); //not use,but necessary
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(0.0000000001); //0.0000000001
    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(35);
    icp.setRANSACIterations(0);
    icp.setMaximumOptimizerIterations(50); // default 20

    //=================================
    //prepare display
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> map_color(H_LiDAR_Map, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_color(H_LiDAR_Map, 0, 255, 0);

    viewer_final->addPointCloud<pcl::PointXYZ>(H_LiDAR_Map, map_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer_final->addPointCloud<pcl::PointXYZ>(H_LiDAR_Map, match_color, "match cloud"); //display the match cloud

    //=================================
    // prepare save matrix

    char filename[] = "../data/calib_data.txt";
    ofstream fout(filename);
    fout.setf(ios::fixed, ios::floatfield);
    fout.precision(7);

    //=================================
    //              START
    //=================================
    while (!viewer_final->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr frames(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(string(framesDir) + "/" + itos(frame_count) + ".pcd", *frames) == -1)
        {
            PCL_ERROR("Couldn't read H_LiDAR_Map \n");
            return (-1);
        }
        //std::cout << "Loaded " << frames->size() << " data points from frames" << std::endl;

        //Load H-LiDAR's Trajectory
        for (int mat_i = 0; mat_i != 4; mat_i++)
        {
            for (int mat_j = 0; mat_j != 4; mat_j++)
            {
                T_Mat_File >> T_Matrix(mat_i, mat_j);
            }
        }

        //================== Step.4 Start calibration =====================//

        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*frames, *trans_output_cloud, init_guess); //Tiny_T * init_guess   ->update this matrix
        pcl::transformPointCloud(*trans_output_cloud, *final_output_cloud, T_Matrix);

        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbors_L(new pcl::PointCloud<pcl::PointXYZ>); //201 neighbors points from nap202
        maps.ApproxNearestNeighbors(*final_output_cloud, neighbors_L.get());

        //INVERSE T_mat==============================
        
        gu::Transform3 inverse_mat;
        inverse_mat.translation = gu::Vec3(T_Matrix(0, 3), T_Matrix(1, 3), T_Matrix(2, 3));
        inverse_mat.rotation = gu::Rot3(T_Matrix(0, 0), T_Matrix(0, 1), T_Matrix(0, 2),
                                        T_Matrix(1, 0), T_Matrix(1, 1), T_Matrix(1, 2),
                                        T_Matrix(2, 0), T_Matrix(2, 1), T_Matrix(2, 2));

        const gu::Transform3 estimate = gu::PoseInverse(inverse_mat); //integrated_estimate from config parameters
        const Eigen::Matrix<double, 3, 3> T_Matrix_Inverse_R = estimate.rotation.Eigen();
        const Eigen::Matrix<double, 3, 1> T_Matrix_Inverse_T = estimate.translation.Eigen();

        Eigen::Matrix4d T_Matrix_Inverse;
        T_Matrix_Inverse.block(0, 0, 3, 3) = T_Matrix_Inverse_R;
        T_Matrix_Inverse.block(0, 3, 3, 1) = T_Matrix_Inverse_T;

        //====== Core step ======//
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbors_trans(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*neighbors_L, *neighbors_trans, T_Matrix_Inverse);

        //Do ICP and get the tiny trans T
        icp.setInputSource(trans_output_cloud); //201
        icp.setInputTarget(neighbors_trans);    //202 (201's neighbor's point cloud)
        icp.align(*ICP_output_cloud);
        const Eigen::Matrix4f Tiny_T = icp.getFinalTransformation();

        //std::cout << "Score: " << icp.getFitnessScore() << std::endl;

        if (icp.getFitnessScore() > 1)
        {
            //std::cout<<"not match, skip this"<<std::endl;
            init_guess = init_guess_0;
            //continue;
        }
        else
        {
            Eigen::Matrix4f Final_Calib_T = Eigen::Matrix4f::Identity();

            Final_Calib_T = Tiny_T * init_guess;
            //std::cout << Final_Calib_T.matrix() << std::endl;
            init_guess = Final_Calib_T;

            //===== Out put Euler angle =====//
            gu::Vector3 EulerAngle;
            gu::Rot3 rot_mat(Final_Calib_T(0, 0), Final_Calib_T(0, 1), Final_Calib_T(0, 2),
                             Final_Calib_T(1, 0), Final_Calib_T(1, 1), Final_Calib_T(1, 2),
                             Final_Calib_T(2, 0), Final_Calib_T(2, 1), Final_Calib_T(2, 2));
            EulerAngle = rot_mat.GetEulerZYX();
            const Eigen::Matrix<double, 3, 1> EulerAngle_T = EulerAngle.Eigen();
            //std::cout<<"EulerAngle:  "<<EulerAngle_T(0,0)<<"  "<<EulerAngle_T(1,0)<<"  "<<EulerAngle_T(2,0)<<"  "<<std::endl;

            if (icp.getFitnessScore() < 0.1)
                fout << frame_count - 100000 << " " << icp.getFitnessScore() << " " << Final_Calib_T(0, 3) << " " << Final_Calib_T(1, 3) << " " << Final_Calib_T(2, 3) << " " << EulerAngle_T(0, 0) << " " << EulerAngle_T(1, 0) << " " << EulerAngle_T(2, 0) << endl; //x,y,z,roll,pitch,yaw
        }

        frame_count++;
        cframe_count++;

        printProgress((double)cframe_count / (double)framenumbers);
        viewer_final->updatePointCloud<pcl::PointXYZ>(final_output_cloud, match_color, "match cloud");
        viewer_final->spinOnce(10);

        if (cframe_count == framenumbers)
        {
            std::cout << "\n Matching complete." << std::endl;
            break;
        }
    }

    return 0;
}
