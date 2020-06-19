# Livox 外参自动标定工具v0.1　快速参考手册

## 简介
本工具提供了用于Livox多雷达之间的外参自动标定方法。

![image](../pic/1.png)   
**图１.** 红色点云：基准雷达建图结果, 绿色点云: 自动标定中的目标雷达点云

## 依赖库
Cmake, PCL1.7, Eigen
建议直接安装ROS，包含了以上的库

```
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport

```
## 编译

```

mkdir build
cd build
cmake ..
make

```

生成三个文件，分别是**mapping**, **calibration**, **fitline**　  

* **mapping:**可视化建图工具　　

* **calibration:**可视化自动标定工具　　

* **fitline:**　拟合标定参数，计算最终参数矩阵　　



## 运行
###　**1.准备双雷达标定pcd数据**　　

* 基准雷达数据放在	data/Base_LiDAR_Frames/.pcd (以 100000.pcd 作为第一帧文件名，后续帧往上累加)  

* 待标定雷达数据放在	data/Target_LiDAR_Frames/.pcd (以 100000.pcd 作为第一帧文件名，后续帧往上累加)  

* 粗配准外参矩阵放在	data/Init_Matrix.txt  

**注意**：基准雷达数据和待标定雷达数据需要尽可能在时间上同步，文件名和时间戳为同步对应。

 **示例数据下载**  
 [**Target-LiDAR-Frames**](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/Target-LiDAR-Frames.tar.gz)  
 [**Base_LiDAR_Frames**](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/Base_LiDAR_Frames.tar.gz)  

###　**2.运行**
```
cd livox_calibration
cp run.sh build/
cd build
sh run.sh

```
脚本首先启动可视化建图程序**mapping**，用基准雷达建立子地图，建立完成后启动标定程序**calibration**，完成标定参数估计工作，最后启动
参数拟合器**fitline**，完成最终参数矩阵计算。

![image](../pic/output.png ) 
**图２．** 样例数据的自动标定结果输出

## **注意：**  
1.必须保证双雷达数据同步  
2.基准雷达建图必须尽可能准确  
3.采集车、平台运动必须尽可能缓慢，数据建议进行运动畸变修正以保证最终的精度  
4.粗配准外参数矩阵不必要很精准，但需要大致能够对齐，尽可能保证最终的精度。  
5.对于Mid-40, Horizon型号，转换的PCD文件参照100ms为一帧。  

**Develper: [Livox](https://www.livoxtech.com/)**

