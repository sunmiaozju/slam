### 简要思想
利用这一次扫描到的点云和下一次扫描到的点云，判断出机器人的位置和姿态。

点云匹配问题是激光SLAM算法的主要研究问题，如何进行两片或多片点云的匹配呢？？

### pcl点云库学习
####特征提取
对点云进行理解，不能仅仅局限于单一点的三维笛卡尔坐标系xyz

因为即使两个点的坐标完全相同（在两个不同时间拍摄），也不能代表这两个点具有相同的含义，因为还要考虑到周边点的分布特征。因此提出了局部特征描述的概念。这是点云描述的基本特征。

一个好的特征分类器应该具有一下三个特点：
1. 转换不变性：旋转和坐标变换不应该影响特征提取器的提取结果
2. 采样密度不变性：对于同一个局部，不同的采样密度，特征提取器提取的特征向量应该接近或一致
3. 噪音适应性：对于具有轻微噪音的点云，对特征提取器的影响应该很小。

#### 配准
配准是点云建图常用的技术手段，核心思想就是识别两个点云之间的对应点，然后寻找出一个矩阵使得对应点之间的距离最小，换句话说就是最小化对齐误差。

配准过程是一个迭代的过程，一旦对齐误差小于某一个设定的阈值，那么配准就结束了。

#### 基本数据结构
1.pcl::PointCloud::width (int)

对于非结构化点云来说（例如激光雷达产生的）用来描述点云的点数目，对于结构化点云来说（例如深度摄像机产生的），用来描述点云矩阵的列数目。

2.pcl::PointCloud::height (int)

对于非结构化点云来说都等于1，对于结构化点云来说，用来描述点云矩阵的行数目，因此可以从该变量判断出是结构化点云还是非结构化点云（也可以用 pcl::PointCloud::isOrganized 来判断）

3.pcl::PointCloud::points (std::vector<pcl::PointXYZ>)

>例如：新建一个点云和3维点序列：
>
>pcl::PointCloud<<pcl::PointXYZ>> cloud;
>
>std::vector<<pcl::PointXYZ>> data = cloud.points;

4.pcl::PointCloud::is_dense> (bool)

true：点云中的点都是有限的   false:点云中的某个点的xyz包含Inf/NaN值

#### CMakeLists.txt
分析添加pcl库到项目中CMakeLists.txt如何编写：
```shell
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
```
一句一句来分析作用：
```shell
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
```
这一步代表所需要的cmake的最低版本是2.6，
```
project(MY_GRAND_PROJECT)
```
这一步命名自己的项目名字，还会引申定义一些相关的宏定义，例如（MY_GRAND_PROJECT_SOURCE_DIR）
```
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(PCL 1.3 REQUIRED)
```
这一步是引入pcl的相关模块，因为pcl是基于模块化的，可以自定义使用哪些模块，也可以根据需要引入pcl的全部模块
```shell
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
```
当pcl库被找到之后，上面这些宏变量就被定义好了,include_directories代表编译的时候引入头文件的目录，link_directories代表连接时查找库的目录地址
```
add_executable(pcd_write_test pcd_write.cpp)
```
这一步是告诉cmake使用什么源文件，编译产生名字是什么的目标文件
```shell
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
```
这一步是把编译好的目标文件和需要的pcl库进行连接
