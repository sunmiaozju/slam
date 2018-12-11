### pcl点云库学习

#### 基本数据结构
1. pcl::PointCloud::width (int)

    对于非结构化点云来说（例如激光雷达产生的）用来描述点云的点数目，对于结构化点云来说（例如深度摄像机产生的），用来描述点云矩阵的列数目。

2. pcl::PointCloud::height (int)

    对于非结构化点云来说都等于1，对于结构化点云来说，用来描述点云矩阵的行数目，因此可以从该变量判断出是结构化点云还是非结构化点云（也可以用 pcl::PointCloud::isOrganized 来判断）

3. pcl::PointCloud::points (std::vector<pcl::PointT>)

    例如：新建一个点云和3维点序列：
    >
    >pcl::PointCloud<<pcl::PointXYZ>> cloud;
    >
    >std::vector<<pcl::PointXYZ>> data = cloud.points;

4. pcl::PointCloud::is_dense> (bool)

    用来说明点云中是否包含无效值

    true：点云中的点xyz数值都是有限的

    false:点云中的某个点的xyz包含Inf/NaN值
5. pcl::PointT 这个是一个集合，包含了很多点类型

    前面所说的，pcl::PointCloud是包含众多3维点的容器，点是PointT类型的，PointT是pcl::PointCloud类的模板参数，定义了点云的存储类型。PCL定义了很多类型的点，下面是一些最常用的：
    + pcl::PointXYZ
      这是最简单的点的类型，存储着点的x,y,z信息
    + pcl::PointXYZI
      这个类型的点和前面那个很相似，但是它包含了一个维度描述点的intensity,代表单通道图像中的灰度强度,类似于亮度。
    + pcl::InterestPoint
      与前面类似，多一个维度去存储点的返回强度，侧重于品质.

    + pcl::PointWithRange
      多一个维度存储了点的范围(深度).
    + pcl::PointXYZRGB
      存储了3D信息同时还有RGB信息
    + pcl::PointXYZRGBA
      存储了3D信息和RGB与Alpha(透明度)
    - pcl::Normal
      这是一个最常用的点的类型，它代表了给定点的表面法线估计值和表面曲率估计值

    - pcl::PointNormal
      这个类型和前面那个一样，只不过多了坐标(x,y,z)，相关变体还有PointXYZRGBNormal和PointXYZINormal,就像名字所说的一样，前者包含颜色，后者包含密集度。
#### 添加自定义PointT
如上面所说，pcl里面预先提供了众多PointT数据类型。但是在某一情况之下，我们需要的PointT类型比较特殊，比如需要xyz和表面曲率估计值（不要表面法向属性），那么如果使用含有多余属性的预定义PointT类型，会使得内存浪费和数据传输变慢，因此有的时候我们需要自定义PointT数据类型。

详细参考链接：[Adding your own custom PointT type](http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php#adding-custom-ptype)

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

#### 特征提取
对点云进行理解，不能仅仅局限于单一点的三维笛卡尔坐标系xyz

因为即使两个点的坐标完全相同（在两个不同时间拍摄），也不能代表这两个点具有相同的含义，因为还要考虑到周边点的分布特征。因此提出了局部特征描述的概念。这是点云描述的基本特征。

一个好的特征分类器应该具有一下三个特点：
1. 转换不变性：旋转和坐标变换不应该影响特征提取器的提取结果
2. 采样密度不变性：对于同一个局部，不同的采样密度，特征提取器提取的特征向量应该接近或一致
3. 噪音适应性：对于具有轻微噪音的点云，对特征提取器的影响应该很小。

#### 配准
将多帧3D点云数据一致的对准到同一个3D模型中的过程被称为配准，其目标是在全局坐标系下寻找出获得的（不同视角）单帧点云数据的相对位置和朝向，以便于让不同帧点云的交叉区域完全重叠。

主要工作就是找到不同点云中的点对应关系，然后估计多个可以将不同点云转换为全局统一坐标系的刚性变换。对两个点云来说，就是识别两个点云之间的对应点，然后寻找出一个矩阵使得对应点之间的距离最小，换句话说就是最小化对齐误差。

这个过程是一个迭代的过程，一旦对齐误差小于某一个设定的阈值，那么配准就结束了。

配准是点云建图常用的技术手段

 <img src="./block_diagram_single_iteration.jpg" width = "700" height = "600" alt="pic" align=center />

下面是配准的单步迭代的流程：
- 获取两个点云数据，识别关键点代表原始点云
- 在每一个关键点，计算特征特征描述器
- 基于一系列特征描述器的特征和位置，从两个点云数据计算出对应关系
- 拒绝由于点云噪声引起的坏对应关系
- 从余下的优对应关系中，估计刚性变换矩阵
- 迭代循环，直到对应误差达到收敛标准...

详细参考链接：[The PCL Registration API](http://pointclouds.org/documentation/tutorials/registration_api.php#registration-api)
