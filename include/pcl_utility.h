/**
* 
* @file pcl_ultility.h
* @brief pcl_ultility.h
* @details pcl教程中的一些例程整合, 抽象化, 便于调用和理解, 以及一些常用的函数
* 模板化的例程在pcl_ultility_template.hpp中实现
* 在namespace pcl_utils命名空间中包含十大模块:
*  io: 输入输出
*  vis: 可视化
*  feature: 特征提取
*  filter: 滤波算法整合
*  keypoints: 关键点提取
*  segment: 分割
*  surface: 表面重建
*  registration: 点云配准
*  recognition: 点云识别
*  ml: 机器学习(暂未实现)
* @version 1.0.0
* @date 2023/12/10 上午完成最后的修改
* 
**/

#pragma once

#ifndef __PCL_ULTILITY_H__
#define __PCL_ULTILITY_H__

#include "pcl.h"

namespace pcl_utils {
    using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
    using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
    using namespace std;

    namespace io {
        void loadTXTFile(string const& filename, pcl::PointCloud<pcl::PointXYZ>& cloud);
        void loadTXTFile(string const& filename, pcl::PointCloud<pcl::PointNormal>& cloud);
        /**
        *   @brief 读取pcd文件
        *   @param pcd_file pcd文件路径
        *   @return PointCloudXYZ::Ptr
        */
        PointCloudXYZ::Ptr open(const string& pcd_file);

        /**
        *   @brief 将点云转换为txt文件
        *   @param pointCloud 点云
        *   @return void
        */
        void PointCloud2TXT(PointCloudXYZ::Ptr pointCloud, string const& out_dir);

        /**
        *   @brief 将点云转换为深度图
        *   @param pointCloud 点云
        *   @param sensorPose 相机位姿
        *   @param coordinate_frame 深度图坐标系
        *   @return void
        */
        void PointCloud2RangeImage(PointCloudXYZ::Ptr pointCloud,
                                   Eigen::Translation3f sensorPose,
                                   pcl::RangeImage::CoordinateFrame coordinate_frame,
                                   std::string const& save_dir);

        void save(const string& filename, PointCloudXYZ::Ptr cloud);

        void save(vector<string> const& filename, vector<PointCloudXYZ::Ptr> clouds, string const& root_dir = "");

    }

    namespace vis {
        

        void Visualize(pcl::PolygonMesh mesh);

        //template<typename T, template<class> typename PointCloud>
        //void Visualize(PointCloud<T> const& fpfhs) {
        //    pcl::visualization::PCLPlotter viewer;
        //    viewer.addFeatureHistogram(fpfhs, 300);
        //    viewer.spin();
        //}

        void
        Visualize(ON_NurbsCurve& curve, 
                  ON_NurbsSurface& surface, 
                  pcl::visualization::PCLVisualizer& viewer);
    }

    namespace search {
        class splitTree_wrapper {
        public:
            virtual tuple<vector<int>, vector<float>>
                kNNSearch(pcl::PointXYZ const& point, const unsigned int K, float radius) = 0;
        };
        /**
        *   @brief pcl::KdTreeFLANN<pcl::PointXYZ>的包装类
        */
        class kdTree_wrapper : public splitTree_wrapper {
        public:
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdTree_wrapper(PointCloudXYZ::Ptr cloud) {
                kdtree.setInputCloud(cloud);
            }
            kdTree_wrapper(pcl::KdTreeFLANN<pcl::PointXYZ> const& kdtree) : kdtree(kdtree) {}

            tuple<vector<int>, vector<float>>
                kNNSearch(pcl::PointXYZ const& point, const unsigned int K, float radius = -1.f) override;
        };
        class OctTree_wrapper : public splitTree_wrapper {
        public:
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;

            OctTree_wrapper(PointCloudXYZ::Ptr cloud, float resolution = 32);

            /**
             *   @brief K近邻搜索
             *   @param point 搜索点
             *   @param K K近邻
             *   @param radius 搜索半径
             *   @return tuple<vector<int>, vector<float>> 返回搜索到的点的索引和距离
             */
            tuple<vector<int>, vector<float>>
                kNNSearch(pcl::PointXYZ const& point, const unsigned int K, float radius = -1.f) override;

            std::vector<int> voxelSearch(pcl::PointXYZ const& point);

        };
    }

    namespace filter {
        /**
         *   @brief 体素下采样滤波
         *
         *   @param cloud 输入点云
         *   @param filter 体素滤波器
         *   @param voxSize 体素大小
         *
         *   @return 采样后的点云
         */
        pcl::PCLPointCloud2::Ptr
        Filtering(
              pcl::PCLPointCloud2::Ptr cloud, 
              pcl::VoxelGrid<pcl::PCLPointCloud2>& filter,
              Eigen::Vector3f const& voxSize);

        /**
         *   @brief 体素下采样滤波
         *
         *   @param cloud 输入点云
         *   @param filter 体素滤波器
         *   @param voxSize 体素大小
         *
         *   @return 采样后的点云
         */
        PointCloudXYZ::Ptr
        Filtering(
            PointCloudXYZ::Ptr cloud, 
            pcl::VoxelGrid<pcl::PCLPointCloud2>& filter,
            Eigen::Vector3f const& voxSize);

        /**
         *   @brief 直通滤波
         *
         *   @param cloud 输入点云
         *   @param filter 直通滤波器
         *   @param field 滤时所需要点云类型的字段
         *   @param dlimit, ulimit: 上下限
         *
         *   @return 采样后的点云
         */
        PointCloudXYZ::Ptr
        Filtering(
            PointCloudXYZ::Ptr const cloud, 
            pcl::PassThrough<pcl::PointXYZ>& filter,
            string const& field, float dlimit, float ulimit);

        /**
         *   @brief 直通滤波
         *
         *   @param cloud 输入点云
         *   @param filter 直通滤波器
         *   @param field 滤时所需要点云类型的字段
         *   @param limit: 上下限
         *
         *   @return 采样后的点云
         */
        PointCloudXYZ::Ptr
        Filtering(
            PointCloudXYZ::Ptr const cloud,
            pcl::PassThrough<pcl::PointXYZ>& filter,
            string const& field,
            Eigen::Vector2f const& limit
        );

        /**
         *   @brief Statistical离群点滤波
         *
         *   @param cloud 输入点云
         *   @param Statistical离群点滤波器
         *   @param K 均值计算参考点个数
         *   @param threshold 阈值
         *
         *   @return 采样后的点云
         */
        PointCloudXYZ::Ptr
        Filtering(
            PointCloudXYZ::Ptr cloud,
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ>& filter,
            unsigned int const K, float threshold
        );


        /**
        *    @brief 体素下采样滤波
        *
        *    @param cloud 输入点云
        *    @param voxSize 体素大小
        *
        *    @return 输出点云
        */
        pcl::PCLPointCloud2::Ptr
        VoxelFilt(pcl::PCLPointCloud2::Ptr const cloud, Eigen::Vector3f const& voxSize);

        /**
        *    @brief 体素下采样滤波
        *
        *    @param cloud 输入点云
        *    @param x, y, z 体素大小
        *    @return 输出点云
        */
        pcl::PCLPointCloud2::Ptr
        VoxelFilt(pcl::PCLPointCloud2::Ptr const cloud, float x, float y, float z);

        /**
        *    @brief 体素下采样滤波
        *
        *    @param cloud 输入点云
        *    @param x 体素大小
        *    @return 输出点云
        */
        pcl::PCLPointCloud2::Ptr
        VoxelFilt(pcl::PCLPointCloud2::Ptr const cloud, float x);

        /**
        *    @brief 体素下采样滤波
        *
        *    @param cloud 输入点云
        *    @param voxSize 体素大小
        *
        *    @return 输出点云
        */
        PointCloudXYZ::Ptr
        VoxelFilt(PointCloudXYZ::Ptr const cloud, Eigen::Vector3f const& voxSize);

        /**
        *    @brief 体素下采样滤波
        *
        *    @param cloud 输入点云
        *    @param x, y, z 体素大小
        *
        *    @return 输出点云
        */
        PointCloudXYZ::Ptr
        VoxelFilt(PointCloudXYZ::Ptr const cloud, float x, float y, float z);

        /**
        *    @brief 体素下采样滤波
        *
        *    @param cloud 输入点云
        *    @param x 体素大小
        *
        *    @return 输出点云
        */
        PointCloudXYZ::Ptr
        VoxelFilt(PointCloudXYZ::Ptr const cloud, float x);

        /**
        *   @brief 直通滤波
        *
        *   @param cloud 输入点云
        *   @param field 滤时所需要点云类型的字段
        *   @param dlimit, ulimit: 上下限
        *
        *   @return 采样后的点云
        *
        */
        PointCloudXYZ::Ptr
        Passthrough(PointCloudXYZ::Ptr const cloud, string const& field, float dlimit, float ulimit);

        /**
        *   @brief 直通滤波
        *
        *   @param cloud 输入点云
        *   @param field 滤时所需要点云类型的字段
        *   @param limit: 上下限
        *
        *   @return 采样后的点云
        *
        */
        PointCloudXYZ::Ptr
        Passthrough(PointCloudXYZ::Ptr const cloud, string const& field, Eigen::Vector2f const& limit);

        /**
         *   @brief Statistical离群点滤波
         *
         *   @param cloud 输入点云
         *   @param Statistical离群点滤波器
         *   @param K 均值计算参考点个数
         *   @param threshold 阈值
         *
         *   @return 采样后的点云
         */
        PointCloudXYZ::Ptr
        StasticRemove(PointCloudXYZ::Ptr cloud, unsigned int const K, float threshold);
    }

    namespace trans {

        PointCloudXYZ::Ptr
        TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Isometry3d const& T);

        PointCloudXYZ::Ptr
        TransForm(
            PointCloudXYZ::Ptr input_cloud, 
            PointCloudXYZ::Ptr output_cloud, 
            Eigen::Vector3d const& s, 
            Eigen::AngleAxisd rotation_vector);

        PointCloudXYZ::Ptr
            TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::AngleAxisd rotation_vector, Eigen::Vector3d const& s);

        PointCloudXYZ::Ptr
            TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s, double alpha, Eigen::Vector3d const& Axis);

        PointCloudXYZ::Ptr
            TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s, double alpha, double x, double y, double z);

        PointCloudXYZ::Ptr
            TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s, Eigen::Vector3d const& EulerAngle);

        PointCloudXYZ::Ptr
            TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s, double Ax, double Ay, double Az);

        PointCloudXYZ::Ptr
            Translation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s);

        PointCloudXYZ::Ptr
            Translation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, double x, double y, double z);
        PointCloudXYZ::Ptr
            Translation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, double s);

        PointCloudXYZ::Ptr
            Rotation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& EulerAngle);

        Eigen::Vector3d Quad2EulerA(Eigen::Quaterniond const& quaternion);

        Eigen::Quaterniond EulerA2Quad(Eigen::Vector3d EulerAngle);

        PointCloudXYZ::Ptr
            TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Quaterniond const& quaternion);
    }

    namespace filter {
        // 返回点云inliers的切片
        PointCloudXYZ::Ptr
            slides(PointCloudXYZ::Ptr cloud, vector<int> const& inliers);

        /**
        *
        *   @brief 投影到平面 ax + by + cz = d
        *
        *   @param input_cloud, output_cloud 输入输出点云
        *   @param a, b, c, d 平面参数
        *
        *   @return 投影后的点云
        */
        PointCloudXYZ::Ptr
            projPlane(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, double a, double b, double c, double d);
         
        PointCloudXYZ::Ptr
            projPlane(PointCloudXYZ::Ptr input_cloud, double a, double b, double c, double d);


        /**
        *
        *   @brief 双边滤波
        *   @param input, ouput 输入点云
        *   @param std 标准差
        *   @param HalfSize 滤波窗口大小
        *
        *   @return 滤波后的点云
        **/
        PointCloudXYZI::Ptr
            bilateralFilter(
                pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                pcl::PointCloud<pcl::PointXYZI>::Ptr output, 
                double std, double HalfSize);

        /**
        *
        *   @brief 投影到平面 ax + by + cz = d
        *
        *   @param input_cloud输出点云
        *   @param a, b, c, d 平面参数
        *
        *   @return 投影后的点云
        */
        PointCloudXYZI::Ptr
            bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input, double std, double HalfSize);
        /**
        *
        *   @brief 半径滤波
        *
        *   @param input, output 输入点云
        *   @param radius 半径
        *   @param K 最小邻居数
        *
        *   @return 滤波后的点云
        */
        PointCloudXYZ::Ptr
            RadiusOutlierRemoval(PointCloudXYZ::Ptr input, PointCloudXYZ::Ptr output, double radius, unsigned int K);


        /**
        *
        *   @brief 半径滤波
        *
        *   @param input, output 输入点云
        *   @param radius 半径
        *   @param K 最小邻居数
        *
        *   @return 滤波后的点云
        */
        PointCloudXYZ::Ptr
            RadiusOutlierRemoval(PointCloudXYZ::Ptr input, double radius, unsigned int K);
    }
    
    namespace feature {

        /**
        * 
        * @brief 从input中分离出点云和法向量
        * 
        * @param input 输入点云
        * 
        * @return <PointCloudXYZ, PointCloudNormal> 点云和法向量
        */
        void
            split(pcl::PointCloud<pcl::PointNormal>::Ptr input, PointCloudXYZ::Ptr outputPoint, PointCloudNormal::Ptr outputNormal);

        namespace normal {
            /**
            *
            *   @brief 普通点云表面法向量估计
            *
            *   @param cloud 输入点云
            *   @param radius 检测半径
            *
            *   @return 法向量集合
            */
            pcl::PointCloud<pcl::Normal>::Ptr
                Normal_Estimation(PointCloudXYZ::Ptr cloud, double radius = 0.03);

            /***
            *
            *   @brief 基于多项式重构的平滑和法线估计
            *   
            *   @param cloud 输入点云
            *   @param radius 检测半径
            *   @param order 多项式拟合阶数
            * 
            *   @return 法向量集合
            */
            pcl::PointCloud<pcl::PointNormal>::Ptr
                MLSNormalEstimation(PointCloudXYZ::Ptr cloud, double radius, int order = 2);

            /**
            *
            *   @brief 法向量直方图计算
            *
            *   @param cloud 输入点云
            *   @param normals 法向量集合
            *   @param radius 检测半径
            *
            *   @return 法向量直方图
            */
            pcl::PointCloud<pcl::PFHSignature125>::Ptr
                PFHEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius);

            /**
            *
            *   @brief 法向量直方图计算
            *
            *   @param cloud 输入点云
            *   @param radius1, radius2 检测半径
                        radius1 < radius2
            *   @return 法向量直方图
            *
            */
            pcl::PointCloud<pcl::PFHSignature125>::Ptr
                PFHEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2);

            /**
            *
            *   @brief 快速法向量直方图计算
            *
            *   @param cloud 输入点云
            *   @param normals 法向量集合
            *   @param radius 检测半径
            *
            *   @return 法向量直方图
            */
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr
                FPFHEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius);
            /**
            *
            *   @brief 快速法向量直方图计算
            *
            *   @param cloud 输入点云
            *   @param radius1, radius2 检测半径
                        radius1 < radius2
            *   @return 法向量直方图
            *
            */
            pcl::PointCloud<pcl::FPFHSignature33>::Ptr
                FPFHEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2);
        }

        /**
        *   @brief 点特征直方图计算
        *   
        *   @param cloud 输入点云
        *   @param normals 法向量集合
        *   @param radius 检测半径
        *   
        *   @return 点特征直方图
        */
        pcl::PointCloud<pcl::VFHSignature308>::Ptr
            VFHEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius);

        pcl::PointCloud<pcl::VFHSignature308>::Ptr
            VFHEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2);

        pcl::PointCloud<int>::Ptr
            NarfKeyPoint(pcl::RangeImage const& range_image, float support_size);

        pcl::PointCloud<pcl::SHOT352>::Ptr
            SHOTEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius);

        pcl::PointCloud<pcl::SHOT352>::Ptr
            SHOTEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2);
}

    namespace keypoints {
        PointCloudXYZ::Ptr
            SiftKeyPoints(PointCloudXYZ::Ptr srcCloud, float min_scale = 5.f, int n_octaves = 3, int n_scales_per_octave = 15, float min_contrast = 0.01f);
    }

    namespace segment {

        PointCloudXYZ::Ptr
            plannar_segmentation(PointCloudXYZ::Ptr cloud, double thresh);
    }

    namespace surface {
        /***
        * 
        * @brief 平面凹多边形提取 记得先进行滤波和平面分割, 投影
        * 
        * @param cloud 输入点云
        * @param alpha 平面凹多边形提取参数
        * 
        * @return 平面凹多边形点云
        */
        PointCloudXYZ::Ptr
            ConCaveHull_2d(PointCloudXYZ::Ptr cloud, double alpha);

        /**
        * 
        * @brief 平面凸多边形提取 记得先进行滤波和平面分割, 投影
        * 
        * @param cloud 输入点云
        * 
        * @return 平面凸多边形点云
        */
        PointCloudXYZ::Ptr
            ConvexHull_2d(PointCloudXYZ::Ptr cloud);

        /**
        * 
        * @brief 无序点云的快速三角化
        * 
        * @param cloud_with_normals 输入带法向量的点云
        * @param radius 三角形最大边长
        * 
        * @return 存储最终三角化的网络模型
        */
        pcl::PolygonMesh::Ptr
            greedy_projection(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double radius);

        /**
        *
        * @brief 无序点云的快速三角化
        *
        * @param cloud 点云
        * @param normals 法向量
        * @param radius 三角形最大边长
        *
        * @return 存储最终三角化的网络模型
        */
        pcl::PolygonMesh::Ptr
            greedy_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius);

        void
            PointCloud2Vector3d(PointCloudXYZ::Ptr cloud, pcl::on_nurbs::vector_vec3d& data);


        /**
        * 
        * @brief 将修剪的B样条曲线拟合到无序点云
        * 
        * @
        * 
        */
        void
            bspline_fitting(PointCloudXYZ::Ptr cloud);
        
        //点云数据转为vecotr
        void
            PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data);

        //可视化曲线
        void
            VisualizeCurve(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps, pcl::visualization::PCLVisualizer viewer);

        void
            printUsage(const char* progName);


        /**
        * 
        *  @brief平面点云B样条曲线拟合
        * 
        */
        void fitting_curve_2d(int argc, char* argv[]);

        pcl::PolygonMesh
            recon_poisson(PointCloudXYZ::Ptr srcCloud);
    }

    namespace registration {
        /**
        *   @brief ICP配准
        *   @param source 源点云
        *   @param target 目标点云
        *   @param max_iter 最大迭代次数
        *   @param thresh 收敛阈值
        *   @return ICP对象
        */
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr
            ICP(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, Eigen::Matrix4f& trans, int max_iter);

		/**
        *   @brief 结合法向量的ICP配准
        *   @param source 源点云
        *   @param target 目标点云
        *   @param normals 法向量
        *   @param max_iter 最大迭代次数
        *   @param thresh 收敛阈值
        *   @return ICP对象
        */
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Ptr
            ICP(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, Eigen::Matrix4f& trans, double thresh = 1e-6, int max_iter = 100);

        class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
        {
            using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
        public:
            MyPointRepresentation()
            {
                nr_dimensions_ = 4;    //定义点的维度
            }

            // 重载copyToFloatArray方法将点转化为四维数组 
            virtual void copyToFloatArray(const pcl::PointNormal& p, float* out) const
            {
                // < x, y, z, curvature >
                out[0] = p.x;
                out[1] = p.y;
                out[2] = p.z;
                out[3] = p.curvature;
            }
        };


        void pairAlign(const PointCloudXYZ::Ptr cloud_src, const PointCloudXYZ::Ptr cloud_tgt, PointCloudXYZ::Ptr output, Eigen::Matrix4f& final_transform,
            double voxsize = 0.05, bool downsample = false, int k = 30, double eps = 1e-6);

        PointCloudXYZ::Ptr
            pairwise_incremental_registration(vector<PointCloudXYZ::Ptr> data, Eigen::Matrix4f& GlobalTransform,
                double voxsize = 0.05, bool downsample = false, int k = 30, double eps = 1e-6);

        PointCloudXYZ::Ptr
            NDT(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, Eigen::Matrix4f& finalTrans,
                double eps = 0.01, double stepSize = 0.1, double resolution = 1.0, int max_iter = 100);
        
        pcl::PointCloud<pcl::PointNormal>::Ptr
            Alignment(pcl::PointCloud<pcl::PointNormal>::Ptr object, pcl::PointCloud<pcl::PointNormal>::Ptr scene);
        /**
        * 基于FPFH特征的粗配准
        * @param source_cloud 源点云
        * @param target_cloud 目标点云
        * 
        * @return 粗配准后的点云
        * 
        */
        PointCloudXYZ::Ptr
            FPFHRegistration(PointCloudXYZ::Ptr source_cloud, PointCloudXYZ::Ptr target_cloud);

        PointCloudXYZ::Ptr
            SiftRegistration(PointCloudXYZ::Ptr source_cloud, PointCloudXYZ::Ptr target_cloud);

        pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ>
            FourPCS(PointCloudXYZ::Ptr source_cloud, PointCloudXYZ::Ptr target_cloud,
                float Overlap = 0.7,
                float delta = 0.01,
                size_t maxComputationTime = 1000,
                size_t NumerOfSamples = 200);
    }

    

    namespace recognition {
        
    }

    namespace ml {
        
    }

}

#endif // !PCL_ULTILITY