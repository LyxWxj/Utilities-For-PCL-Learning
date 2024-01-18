#pragma once
#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include "pcl.h"
#include "pcl_utility.h"

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

using namespace std;

namespace pcl_utils {
    namespace io {

    }
	namespace vis {
        /**
        *   @brief ���ӻ�����
        *   @param pointCloud ����
        *   @return void
        */
        template<typename PointT>
        void Visualize(pcl::PointCloud<PointT> const& pointCloud, std::string const& id) {
            pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
            viewer.setBackgroundColor(0.0, 0.0, 0.0);
            viewer.addPointCloud<PointT>(pointCloud.makeShared(), id.c_str());
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id.c_str());
            viewer.addCoordinateSystem(1.0);
            viewer.initCameraParameters();
            viewer.spin();
        }

        template<typename PointT>
        void Visualize(pcl::PointCloud<PointT> const& pointCloud, pcl::PointCloud<PointT> const& keyPoints) {
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

            // add the point cloud to the viewer
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler(pointCloud.makeShared(), 255, 0, 0);
            viewer->addPointCloud(pointCloud.makeShared(), cloud_color_handler, "point cloud");

            // add the key points to the viewer
            pcl::visualization::PointCloudColorHandlerCustom<PointT> key_points_color_handler(keyPoints.makeShared(), 0, 255, 0);
            viewer->addPointCloud(keyPoints.makeShared(), key_points_color_handler, "key points");

            // set the background color of the viewer
            viewer->setBackgroundColor(0, 0, 0);

            // set the camera position and orientation
            viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0, 0, -1);

            // display the viewer
            while (!viewer->wasStopped()) {
                viewer->spinOnce();
            }
        }
	}
    namespace trans {
        template<typename PointT> Eigen::Vector3d
            getCenter(pcl::PointCloud<PointT> const& input) {
            // �����������
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(input, centroid);
            return Eigen::Vector3d(centroid[0], centroid[1], centroid[2]);
        }
    }

    namespace ransac {
        template<typename SampleConsensusModel>
        void _RANSAC(PointCloudXYZ::Ptr cloud, vector<int>& output, double thresh) {
            auto model = SampleConsensusModel::Ptr(new SampleConsensusModel(cloud));
            auto ransac = pcl::RandomSampleConsensus<pcl::PointXYZ>(model);
            ransac.setDistanceThreshold(thresh);
            ransac.computeModel();
            ransac.getInliers(output);
        }

        enum Type_Ransac { FACE, SPHERE };

        /**
        *    RANSAC������
        *    @param <T> Ŀǰ֧��: pcl_utils::FACE, pcl_utils::SPHERE
        *    @param output �������
        *    @param thresh ��ֵ
        *
        *    @return void
        */
        template<Type_Ransac T>
        auto RANSAC(PointCloudXYZ::Ptr cloud, vector<int>& output, double thresh) -> void {
            if (T == FACE) { // face����
                _RANSAC<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(cloud, output, thresh);
            }
            else if (T == SPHERE) { //sphere ����
                _RANSAC<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(cloud, output, thresh);
            }
        }
    }

    namespace filter {
        /**
        *   @brief ����ͶӰ
        *
        *   @param <ModelType> ģ������
        *   @param input_cloud, output_cloud �����������
        *   @param coefficients ģ��ϵ��
        *
        */
        template<pcl::SacModel ModelType>
        PointCloudXYZ::Ptr
            Projection(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, pcl::ModelCoefficients::Ptr coefficients) {
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType(ModelType);
            proj.setInputCloud(input_cloud);
            proj.setModelCoefficients(coefficients);
            proj.filter(*output_cloud);
            return output_cloud;
        }

#define GT pcl::ComparisonOps::GT
#define GE pcl::ComparisonOps::GE
#define LT pcl::ComparisonOps::LT
#define LE pcl::ComparisonOps::LE
#define EQ pcl::ComparisonOps::EQ

        //// ����չ���ĵݹ���ֹ
        //void
        //    _addCondition(pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition, string const& field, pcl::ComparisonOps::CompareOp comp, double thresh) {
        //    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(field, comp, thresh)));
        //}

        ///***
        //*   @brief Ϊcondition��ӱȽϲ���
        //*
        //*   @param field �ֶ�
        //*   @param comp ��ѡ����: pcl::ComparisonOps::GT, GE, LT, LE, EQ
        //*   @param thresh ��ֵ
        //*   @param ...args ���(field, comp, thresh)
        //*
        //*   @return void
        //*/

        //template<typename...Args> void
        //    _addCondition(pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition, string const& field, pcl::ComparisonOps::CompareOp comp, double thresh, Args...args) {
        //    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(field, comp, thresh)));
        //    _addCondition(condition, args...);
        //}

        ///**
        //*
        //*   @brief �����˲�
        //*
        //*   @param input �������
        //*   @param output �������
        //*   @param conditions ���� ���(field, comp, thresh) ����comp��ѡ����: pcl::ComparisonOps::GT, GE, LT, LE, EQ
        //*
        //*   @return �˲���ĵ���
        //*/
        //template<typename...Args>
        //PointCloudXYZ::Ptr
        //    ConditionFilter(PointCloudXYZ::Ptr input, PointCloudXYZ::Ptr output, Args...conditions) {
        //    assert(sizeof...(conditions) % 3 == 0);
        //    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
        //    _addCondition(range_cond, conditions...);
        //    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        //    condrem.setCondition(range_cond);
        //    condrem.setInputCloud(input);
        //    condrem.setKeepOrganized(true);
        //    condrem.filter(*output);
        //    return output;
        //}

        ///**
        //*
        //*   @brief �����˲�
        //*
        //*   @param input �������
        //*   @param conditions ����
        //*
        //*   @return �˲���ĵ���
        //*/
        //template<typename...Args>
        //PointCloudXYZ::Ptr
        //    ConditionFilter(PointCloudXYZ::Ptr input, Args... conditions) {
        //    PointCloudXYZ::Ptr output(new PointCloudXYZ());
        //    return ConditionFilter(input, output, conditions...);
        //}

        template<typename PointCloud_Ptr>
        PointCloud_Ptr slides(PointCloud_Ptr cloud, vector<int> const& inliers) {
            PointCloud_Ptr res(new PointCloud_Ptr());
            pcl::copyPointCloud(*cloud, inliers, *res);
            return res;
        }
    }

    namespace feature {
        namespace normal {
            /***
            *
            *   @brief ���ڻ��ֵı��淨��������
            *
            *   @param <Method> ��ѡ:pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::
                 COVARIANCE_MATRIX ģʽ�Ӿ���ĳ����ľֲ������Э������󴴽�9�����֣������������ķ���
                 AVERAGE_3D_GRADIENT   ģʽ����6������ͼ������ˮƽ����ʹ�ֱ�����ƽ�������ά�ݶȲ�ʹ�������ݶȼ�����������㷨��
                 AVERAGE_DEPTH__CHANGE  ģʽֻ������һ����һ�Ļ���ͼ���Ӷ�ƽ����ȱ仯���㷨��
            *
            *   @param cloud �������
            *   @param DepthChangeFactor ��ȱ仯ϵ��
            *   @param NormalSmoothingSize �����Ż�ʱ���ǵ������С
            *
            *   @return ����������
            */

#define COVARIANCE_MATRIX pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::COVARIANCE_MATRIX
#define AVERAGE_3D_GRADIENT pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::AVERAGE_3D_GRADIENT
#define AVERAGE_DEPTH__CHANGE pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::AVERAGE_DEPTH__CHANGE

            template<pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod Method>
            pcl::PointCloud<pcl::Normal>::Ptr
                Normal_Estimation(PointCloudXYZ::Ptr cloud, double DepthChangeFactor, double NormalSmoothingSize) {
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
                ne.setInputCloud(cloud);
                ne.setNormalEstimationMethod(Method);
                ne.setMaxDepthChangeFactor(DepthChangeFactor);
                ne.setNormalSmoothingSize(NormalSmoothingSize);
                ne.compute(*normals);
                return normals;
            }
        }

        template<typename PointCloudFeatureTypePtr, typename...Args>
        PointCloudFeatureTypePtr
            ComputeFeature(PointCloudXYZ::Ptr cloud, Args...args) {return nullptr; }

        template<typename...Args>
        pcl::PointCloud<pcl::SHOT352>::Ptr
            ComputeFeature(PointCloudXYZ::Ptr cloud, Args...args) {
            return feature::SHOTEstimation(cloud, args...);
        }

        template<typename...Args>
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr
            ComputeFeature_FPFHSignature33(PointCloudXYZ::Ptr cloud, Args...args) {
            return feature::normal::FPFHEstimation(cloud, args...);
        }

        template<typename...Args>
        pcl::PointCloud<pcl::VFHSignature308>::Ptr
            ComputeFeature(PointCloudXYZ::Ptr cloud, Args...args) {
            return feature::VFHEstimation(cloud, args...);
        }
        template<typename...Args>
        pcl::PointCloud<pcl::Normal>::Ptr
            ComputeFeature(PointCloudXYZ::Ptr cloud, Args...args) {
            return feature::normal::Normal_Estimation(cloud, args...);
        }

    }

    namespace segment {
        template<pcl::SacModel sacmodel = pcl::SACMODEL_PLANE, int method = pcl::SAC_RANSAC>
        PointCloudXYZ::Ptr
            planar_segmentation(PointCloudXYZ::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double thresh) {
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(sacmodel);
            seg.setMethodType(method);
            seg.setDistanceThreshold(thresh);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            return filter::slides(cloud, inliers->indices);
        }

        template<pcl::SacModel sacmodel = pcl::SACMODEL_PLANE, int method = pcl::SAC_RANSAC>
        vector<PointCloudXYZ::Ptr>
            euclidean_cluster_extraction(PointCloudXYZ::Ptr cloud, vector<pcl::PointIndices>& cluster_indices,
                double voxeSize = 0.01, double tolerance = 0.02, int min_size = 100, int max_size = 25000) {
            auto cloud_filtered = filter::VoxelFilt(cloud, voxeSize);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
            seg.setOptimizeCoefficients(true);
            seg.setModelType(sacmodel);    //�ָ�ģ��
            seg.setMethodType(method);       //����������Ʒ���
            seg.setMaxIterations(100);                //���ĵ����Ĵ���
            seg.setDistanceThreshold(0.02);           //���÷�ֵ
            int i = 0, nr_points = (int)cloud_filtered->points.size();
            while (cloud_filtered->points.size() > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.size() == 0)
                {
                    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                    break;
                }

                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);

                // Get the points associated with the planar surface
                extract.filter(*cloud_plane);
                std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

                //  // ��ȥƽ����ڵ㣬��ȡʣ�����
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *cloud_filtered = *cloud_f;
            }

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //ŷʽ�������
            ec.setClusterTolerance(tolerance);                     // ���ý��������������뾶Ϊ2cm
            ec.setMinClusterSize(min_size);                 //����һ��������Ҫ�����ٵĵ���ĿΪ100
            ec.setMaxClusterSize(max_size);               //����һ��������Ҫ��������ĿΪ25000
            ec.setSearchMethod(tree);                    //���õ��Ƶ���������
            ec.setInputCloud(cloud_filtered);
            ec.extract(cluster_indices);           //�ӵ�������ȡ���࣬������������������cluster_indices��
            vector<PointCloudXYZ::Ptr> res;
            //�������ʵ�������cluster_indices,ֱ���ָ���о���
            for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                res.push_back(cloud_cluster);
            }
            return res;
        }
    }

    namespace registration {
        template<typename FeatureType>
        PointCloudXYZ::Ptr
            SACRegistration(PointCloudXYZ::Ptr source_cloud, PointCloudXYZ::Ptr target_cloud,
                pcl::PointCloud<FeatureType>const& source_descriptors, pcl::PointCloud<FeatureType>const& target_descriptors,
                double thresh = 0.01, int max_iter = 1000) {
            pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureType> sample_consensus;
            sample_consensus.setInputSource(source_cloud);
            sample_consensus.setSourceFeatures(source_descriptors.makeShared());
            sample_consensus.setInputTarget(target_cloud);
            sample_consensus.setTargetFeatures(target_descriptors.makeShared());
            sample_consensus.setMaximumIterations(max_iter);
            sample_consensus.setNumberOfSamples(3);
            sample_consensus.setCorrespondenceRandomness(5);
            sample_consensus.setSimilarityThreshold(thresh);
            sample_consensus.setMaxCorrespondenceDistance(0.05f);
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            sample_consensus.align(*aligned_cloud);
            return aligned_cloud;
        }
    }
}

#endif // !PCL_UTILS_H