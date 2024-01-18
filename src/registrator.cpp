//#include "registrator.h"
//#include "random_engine.h"
//
//template<typename FeatureType>
//void FPCS<FeatureType>::setSrc(pcl_utils::PointCloudXYZ::Ptr src) {
//    this->src = src;
//}
//
//template<typename FeatureType>
//void FPCS<FeatureType>::settarg(pcl_utils::PointCloudXYZ::Ptr targ) {
//    this->targ = targ;
//}
//
//template<typename FeatureType>
//void FPCS<FeatureType>::setup_base(pcl::Indices& base_indices, float(&ratio)[2]) {
//    float best_t = std::numeric_limits<float>::max();
//    const pcl::Indices copy(base_indices.begin(), base_indices.end());
//    pcl::Indices temp(base_indices.begin(), base_indices.end());
//    // loop over all combinations of base points
//    for (auto i = copy.begin(), i_e = copy.end(); i != i_e; ++i)
//        for (auto j = copy.begin(), j_e = copy.end(); j != j_e; ++j) {
//            if (i == j)
//                continue;
//            for (auto k = copy.begin(), k_e = copy.end(); k != k_e; ++k) {
//                if (k == j || k == i)
//                    continue;
//                auto l = copy.begin();
//                while (l == i || l == j || l == k)
//                    ++l;
//                temp[0] = *i;
//                temp[1] = *j;
//                temp[2] = *k;
//                temp[3] = *l;
//                // calculate diagonal intersection ratios and check for suitable segment to
//                // segment distances
//                float ratio_temp[2];
//                float t = segmentToSegmentDist(temp, ratio_temp);
//                if (t < best_t) {
//                    best_t = t;
//                    ratio[0] = ratio_temp[0];
//                    ratio[1] = ratio_temp[1];
//                    base_indices = temp;
//                }
//            }
//        }
//}
//
//template<typename FeatureType>
//float FPCS<FeatureType>::segmentToSegmentDist(pcl::Indices const& base_indices, float(&ratio)[2])
//{
//    //// get point vectors
//    //Eigen::Vector3f u = (*corres)[base_indices[1]].getVector3fMap() -
//    //    (*targ)[base_indices[0]].getVector3fMap();
//    //Eigen::Vector3f v = (*targ)[base_indices[3]].getVector3fMap() -
//    //    (*targ)[base_indices[2]].getVector3fMap();
//    //Eigen::Vector3f w = (*targ)[base_indices[0]].getVector3fMap() -
//    //    (*targ)[base_indices[2]].getVector3fMap();
//
//    //// calculate segment distances
//    //float a = u.dot(u);
//    //float b = u.dot(v);
//    //float c = v.dot(v);
//    //float d = u.dot(w);
//    //float e = v.dot(w);
//    //float D = a * c - b * b;
//    //float sN = 0.f, sD = D;
//    //float tN = 0.f, tD = D;
//
//    //// check segments
//    //if (D < small_error_) {
//    //    sN = 0.f;
//    //    sD = 1.f;
//    //    tN = e;
//    //    tD = c;
//    //}
//    //else {
//    //    sN = (b * e - c * d);
//    //    tN = (a * e - b * d);
//
//    //    if (sN < 0.f) {
//    //        sN = 0.f;
//    //        tN = e;
//    //        tD = c;
//    //    }
//    //    else if (sN > sD) {
//    //        sN = sD;
//    //        tN = e + b;
//    //        tD = c;
//    //    }
//    //}
//
//    //if (tN < 0.f) {
//    //    tN = 0.f;
//
//    //    if (-d < 0.f)
//    //        sN = 0.f;
//
//    //    else if (-d > a)
//    //        sN = sD;
//
//    //    else {
//    //        sN = -d;
//    //        sD = a;
//    //    }
//    //}
//
//    //else if (tN > tD) {
//    //    tN = tD;
//
//    //    if ((-d + b) < 0.f)
//    //        sN = 0.f;
//
//    //    else if ((-d + b) > a)
//    //        sN = sD;
//
//    //    else {
//    //        sN = (-d + b);
//    //        sD = a;
//    //    }
//    //}
//
//    //// set intersection ratios
//    //ratio[0] = (std::abs(sN) < small_error_) ? 0.f : sN / sD;
//    //ratio[1] = (std::abs(tN) < small_error_) ? 0.f : tN / tD;
//
//    //Eigen::Vector3f x = w + (ratio[0] * u) - (ratio[1] * v);
//    //return (x.norm());
//}
//
////template<typename FeatureType>
////void FPCS<FeatureType>::extractCorrespondentIdx(std::vector<size_t>& src_idx, std::vector<size_t>& targ_idx) {
////    // 从源点云和目标点云中提取对应点
////    // src_idx: 源点云中对应点的索引
////    // targ_idx: 目标点云中对应点的索引
////
////    auto Feature_src = pcl_utils::feature::ComputeFeature<pcl::PointCloud<FeatureType>::Ptr>(this->src, 0.05, 0.10);
////    auto Feature_targ = pcl_utils::feature::ComputeFeature<pcl::PointCloud<FeatureType>::Ptr>(this->targ, 0.05, 0.10);
////
////    pcl::registration::CorrespondenceEstimation<FeatureType, FeatureType> Estimator;
////    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
////    
////    Estimator.setInputSource(Feature_src);
////    Estimator.setInputTarget(Feature_targ);
////    Estimator.setSearchMethodSource(pcl::search::KdTree<FeatureType>::Ptr(new pcl::search::KdTree<FeatureType>));
////    Estimator.setSearchMethodTarget(pcl::search::KdTree<FeatureType>::Ptr(new pcl::search::KdTree<FeatureType>));
////    Estimator.determineReciprocalCorrespondences(*correspondences);
////    src_idx = std::move((*correspondences)[0].index_query);
////    targ_idx = std::move((*correspondences)[0].index_match);
////}
//
//template<typename FeatureType>
//std::pair<pcl_utils::PointCloudXYZ::Ptr, pcl_utils::PointCloudXYZ::Ptr> 
//    FPCS<FeatureType>::extractCorrespondentPoint(
//        std::vector<size_t>& src_base_idx, std::vector<size_t>& targ_base_idx) {
//    auto Src_base = pcl_utils::filter::slides(src, src_base_idx);
//    auto Targ_base = pcl_utils::filter::slides(targ, targ_base_idx);
//    return {Src_base, Targ_base};
//}
//
//template<typename FeatureType>
//Eigen::Matrix3f FPCS<FeatureType>::PCA(pcl_utils::PointCloudXYZ::Ptr pointCloud) {
//    // 主成分分析
//    // pointCloud: 点云
//    // return: 主成分矩阵
//    Eigen::Matrix3f covarianceMatrix;
//    Eigen::Vector3f mean;
//    pcl::computeMeanAndCovarianceMatrix(*pointCloud, covarianceMatrix, mean);
//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covarianceMatrix, Eigen::ComputeEigenvectors);
//    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//    return eigenVectorsPCA;
//}
//
//template<typename FeatureType>
//void FPCS<FeatureType>::sortByPCA_Axis(pcl_utils::PointCloudXYZ::Ptr pointCloud, Eigen::Matrix3f const& princepleMatrix) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    projectedCloud->points.reserve(pointCloud->points.size());
//
//    // 投影后, 主要成分为x轴, 次要成分为y轴, 次次要成分为z轴
//    std::vector<size_t> indices(pointCloud->points.size());
//    std::iota(indices.begin(), indices.end(), 0);
//
//    for (const auto& point : pointCloud->points) {
//        Eigen::Vector3f p(point.x, point.y, point.z);
//        Eigen::Vector3f projected = princepleMatrix.transpose() * p;
//        projectedCloud->points.push_back(pcl::PointXYZ(projected(0), projected(1), projected(2)));
//    }
//
//    // 根据projectedCloud中的点对indices进行排序
//    std::sort(indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
//        const auto& p1 = projectedCloud->points[i1];
//        const auto& p2 = projectedCloud->points[i2];
//        return std::tie(p1.x, p1.y, p1.z) < std::tie(p2.x, p2.y, p2.z);
//        });
//     
//    // 根据indices对pointCloud进行排序
//    pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    sortedCloud->points.reserve(pointCloud->points.size());
//    for (size_t i : indices) {
//        sortedCloud->points.push_back(pointCloud->points[i]);
//    }
//
//    // 将排序后的点云赋值给原点云
//    *pointCloud = *sortedCloud;
//}
//
//template<typename FeatureType>
//std::vector<size_t> FPCS<FeatureType>::Match(pcl_utils::PointCloudXYZ::Ptr src_QuadPoints, pcl_utils::PointCloudXYZ::Ptr targ_BaseSet, double eps) {
//    // 从目标点云中找到与源点云中的四个点对应的点
//    // src_QuadPoints: 源点云中的四个点
//    // targ_BaseSet: 目标点云中的基准点集
//    // return: 与源点云中的四个点对应的点的索引
//    SobolSampler sampler(targ_BaseSet->points.size(), 1);
//    double min_error = std::numeric_limits<double>::max();
//    std::vector<size_t> match_idxs;
//    float ratio[2] = 0;
//    setup_base(src_QuadPoints, ratio);
//    while (min_error > eps) {
//		auto sample_idxs = this->extractCoplanarIdx(targ_BaseSet, sampler);
//		auto targ_QuadPoints = pcl_utils::filter::slides(targ_BaseSet, sample_idxs);
//		// 计算仿射不变比
//        float ratio_temp[2];
//
//        /*if (error < min_error) {
//			min_error = error;
//			match_idxs = std::move(sample_idxs);
//		}*/
//	}
//}
//
//template<typename FeatureType>
//Eigen::Matrix4f FPCS<FeatureType>::computeTransformMatrix(pcl_utils::PointCloudXYZ::Ptr src_QuadPoints, pcl_utils::PointCloudXYZ::Ptr targ_QuadPoints) {
//    auto transform = Eigen::Matrix4f::Identity();
//    return transform;
//}
//
//template<typename FeatureType>
//Eigen::Matrix4f FPCS<FeatureType>::Align() {
//    if(src.get() == nullptr || targ.get() == nullptr) {
//		std::cerr << "src or targ is nullptr" << std::endl;
//		return transform;
//	}
//    std::vector<size_t> src_idx, targ_idx;
//    this->extractCorrespondentIdx(src_idx, targ_idx);
//    auto [src_baseSet, targ_baseSet] = this->extractCorrespondentPoint(src_idx, targ_idx);
//    auto src_princepleMatrix = this->PCA(src_baseSet);
//    auto targ_princepleMatrix = this->PCA(targ_baseSet);
//    this->sortByPCA_Axis(src_baseSet, src_princepleMatrix);
//    this->sortByPCA_Axis(targ_baseSet, targ_princepleMatrix);
//    SobolSampler sampler(src_baseSet.size(), 1);
//    auto best_transform = Eigen::Matrix4f::Identity();
//    auto transform = Eigen::Matrix4f::Identity();
//    double min_error = std::numeric_limits<double>::max();
//    int cnt = 10;
//    while (min_error > 0.01 && cnt--) {
//        auto sample_idxs = this->extractCoplanarIdx(src_baseSet, sampler);;
//        auto src_QuadPoints = pcl_utils::filter::slides(src_baseSet, sample_idxs);
//        auto match_idxs = this->Match(src_QuadPoints, targ_baseSet, 0.00001);
//        transform = this->computeTransformMatrix(src_QuadPoints, pcl_utils::filter::slides(targ_baseSet, match_idxs));
//        double error = this->computeError(src_baseSet, src_targSet, transform);
//        if (error < min_error) {
//			min_error = error;
//			best_transform = transform;
//		}
//    }
//
//    return best_transform;
//}
