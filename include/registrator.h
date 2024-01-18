//#pragma once
//#ifndef __REGISTRATOR_H_
//#define __REGISTRATOR_H_
//
//#include "pcl_utility.h"
//#include "pcl_utility_template.hpp"
//
//// Do the Normal 4PCS And Trying To Improve It On My Own With the Basic Tools of PCL
//
//class Registrator {
//public:
//	virtual void setSrc(pcl_utils::PointCloudXYZ) = 0;
//	virtual void settarg(pcl_utils::PointCloudXYZ) = 0;
//	virtual Eigen::Matrix4f Align() = 0;
//};
//
//template<typename FeatureType>
//class FPCS : public Registrator{
//private:
// // Member Variables Private
//	pcl_utils::PointCloudXYZ::Ptr src;
//	pcl_utils::PointCloudXYZ::Ptr targ;
//private:
// // Member Methods Private
//	void setup_base(pcl::Indices& , float(&ratio)[2]);
//	float segmentToSegmentDist(pcl::Indices const&, float(&ratio)[2]);
//public:
// // Member Methods Public:
//	template<typename Sampler>
//	//vector<size_t> extractCoplanarIdx(pcl_utils::PointCloudXYZ::Ptr cloud, Sampler& sampler) {
//	//	// 在cloud中随机采样4个共面点, 返回这4个点的索引
//	//	// cloud: 点云
//	//	// sampler: 采样器
//	//	// return: 4个共面点的索引
//	//	vector<size_t> idxs = sampler.sample(3);
//	//	double eps = 0.01;
//	//	double best_error = std::numeric_limits<double>::max();
//	//	size_t best_idx = 0;
//	//	int cnt = 50;
//	//	while (true && cnt--) {
//	//		auto coplanar = pcl_utils::filter::slides(cloud, idxs);
//	//		size_t forthPoint = sampler.sample(1)[0];
//	//		auto vecA = coplanar->points[1].getVector3fMap() - coplanar->points[0].getVector3fMap();
//	//		auto vecB = coplanar->points[2].getVector3fMap() - coplanar->points[0].getVector3fMap();
//	//		// 取叉积
//	//		auto Cross = vecA.cross(vecB);
//	//		auto vecC = cloud->points[forthPoint].getVector3fMap() - coplanar->points[0].getVector3fMap();
//	//		// 计算点积
//	//		auto dot = Cross.dot(vecC);
//	//		if (dot < eps) {
//	//			idx.push_back(forthPoint);
//	//			return idx;
//	//		}
//	//		else if (dot < best_error) {
//	//			best_error = dot;
//	//			best_idx = forthPoint;
//	//		}
//	//	}
//	//	idx.push_back(best_idx);
//	//	return idx;
//	//}
//
// 	void extractCorrespondentIdx(std::vector<size_t>& src_idx, std::vector<size_t>& targ_idx);
//	static std::pair< pcl_utils::PointCloudXYZ::Ptr, pcl_utils::PointCloudXYZ::Ptr> extractCorrespondentPoint(std::vector<size_t>& src_base_idx, std::vector<size_t>& targ_base_idx);
//    static Eigen::Matrix3f PCA(pcl_utils::PointCloudXYZ::Ptr);
//	static void sortByPCA_Axis(pcl_utils::PointCloudXYZ::Ptr, Eigen::Matrix3f const&);
//
//	static std::vector<size_t> Match(pcl_utils::PointCloudXYZ::Ptr, pcl_utils::PointCloudXYZ::Ptr, double eps);
//	Eigen::Matrix4f computeTransformMatrix(pcl_utils::PointCloudXYZ::Ptr src_QuadPoints, pcl_utils::PointCloudXYZ::Ptr targ_QuadPoints);
//	static double computeError(pcl_utils::PointCloudXYZ::Ptr , pcl_utils::PointCloudXYZ::Ptr, Eigen::Matrix4f const& transform);
//public:
// // Member Variables Public
//	void setSrc(pcl_utils::PointCloudXYZ::Ptr) override final;
//	void settarg(pcl_utils::PointCloudXYZ::Ptr) override final;
//	Eigen::Matrix4f Align() override final;
//};
//
//
//
//#endif // !__REGISTRATOR_H_
