#pragma once
#ifndef _PERFORMANCE_TEST_H_
#define _PERFORMANCE_TEST_H_

#include "4pcs.hpp"
#include "pcl_utility.h"
#include "pcl_utility_template.hpp"
#include "goicp/jly_goicp.h"
#include "goicp/ConfigMap.hpp"
#include "gr/algorithms/match4pcsBase.h"
#include "gr/algorithms/FunctorSuper4pcs.h"
#include "gr/utils/geometry.h"
#include "gr/algorithms/PointPairFilter.h"

struct TrVisitorType {
	template <typename Derived>
	inline void operator() (
		typename Derived::Scalar fraction,
		typename Derived::Scalar best_LCP,
		const Eigen::MatrixBase<Derived>& /*transformation*/) {
		if (fraction >= 0)
		{
			printf("done: %d%c best: %f                  \r",
				static_cast<int>(fraction * 100), '%', best_LCP);
			fflush(stdout);
		}
	}
	constexpr bool needsGlobalTransformation() const { return false; }
};

const int nbSet = 2;

using Scalar = float;

std::array<Scalar, nbSet> n_points = {
	200,
	200,
};

std::array<Scalar, nbSet> deltas = {
	0.004,
	0.003,
};

using MatcherType = gr::Match4pcsBase<gr::FunctorSuper4PCS, gr::Point3D<float>, TrVisitorType, gr::AdaptivePointFilter, gr::AdaptivePointFilter::Options>;
using OptionType = typename MatcherType::OptionsType;

constexpr gr::Utils::LogLevel loglvl = gr::Utils::Verbose;
gr::Utils::Logger logger(loglvl);
using SamplerType = gr::UniformDistSampler<gr::Point3D<Scalar> >;
SamplerType sampler;

inline int loadPointCloud(POINT3D** p, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int N = cloud->size();

	*p = (POINT3D*)malloc(sizeof(POINT3D) * N);
	for (int i = 0; i < N; i++)
	{
		(*p)[i].x = cloud->points[i].x;
		(*p)[i].y = cloud->points[i].y;
		(*p)[i].z = cloud->points[i].z;
	}
	return 0;
}

inline double FitnessScore(POINT3D* model, POINT3D* data, int Nm, int Nd) {
	double score = 0.0;
	for (int i = 0; i < Nd; i++)
	{
		double min_dist = 1e10;
		for (int j = 0; j < Nm; j++)
		{
			double dist = (model[j].x - data[i].x) * (model[j].x - data[i].x) +
				(model[j].y - data[i].y) * (model[j].y - data[i].y) +
				(model[j].z - data[i].z) * (model[j].z - data[i].z);
			if (dist < min_dist)
				min_dist = dist;
		}
		score += min_dist;
	}
	return score;

}

inline double FitnessScore(pcl_utils::PointCloudXYZ::Ptr Final, pcl_utils::PointCloudXYZ::Ptr targ) {
	double score = 0.0;
	int Nm = Final->size();
	int Nd = targ->size();
	auto model = Final->points;
	auto data = targ->points;
	for (int i = 0; i < Nd; i++)
	{
		double min_dist = 1e10;
		for (int j = 0; j < Nm; j++)
		{
			double dist = (model[j].x - data[i].x) * (model[j].x - data[i].x) +
				(model[j].y - data[i].y) * (model[j].y - data[i].y) +
				(model[j].z - data[i].z) * (model[j].z - data[i].z);
			if (dist < min_dist)
				min_dist = dist;
		}
		score += min_dist;
	}
	return score;
}

inline double FitnessScore(std::vector<gr::Point3D<float>>const& Final, std::vector<gr::Point3D<float>>const& targ) {
	double score = 0.0;
	int Nm = Final.size();
	int Nd = targ.size();
	auto& model = Final;
	auto& data = targ;
	for (int i = 0; i < Nd; i++)
	{
		double min_dist = 1e10;
		for (int j = 0; j < Nm; j++)
		{
			double dist = (model[j].x() - data[i].x()) * (model[j].x() - data[i].x()) +
				(model[j].y() - data[i].y()) * (model[j].y() - data[i].y()) +
				(model[j].z() - data[i].z()) * (model[j].z() - data[i].z());
			if (dist < min_dist)
				min_dist = dist;
		}
		score += min_dist;
	}
	return score;
}



void readConfig(std::string FName, GoICP& goicp)
{
	// Open and parse the associated config file
	ConfigMap config(FName.c_str());

	goicp.MSEThresh = config.getF("MSEThresh");
	goicp.initNodeRot.a = config.getF("rotMinX");
	goicp.initNodeRot.b = config.getF("rotMinY");
	goicp.initNodeRot.c = config.getF("rotMinZ");
	goicp.initNodeRot.w = config.getF("rotWidth");
	goicp.initNodeTrans.x = config.getF("transMinX");
	goicp.initNodeTrans.y = config.getF("transMinY");
	goicp.initNodeTrans.z = config.getF("transMinZ");
	goicp.initNodeTrans.w = config.getF("transWidth");
	goicp.trimFraction = config.getF("trimFraction");
	// If < 0.1% trimming specified, do no trimming
	if (goicp.trimFraction < 0.001)
	{
		goicp.doTrim = false;
	}
	goicp.dt.SIZE = config.getI("distTransSize");
	goicp.dt.expandFactor = config.getF("distTransExpandFactor");

	cout << "CONFIG:" << endl;
	config.print();
	//cout << "(doTrim)->(" << goicp.doTrim << ")" << endl;
	cout << endl;
}

inline void getTxtFilenames(const std::string& dir, std::vector<std::string>& txts) {
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(dir).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getTxtFilenames(p.assign(dir).append("\\").append(fileinfo.name), txts);
			}
			else {
				if (strcmp(fileinfo.name + strlen(fileinfo.name) - 4, ".txt") == 0)
					txts.push_back(p.assign(dir).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

ofstream fout("result.txt", ios::app);
// 基础情况
inline void output() { fout << std::endl; }

// 递归函数
template<typename T, typename...Args>
inline void output(T first, Args...args) {
	fout << first;
	output(args...);
}


inline std::vector<pcl_utils::PointCloudXYZ::Ptr> readAllTXTPointCloudFrom(const std::string& file_dir) {
	std::vector <pcl_utils::PointCloudXYZ::Ptr> point_clouds;
	std::vector<std::string> txt_files;
	getTxtFilenames(file_dir, txt_files);
	for (auto const& txt_file : txt_files) {
		point_clouds.push_back(pcl_utils::io::open(txt_file));
	}
	return point_clouds;
}

void PCLPointCloud2POINT3Ds(pcl_utils::PointCloudXYZ::Ptr cloud, std::vector<gr::Point3D<float>>& points) {
	for (auto const& point : cloud->points) {
		points.push_back(gr::Point3D<float>(point.x, point.y, point.z));
	}
}

void POINT3Ds2PCLPointCloud(std::vector<gr::Point3D<float>>const& points, pcl_utils::PointCloudXYZ::Ptr cloud) {
	for (auto const& point : points) {
		cloud->points.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
	}
}


void Performance_test(std::string const& file_dir_src, std::string const& file_dir_tgt) {
	// 对pcl::FPCS, pcl_utils::M4PCS进行测试, 以及对比goicp, super4pcs
	std::vector<pcl_utils::PointCloudXYZ::Ptr> src_clouds = readAllTXTPointCloudFrom(file_dir_src);
	std::vector<pcl_utils::PointCloudXYZ::Ptr> tgt_clouds = readAllTXTPointCloudFrom(file_dir_tgt);
	std::vector<std::vector<gr::Point3D<float>>> src_points;
	std::vector<std::vector<gr::Point3D<float>>> tgt_points;
	std::vector<POINT3D*> src_points_goicp(src_clouds.size());
	std::vector<POINT3D*> tgt_points_goicp(tgt_clouds.size());
	size_t num_pairs = src_clouds.size();
	for (size_t i = 0; i < num_pairs; ++i) {
		auto src_cloud = src_clouds[i];
		auto tgt_cloud = tgt_clouds[i];
		std::vector<gr::Point3D<float>> src_point;
		std::vector<gr::Point3D<float>> tgt_point;
		PCLPointCloud2POINT3Ds(src_cloud, src_point);
		PCLPointCloud2POINT3Ds(tgt_cloud, tgt_point);
		src_points.push_back(std::move(src_point));
		tgt_points.push_back(std::move(tgt_point));

		// goicp
		loadPointCloud(&src_points_goicp[i], src_cloud);
		loadPointCloud(&tgt_points_goicp[i], tgt_cloud);
	}
	size_t num_clouds = src_clouds.size();
	{
		// 对pcl::FPCS进行测试
		std::cout << "pcl::FPCS Begin" << std::endl;
		long long time = clock();
		double score = 0.0;

		for (size_t i = 0; i < num_clouds; ++i) {
			auto src_cloud = src_clouds[i];
			auto tgt_cloud = tgt_clouds[i];
			auto fpcs = pcl_utils::registration::FourPCS(src_cloud, tgt_cloud);
			auto trans = fpcs.getFinalTransformation();
			pcl_utils::PointCloudXYZ::Ptr new_src_cloud(new pcl_utils::PointCloudXYZ);
			pcl::transformPointCloud(*src_cloud, *new_src_cloud, trans);
			score += FitnessScore(new_src_cloud, tgt_cloud);
		}

		time = clock() - time;
		double avg_score = score / num_clouds;
		double avg_time = time / num_clouds;
		output("pcl::FPCS", " 总耗时: ", time, " 平均耗时: ", avg_time, " 总得分: ", score, "平均得分: ", avg_score);
	}

	{
		// 测试pcl::icp
		std::cout << "pcl::icp Begin" << std::endl;
		long long time = 0;
		double score = 0.0;
		for (size_t i = 0; i < num_clouds; ++i) {
			auto src_cloud = src_clouds[i];
			auto tgt_cloud = tgt_clouds[i];
			Eigen::Matrix4f trans;
			auto icp = pcl_utils::registration::ICP(src_cloud, tgt_cloud, trans, 100);
			pcl_utils::PointCloudXYZ::Ptr new_src_cloud(new pcl_utils::PointCloudXYZ);
			pcl::transformPointCloud(*src_cloud, *new_src_cloud, trans);
			score += FitnessScore(new_src_cloud, tgt_cloud);
		}
		time = clock() - time;
		double avg_score = score / num_clouds;
		double avg_time = time / num_clouds;
		output("pcl::ICP", " 总耗时: ", time, " 平均耗时: ", avg_time, " 总得分: ", score, "平均得分: ", avg_score);
	}

	{
		// 测试goicp
		std::cout << "goicp Begin" << std::endl;
		long long time = 0;
		double score = 0.0;
		GoICP goicp;
		readConfig("config_example.txt", goicp);
		for (size_t i = 0; i < num_clouds; ++i) {
			
			goicp.pModel = src_points_goicp[i];
			goicp.pData = tgt_points_goicp[i];
			goicp.Nm = src_clouds[i]->size();
			goicp.Nd = tgt_clouds[i]->size();
			goicp.BuildDT();
			score += goicp.Register();
		}
		time = clock() - time;
		double avg_score = score / num_clouds;
		double avg_time = time / num_clouds;
		output("goicp", " 总耗时: ", time, " 平均耗时: ", avg_time, " 总得分: ", score, "平均得分: ", avg_score);
	}
	{
		// 测试super4pcs
		std::cout << "super4pcs Begin" << std::endl;
		long long time = 0;
		double score = 0.0;
		for (size_t i = 0; i < num_clouds; ++i) {
			auto src_point = src_points[i];
			auto tgt_point = tgt_points[i];
			OptionType options;
			Eigen::Matrix4f  trans(Eigen::Matrix4f::Identity());
			SamplerType sampler;
			TrVisitorType visitor;
			options.sample_size = n_points[0];
			options.max_time_seconds = 10;
			options.delta = deltas[0];
			MatcherType matcher(options, logger);
			score += matcher.ComputeTransformation(src_point, tgt_point, trans, sampler, visitor);
			pcl_utils::PointCloudXYZ::Ptr src_cloud(new pcl_utils::PointCloudXYZ);
			pcl_utils::PointCloudXYZ::Ptr tgt_cloud(new pcl_utils::PointCloudXYZ);
			POINT3Ds2PCLPointCloud(src_point, src_cloud);
			POINT3Ds2PCLPointCloud(tgt_point, tgt_cloud);
			pcl_utils::PointCloudXYZ::Ptr new_src_cloud(new pcl_utils::PointCloudXYZ);
			pcl::transformPointCloud(*src_cloud, *new_src_cloud, trans);
			score += FitnessScore(new_src_cloud, tgt_cloud);
		}
		time = clock() - time;
		double avg_score = score / num_clouds;
		double avg_time = time / num_clouds;
		output("super4pcs", " 总耗时: ", time, " 平均耗时: ", avg_time, " 总得分: ", score, "平均得分: ", avg_score);
	}
	{
		// 测试pcl_utils::M4PCS
		std::cout << "pcl_utils::M4PCS Begin" << std::endl;
		long long time = 0;
		double score = 0.0;
		for (size_t i = 0; i < num_clouds; ++i) {
			auto src_cloud = src_clouds[i];
			auto tgt_cloud = tgt_clouds[i];
			Eigen::Matrix4f trans;
			pcl_utils::PointCloudXYZ::Ptr Final(new pcl_utils::PointCloudXYZ);
			pcl_utils::M4PCS<pcl::FPFHSignature33> m4pcs;
			m4pcs.setInputSource(src_cloud);
			m4pcs.setInputTarget(tgt_cloud);
			m4pcs.setSearchMethodSource(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
			m4pcs.setSearchMethodTarget(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
			m4pcs.align(*Final);
			score += FitnessScore(Final, tgt_cloud);
		}
		time = clock() - time;
		double avg_score = score / num_clouds;
		double avg_time = time / num_clouds;
		output("pcl_utils::M4PCS", " 总耗时: ", time, " 平均耗时: ", avg_time, " 总得分: ", score, "平均得分: ", avg_score);
	}

	// 释放内存
	for (size_t i = 0; i < num_clouds; ++i) {
		free(src_points_goicp[i]);
		free(tgt_points_goicp[i]);
	}
}


void test1() {
	std::string file_dir_src = ".//resource//点云配准数据_tree//LiDAR";
	std::string file_dir_tgt = ".//resource//点云配准数据_tree//MVS";
	Performance_test(file_dir_src, file_dir_tgt);
}

void test2() {
	std::string file_dir_src = ".//resource//Faces//Face1.pcd";
	std::string file_dir_tgt = ".//resource//Faces//Face2.pcd";
	// 对pcl::FPCS, pcl_utils::M4PCS进行测试, 以及对比goicp, super4pcs, 将配准结果可视化
	pcl_utils::PointCloudXYZ::Ptr src_clouds(new pcl_utils::PointCloudXYZ);
	pcl_utils::PointCloudXYZ::Ptr tgt_clouds(new pcl_utils::PointCloudXYZ);
	src_clouds = pcl_utils::io::open(file_dir_tgt);
	tgt_clouds = pcl_utils::io::open(file_dir_src);
	auto fpcs = pcl_utils::registration::FourPCS(src_clouds, tgt_clouds);
	auto T = fpcs.getFinalTransformation();
	pcl_utils::PointCloudXYZ::Ptr new_src_cloud(new pcl_utils::PointCloudXYZ);
	pcl::transformPointCloud(*src_clouds, *new_src_cloud, T);
	pcl_utils::PointCloudXYZ::Ptr new_src_cloud_tgt(new pcl_utils::PointCloudXYZ);
	*new_src_cloud_tgt = *new_src_cloud + *tgt_clouds;
	pcl_utils::vis::Visualize(*new_src_cloud_tgt, "PCl::FPCS");
	// icp
	auto icp = pcl_utils::registration::ICP(src_clouds, tgt_clouds, T, 100);
	pcl::transformPointCloud(*src_clouds, *new_src_cloud, T);
	*new_src_cloud = *new_src_cloud + *tgt_clouds;
	pcl_utils::vis::Visualize(*new_src_cloud, "pcl::icp");
	// goicp
	GoICP goicp;
	readConfig("config_example.txt", goicp);
	goicp.pModel = (POINT3D*)malloc(sizeof(POINT3D) * src_clouds->size());
	goicp.pData = (POINT3D*)malloc(sizeof(POINT3D) * tgt_clouds->size());
	goicp.Nm = src_clouds->size();
	goicp.Nd = tgt_clouds->size();
	for (int i = 0; i < src_clouds->size(); i++)
	{
		goicp.pModel[i].x = src_clouds->points[i].x;
		goicp.pModel[i].y = src_clouds->points[i].y;
		goicp.pModel[i].z = src_clouds->points[i].z;
	}
	for (int i = 0; i < tgt_clouds->size(); i++)
	{
		goicp.pData[i].x = tgt_clouds->points[i].x;
		goicp.pData[i].y = tgt_clouds->points[i].y;
		goicp.pData[i].z = tgt_clouds->points[i].z;
	}
	goicp.BuildDT();
	goicp.Register();
	pcl_utils::PointCloudXYZ::Ptr new_src_cloud_goicp(new pcl_utils::PointCloudXYZ);
	for (int i = 0; i < src_clouds->size(); i++)
	{
		new_src_cloud_goicp->points.push_back(pcl::PointXYZ(goicp.pModel[i].x, goicp.pModel[i].y, goicp.pModel[i].z));
	}
	pcl_utils::PointCloudXYZ::Ptr new_src_cloud_goicp_tgt(new pcl_utils::PointCloudXYZ);
	*new_src_cloud_goicp_tgt = *new_src_cloud_goicp + *tgt_clouds;
	pcl_utils::vis::Visualize(*new_src_cloud_goicp_tgt, "goicp");
	// super4pcs
	OptionType options;
	Eigen::Matrix4f  trans(Eigen::Matrix4f::Identity());
	SamplerType sampler;
	TrVisitorType visitor;
	options.sample_size = n_points[0];
	options.max_time_seconds = 10;
	options.delta = deltas[0];
	MatcherType matcher(options, logger);
	std::vector<gr::Point3D<float>> src_point;
	std::vector<gr::Point3D<float>> tgt_point;
	PCLPointCloud2POINT3Ds(src_clouds, src_point);
	PCLPointCloud2POINT3Ds(tgt_clouds, tgt_point);
	double score = matcher.ComputeTransformation(src_point, tgt_point, trans, sampler, visitor);
	pcl_utils::PointCloudXYZ::Ptr src_cloud(new pcl_utils::PointCloudXYZ);
	pcl_utils::PointCloudXYZ::Ptr tgt_cloud(new pcl_utils::PointCloudXYZ);
	POINT3Ds2PCLPointCloud(src_point, src_cloud);
	POINT3Ds2PCLPointCloud(tgt_point, tgt_cloud);
	pcl::transformPointCloud(*src_cloud, *new_src_cloud, trans);
	pcl_utils::PointCloudXYZ::Ptr new_src_cloud_super4pcs_tgt(new pcl_utils::PointCloudXYZ);
	*new_src_cloud_super4pcs_tgt = *new_src_cloud + *tgt_cloud;
	pcl_utils::vis::Visualize(*new_src_cloud_super4pcs_tgt, "super4pcs");
}



#endif // !_PERFORMANCE_TEST_H_