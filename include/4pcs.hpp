#pragma once
#ifndef __4PCS_H_
#define __4PCS_H_

#include "pcl_utility.h"
#include "pcl_utility_template.hpp"

#include "random_engine.h"

namespace pcl_utils {


	template <typename PointT>
	inline float
		getMeanPointDensity(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
			float max_dist,
			int nr_threads = 1);

	template <typename PointT>
	inline float
		getMeanPointDensity(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
			const pcl::Indices& indices,
			float max_dist,
			int nr_threads = 1);

	template<typename FeatureType>
	class M4PCS : public pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float> {
		/**
		*  修改了关于selectBase和selectBaseTriangle的代码, 使得其可以使用sobol序列进行随机采样
		*  
		*/
	private:
		using Scalar = float;
		using PointCloudSource = pcl_utils::PointCloudXYZ;
		using PointCloudTarget = pcl_utils::PointCloudXYZ;
		using PointSource = pcl::PointXYZ;
		using PointTarget = pcl::PointXYZ;
        using NormalT = pcl::Normal;
		using MatchingCandidate = pcl::registration::MatchingCandidate;
		using MatchingCandidates = pcl::registration::MatchingCandidates;
		// Member Variables Private
		PointCloudSource::Ptr src;
		PointCloudTarget::Ptr targ;
	public:
		M4PCS() :
			nr_threads_(1),
			approx_overlap_(0.5f),
			delta_(1.f),
			score_threshold_(std::numeric_limits<float>::max()),
			nr_samples_(0),
			max_norm_diff_(90.f),
			max_runtime_(0),
			fitness_score_(std::numeric_limits<float>::max()),
			diameter_(),
			max_base_diameter_sqr_(),
			use_normals_(false),
			normalize_delta_(true),
			max_pair_diff_(),
			max_edge_diff_(),
			coincidation_limit_(),
			max_mse_(),
			max_inlier_dist_sqr_(),
			small_error_(0.00001f),
			sobolSampler(nullptr)
		{
			reg_name_ = "pcl_utils::M4PCSAlignment";
			max_iterations_ = 0;
			ransac_iterations_ = 1000;
			transformation_estimation_.reset(new pcl::registration::TransformationEstimation3Point<PointSource, PointTarget>);
		}
		~M4PCS() { if (sobolSampler) delete sobolSampler; }

		void setSrc(pcl::PointCloud<PointSource>::Ptr src) {
			this->src = src;
		}

		void setTarg(pcl::PointCloud<PointSource>::Ptr targ) {
			this->targ = targ;
		}

		int nr_threads_;

		float approx_overlap_;

		float delta_;

		float score_threshold_;

		int nr_samples_;

		float max_norm_diff_;

		int max_runtime_;

		float fitness_score_;

		float diameter_;

		float max_base_diameter_sqr_;

		bool use_normals_;

		bool normalize_delta_;

		pcl::IndicesPtr source_indices_;

		pcl::IndicesPtr target_indices_;

		float max_pair_diff_;

		float max_edge_diff_;

		float coincidation_limit_;

		float max_mse_;

		float max_inlier_dist_sqr_;

		const float small_error_;

		inline void
			setNumberOfThreads(int nr_threads)
		{
			nr_threads_ = nr_threads;
		};

		inline int
			getNumberOfThreads() const
		{
			return (nr_threads_);
		};

		inline void
			setDelta(float delta, bool normalize = false)
		{
			delta_ = delta;
			normalize_delta_ = normalize;
		};

		inline float
			getDelta() const
		{
			return (delta_);
		};

		inline void
			setApproxOverlap(float approx_overlap)
		{
			approx_overlap_ = approx_overlap;
		};

		inline float
			getApproxOverlap() const
		{
			return (approx_overlap_);
		};

		inline void
			setScoreThreshold(float score_threshold)
		{
			score_threshold_ = score_threshold;
		};

		inline float
			getScoreThreshold() const
		{
			return (score_threshold_);
		};

		inline void
			setNumberOfSamples(int nr_samples)
		{
			nr_samples_ = nr_samples;
		};

		inline int
			getNumberOfSamples() const
		{
			return (nr_samples_);
		};

		inline void
			setMaxNormalDifference(float max_norm_diff)
		{
			max_norm_diff_ = max_norm_diff;
		};

		inline float
			getMaxNormalDifference() const
		{
			return (max_norm_diff_);
		};

		inline void
			setMaxComputationTime(int max_runtime)
		{
			max_runtime_ = max_runtime;
		};

		inline int
			getMaxComputationTime() const
		{
			return (max_runtime_);
		};

		inline float
			getFitnessScore() const
		{
			return (fitness_score_);
		};

		using pcl::PCLBase<PointSource>::deinitCompute;
		using pcl::PCLBase<PointSource>::input_;
		using pcl::PCLBase<PointSource>::indices_;

		using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
		using pcl:: Registration<PointSource, PointTarget, Scalar>::tree_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::correspondences_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::target_cloud_updated_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::max_iterations_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::ransac_iterations_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::transformation_estimation_;
		using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;

		void
			computeTransformation(PointCloudSource& output,
				const Eigen::Matrix4f& guess) override;
		virtual bool
			initCompute();

		int
			selectBase(pcl::Indices& base_indices, float(&ratio)[2]);

		int
			selectBaseTriangle(pcl::Indices& base_indices);

		void
			setupBase(pcl::Indices& base_indices, float(&ratio)[2]);

		float
			segmentToSegmentDist(const pcl::Indices& base_indices, float(&ratio)[2]);

		virtual int
			bruteForceCorrespondences(int idx1, int idx2, pcl::Correspondences& pairs);

		virtual int
			determineBaseMatches(const pcl::Indices& base_indices,
				std::vector<pcl::Indices>& matches,
				const pcl::Correspondences& pairs_a,
				const pcl::Correspondences& pairs_b,
				const float(&ratio)[2]);

		int
			checkBaseMatch(const pcl::Indices& match_indices, const float(&ds)[4]);

		virtual void
			handleMatches(const pcl::Indices& base_indices,
				std::vector<pcl::Indices>& matches,
				MatchingCandidates& candidates);

		virtual void
			linkMatchWithBase(const pcl::Indices& base_indices,
				pcl::Indices& match_indices,
				pcl::Correspondences& correspondences);

		virtual int
			validateMatch(const pcl::Indices& base_indices,
				const pcl::Indices& match_indices,
				const pcl::Correspondences& correspondences,
				Eigen::Matrix4f& transformation);

		virtual int
			validateTransformation(Eigen::Matrix4f& transformation, float& fitness_score);

		virtual void
			finalCompute(const std::vector<MatchingCandidates>& candidates);

        pcl_utils::PointCloudNormal::Ptr source_normals_;
        pcl_utils::PointCloudNormal::Ptr target_normals_;
		// new added member variables
		vector<pcl::index_t> corres_src_idx;
		vector<pcl::index_t> corres_targ_idx;
		
		pcl_utils::PointCloudXYZ::Ptr corres_src;
		pcl_utils::PointCloudXYZ::Ptr corres_targ;

		pcl::PointCloud<pcl::Normal>::Ptr source_baseSet_normals_;
		pcl::PointCloud<pcl::Normal>::Ptr target_baseSet_normals_;
		
		std::shared_ptr<pcl::PointCloud<FeatureType>> Feature_src;
		std::shared_ptr<pcl::PointCloud<FeatureType>> Feature_targ;
		SobolSampler* sobolSampler;


		// new added member functions
		void extractCorrespondentIdx(std::vector<pcl::index_t>& src_idx, std::vector<pcl::index_t>& targ_idx);
		std::pair< pcl_utils::PointCloudXYZ::Ptr, pcl_utils::PointCloudXYZ::Ptr> 
			extractCorrespondentPoint(std::vector<pcl::index_t>& src_base_idx, std::vector<pcl::index_t>& targ_base_idx);
		static Eigen::Matrix3f PCA(pcl_utils::PointCloudXYZ::Ptr);
		static void sortByPCA_Axis(pcl_utils::PointCloudXYZ::Ptr, Eigen::Matrix3f const&);
	};
}

template <typename PointT>
inline float
pcl_utils::getMeanPointDensity(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
	float max_dist,
	int nr_threads)
{
	const float max_dist_sqr = max_dist * max_dist;
	const std::size_t s = cloud->size();

	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(cloud);

	float mean_dist = 0.f;
	int num = 0;
	pcl::Indices ids(2);
	std::vector<float> dists_sqr(2);

	pcl::utils::ignore(nr_threads);
#pragma omp parallel for \
  default(none) \
  shared(tree, cloud) \
  firstprivate(ids, dists_sqr) \
  reduction(+:mean_dist, num) \
  firstprivate(s, max_dist_sqr) \
  num_threads(nr_threads)
	for (int i = 0; i < 1000; i++) {
		tree.nearestKSearch((*cloud)[rand() % s], 2, ids, dists_sqr);
		if (dists_sqr[1] < max_dist_sqr) {
			mean_dist += std::sqrt(dists_sqr[1]);
			num++;
		}
	}

	return (mean_dist / num);
};

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline float
pcl_utils::getMeanPointDensity(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
	const pcl::Indices& indices,
	float max_dist,
	int nr_threads)
{
	const float max_dist_sqr = max_dist * max_dist;
	const std::size_t s = indices.size();

	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(cloud);

	float mean_dist = 0.f;
	int num = 0;
	pcl::Indices ids(2);
	std::vector<float> dists_sqr(2);

	pcl::utils::ignore(nr_threads);
#if OPENMP_LEGACY_CONST_DATA_SHARING_RULE
#pragma omp parallel for \
  default(none) \
  shared(tree, cloud, indices) \
  firstprivate(ids, dists_sqr) \
  reduction(+:mean_dist, num) \
  num_threads(nr_threads)
#else
#pragma omp parallel for \
  default(none) \
  shared(tree, cloud, indices, s, max_dist_sqr) \
  firstprivate(ids, dists_sqr) \
  reduction(+:mean_dist, num) \
  num_threads(nr_threads)
#endif
	for (int i = 0; i < 1000; i++) {
		tree.nearestKSearch((*cloud)[indices[rand() % s]], 2, ids, dists_sqr);
		if (dists_sqr[1] < max_dist_sqr) {
			mean_dist += std::sqrt(dists_sqr[1]);
			num++;
		}
	}

	return (mean_dist / num);
};

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
void
pcl_utils::M4PCS<FeatureType>::
computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess)
{
    if (!initCompute())
        return;

    final_transformation_ = guess;
    bool abort = false;
    std::vector<MatchingCandidates> all_candidates(max_iterations_);
    pcl::StopWatch timer;
    {
        const unsigned int seed =
            static_cast<unsigned int>(std::time(NULL)) ^ omp_get_thread_num();
        std::srand(seed);
        PCL_DEBUG("[%s::computeTransformation] Using seed=%u\n", reg_name_.c_str(), seed);

        for (int i = 0; i < max_iterations_; i++) {


            MatchingCandidates candidates(1);
            pcl::Indices base_indices(4);
            all_candidates[i] = candidates;

            if (!abort) {
                float ratio[2];
                // select four coplanar point base
                if (selectBase(base_indices, ratio) == 0) {
                    // calculate candidate pair correspondences using diagonal lengths of base
                    pcl::Correspondences pairs_a, pairs_b;
                    if (bruteForceCorrespondences(base_indices[0], base_indices[1], pairs_a) ==
                        0 &&
                        bruteForceCorrespondences(base_indices[2], base_indices[3], pairs_b) ==
                        0) {
                        // determine candidate matches by combining pair correspondences based on
                        // segment distances
                        std::vector<pcl::Indices> matches;
                        if (determineBaseMatches(base_indices, matches, pairs_a, pairs_b, ratio) ==
                            0) {
                            // check and evaluate candidate matches and store them
                            handleMatches(base_indices, matches, candidates);
                            if (!candidates.empty())
                                all_candidates[i] = candidates;
                        }
                    }
                }

                // check terminate early (time or fitness_score threshold reached)
                abort = (!candidates.empty() ? candidates[0].fitness_score < score_threshold_
                    : abort);
                abort = (abort ? abort : timer.getTimeSeconds() > max_runtime_);
            }
        }
    }

    // determine best match over all tries
    finalCompute(all_candidates);

    // apply the final transformation
    pcl::transformPointCloud(*input_, output, final_transformation_);

    deinitCompute();
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
bool
pcl_utils::M4PCS<FeatureType>::
initCompute()
{
    const unsigned int seed = std::time(nullptr);
    std::srand(seed);
    PCL_DEBUG("[%s::initCompute] Using seed=%u\n", reg_name_.c_str(), seed);

    // basic pcl initialization
    if (!pcl::PCLBase<PointSource>::initCompute())
        return (false);

    // check if source and target are given
    if (!input_ || !target_) {
        PCL_ERROR("[%s::initCompute] Source or target dataset not given!\n",
            reg_name_.c_str());
        return (false);
    }

    if (!target_indices_ || target_indices_->empty()) {
        target_indices_.reset(new pcl::Indices(target_->size()));
        int index = 0;
        for (auto& target_index : *target_indices_)
            target_index = index++;
        target_cloud_updated_ = true;
    }

    // if a sample size for the point clouds is given; preferably no sampling of target
    // cloud
    if (nr_samples_ != 0) {
        const int ss = static_cast<int>(indices_->size());
        const int sample_fraction_src = std::max(1, static_cast<int>(ss / nr_samples_));

        source_indices_ = pcl::IndicesPtr(new pcl::Indices);
        for (int i = 0; i < ss; i++)
            if (rand() % sample_fraction_src == 0)
                source_indices_->push_back((*indices_)[i]);
    }
    else
        source_indices_ = indices_;

    // check usage of normals
    if (this->source_normals_ && target_normals_ && source_normals_->size() == input_->size() &&
        target_normals_->size() == target_->size())
        use_normals_ = true;

    // set up tree structures
    if (target_cloud_updated_) {
        tree_->setInputCloud(target_, target_indices_);
        target_cloud_updated_ = false;
    }

    // set predefined variables
    constexpr int min_iterations = 4;
    constexpr float diameter_fraction = 0.3f;

    // get diameter of input cloud (distance between farthest points)
    Eigen::Vector4f pt_min, pt_max;
    pcl::getMinMax3D(*target_, *target_indices_, pt_min, pt_max);
    diameter_ = (pt_max - pt_min).norm();

    // derive the limits for the random base selection
    float max_base_diameter = diameter_ * approx_overlap_ * 2.f;
    max_base_diameter_sqr_ = max_base_diameter * max_base_diameter;

    // normalize the delta
    if (normalize_delta_) {
        float mean_dist = pcl_utils::getMeanPointDensity<PointTarget>(
            target_, *target_indices_, 0.05f * diameter_, nr_threads_);
        delta_ *= mean_dist;
    }

    // heuristic determination of number of trials to have high probability of finding a
    // good solution
    if (max_iterations_ == 0) {
        float first_est = std::log(small_error_) /
            std::log(1.0 - std::pow(static_cast<double>(approx_overlap_),
                static_cast<double>(min_iterations)));
        max_iterations_ =
            static_cast<int>(first_est / (diameter_fraction * approx_overlap_ * 2.f));
    }

    // set further parameter
    if (score_threshold_ == std::numeric_limits<float>::max())
        score_threshold_ = 1.f - approx_overlap_;

    if (max_iterations_ < 4)
        max_iterations_ = 4;

    if (max_runtime_ < 1)
        max_runtime_ = std::numeric_limits<int>::max();

    // calculate internal parameters based on the the estimated point density
    max_pair_diff_ = delta_ * 2.f;
    max_edge_diff_ = delta_ * 4.f;
    coincidation_limit_ = delta_ * 2.f; // EDITED: originally std::sqrt (delta_ * 2.f)
    max_mse_ = powf(delta_ * 2.f, 2.f);
    max_inlier_dist_sqr_ = powf(delta_ * 2.f, 2.f);

    // reset fitness_score
    fitness_score_ = std::numeric_limits<float>::max();

    return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
selectBase(pcl::Indices& base_indices, float(&ratio)[2])
{
    const float too_close_sqr = max_base_diameter_sqr_ * 0.01;

    Eigen::VectorXf coefficients(4);
    pcl::SampleConsensusModelPlane<PointTarget> plane(corres_targ); // 可以用更快速的平面判断方法, 不使用ransac
    plane.setIndices(corres_targ_idx);
    Eigen::Vector4f centre_pt;
    float nearest_to_plane = std::numeric_limits<float>::max();

    // repeat base search until valid quadruple was found or ransac_iterations_ number of
    // tries were unsuccessful
    for (int i = 0; i < ransac_iterations_; i++) {
        // random select an appropriate point triple
        if (selectBaseTriangle(base_indices) < 0)
            continue;

        pcl::Indices base_triple(base_indices.begin(), base_indices.end() - 1);
        plane.computeModelCoefficients(base_triple, coefficients);
        pcl::compute3DCentroid(*corres_targ, base_triple, centre_pt);

        // loop over all points in source cloud to find most suitable fourth point
        const PointTarget* pt1 = &((*corres_targ)[base_indices[0]]);
        const PointTarget* pt2 = &((*corres_targ)[base_indices[1]]);
        const PointTarget* pt3 = &((*corres_targ)[base_indices[2]]);

        for (const auto& corres_targ_index : corres_targ_idx) {
            const PointTarget* pt4 = &((*corres_targ)[corres_targ_index]);

            float d1 = pcl::squaredEuclideanDistance(*pt4, *pt1);
            float d2 = pcl::squaredEuclideanDistance(*pt4, *pt2);
            float d3 = pcl::squaredEuclideanDistance(*pt4, *pt3);
            float d4 = (pt4->getVector3fMap() - centre_pt.head(3)).squaredNorm();

            // check distance between points w.r.t minimum sampling distance; EDITED -> 4th
            // point now also limited by max base line
            if (d1 < too_close_sqr || d2 < too_close_sqr || d3 < too_close_sqr ||
                d4 < too_close_sqr || d1 > max_base_diameter_sqr_ ||
                d2 > max_base_diameter_sqr_ || d3 > max_base_diameter_sqr_)
                continue;

            // check distance to plane to get point closest to plane
            float dist_to_plane = pcl::pointToPlaneDistance(*pt4, coefficients);
            if (dist_to_plane < nearest_to_plane) {
                base_indices[3] = corres_targ_index;
                nearest_to_plane = dist_to_plane;
            }
        }

        // check if at least one point fulfilled the conditions
        if (nearest_to_plane != std::numeric_limits<float>::max()) {
            // order points to build largest quadrangle and calculate intersection ratios of
            // diagonals
            setupBase(base_indices, ratio);
            return (0);
        }
    }

    // return unsuccessful if no quadruple was selected
    return (-1);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
selectBaseTriangle(pcl::Indices& base_indices)
{
    const auto nr_points = corres_targ_idx.size();
    float best_t = 0.f;

    // choose random first point
    base_indices[0] = (corres_targ_idx)[rand() % nr_points];
    auto* index1 = base_indices.data();

    // random search for 2 other points (as far away as overlap allows)
    for (int i = 0; i < ransac_iterations_; i++) {
        auto* index2 = &(corres_targ_idx)[rand() % nr_points];
        auto* index3 = &(corres_targ_idx)[rand() % nr_points];

        Eigen::Vector3f u =
            (*corres_targ)[*index2].getVector3fMap() - (*corres_targ)[*index1].getVector3fMap();
        Eigen::Vector3f v =
            (*corres_targ)[*index3].getVector3fMap() - (*corres_targ)[*index1].getVector3fMap();
        float t =
            u.cross(v).squaredNorm(); // triangle area (0.5 * sqrt(t)) should be maximal

        // check for most suitable point triple
        if (t > best_t && u.squaredNorm() < max_base_diameter_sqr_ &&
            v.squaredNorm() < max_base_diameter_sqr_) {
            best_t = t;
            base_indices[1] = *index2;
            base_indices[2] = *index3;
        }
    }

    // return if a triplet could be selected
    return (best_t == 0.f ? -1 : 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
void
pcl_utils::M4PCS<FeatureType>::
setupBase(pcl::Indices& base_indices, float(&ratio)[2])
{
    float best_t = std::numeric_limits<float>::max();
    const pcl::Indices copy(base_indices.begin(), base_indices.end());
    pcl::Indices temp(base_indices.begin(), base_indices.end());

    // loop over all combinations of base points
    for (auto i = copy.begin(), i_e = copy.end(); i != i_e; ++i)
        for (auto j = copy.begin(), j_e = copy.end(); j != j_e; ++j) {
            if (i == j)
                continue;

            for (auto k = copy.begin(), k_e = copy.end(); k != k_e; ++k) {
                if (k == j || k == i)
                    continue;

                auto l = copy.begin();
                while (l == i || l == j || l == k)
                    ++l;

                temp[0] = *i;
                temp[1] = *j;
                temp[2] = *k;
                temp[3] = *l;

                // calculate diagonal intersection ratios and check for suitable segment to
                // segment distances
                float ratio_temp[2];
                float t = segmentToSegmentDist(temp, ratio_temp);
                if (t < best_t) {
                    best_t = t;
                    ratio[0] = ratio_temp[0];
                    ratio[1] = ratio_temp[1];
                    base_indices = temp;
                }
            }
        }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
float
pcl_utils::M4PCS<FeatureType>::
segmentToSegmentDist(const pcl::Indices& base_indices, float(&ratio)[2])
{
    // get point vectors
    Eigen::Vector3f u = (*corres_targ)[base_indices[1]].getVector3fMap() -
        (*corres_targ)[base_indices[0]].getVector3fMap();
    Eigen::Vector3f v = (*corres_targ)[base_indices[3]].getVector3fMap() -
        (*corres_targ)[base_indices[2]].getVector3fMap();
    Eigen::Vector3f w = (*corres_targ)[base_indices[0]].getVector3fMap() -
        (*corres_targ)[base_indices[2]].getVector3fMap();

    // calculate segment distances
    float a = u.dot(u);
    float b = u.dot(v);
    float c = v.dot(v);
    float d = u.dot(w);
    float e = v.dot(w);
    float D = a * c - b * b;
    float sN = 0.f, sD = D;
    float tN = 0.f, tD = D;

    // check segments
    if (D < small_error_) {
        sN = 0.f;
        sD = 1.f;
        tN = e;
        tD = c;
    }
    else {
        sN = (b * e - c * d);
        tN = (a * e - b * d);

        if (sN < 0.f) {
            sN = 0.f;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.f) {
        tN = 0.f;

        if (-d < 0.f)
            sN = 0.f;

        else if (-d > a)
            sN = sD;

        else {
            sN = -d;
            sD = a;
        }
    }

    else if (tN > tD) {
        tN = tD;

        if ((-d + b) < 0.f)
            sN = 0.f;

        else if ((-d + b) > a)
            sN = sD;

        else {
            sN = (-d + b);
            sD = a;
        }
    }

    // set intersection ratios
    ratio[0] = (std::abs(sN) < small_error_) ? 0.f : sN / sD;
    ratio[1] = (std::abs(tN) < small_error_) ? 0.f : tN / tD;

    Eigen::Vector3f x = w + (ratio[0] * u) - (ratio[1] * v);
    return (x.norm());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
bruteForceCorrespondences(int idx1, int idx2, pcl::Correspondences& pairs)
{
    const float max_norm_diff = 0.5f * max_norm_diff_ * M_PI / 180.f;

    // calculate reference segment distance and normal angle
    float ref_dist = pcl::euclideanDistance((*corres_targ)[idx1], (*corres_targ)[idx2]);
    float ref_norm_angle =
        (use_normals_ ? ((*target_normals_)[idx1].getNormalVector3fMap() -
            (*target_normals_)[idx2].getNormalVector3fMap())
            .norm()
            : 0.f);

    // loop over all pairs of points in source point cloud
    auto it_out = corres_src_idx.begin(), it_out_e = corres_src_idx.end() - 1;
    auto it_in_e = corres_src_idx.end();
    for (; it_out != it_out_e; it_out++) {
        auto it_in = it_out + 1;
        const PointSource* pt1 = &(*corres_src)[*it_out];
        for (; it_in != it_in_e; it_in++) {
            const PointSource* pt2 = &(*corres_src)[*it_in];

            // check point distance compared to reference dist (from base)
            float dist = pcl::euclideanDistance(*pt1, *pt2);
            if (std::abs(dist - ref_dist) < max_pair_diff_) {
                // add here normal evaluation if normals are given
                if (use_normals_) {
                    const NormalT* pt1_n = &((*source_normals_)[*it_out]);
                    const NormalT* pt2_n = &((*source_normals_)[*it_in]);

                    float norm_angle_1 =
                        (pt1_n->getNormalVector3fMap() - pt2_n->getNormalVector3fMap()).norm();
                    float norm_angle_2 =
                        (pt1_n->getNormalVector3fMap() + pt2_n->getNormalVector3fMap()).norm();

                    float norm_diff = std::min<float>(std::abs(norm_angle_1 - ref_norm_angle),
                        std::abs(norm_angle_2 - ref_norm_angle));
                    if (norm_diff > max_norm_diff)
                        continue;
                }

                pairs.push_back(pcl::Correspondence(*it_in, *it_out, dist));
                pairs.push_back(pcl::Correspondence(*it_out, *it_in, dist));
            }
        }
    }

    // return success if at least one correspondence was found
    return (pairs.empty() ? -1 : 0);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
determineBaseMatches(const pcl::Indices& base_indices,
    std::vector<pcl::Indices>& matches,
    const pcl::Correspondences& pairs_a,
    const pcl::Correspondences& pairs_b,
    const float(&ratio)[2])
{
    // calculate edge lengths of base
    float dist_base[4];
    dist_base[0] =
        pcl::euclideanDistance((*corres_targ)[base_indices[0]], (*corres_targ)[base_indices[2]]);
    dist_base[1] =
        pcl::euclideanDistance((*corres_targ)[base_indices[0]], (*corres_targ)[base_indices[3]]);
    dist_base[2] =
        pcl::euclideanDistance((*corres_targ)[base_indices[1]], (*corres_targ)[base_indices[2]]);
    dist_base[3] =
        pcl::euclideanDistance((*corres_targ)[base_indices[1]], (*corres_targ)[base_indices[3]]);

    // loop over first point pair correspondences and store intermediate points 'e' in new
    // point cloud
    PointCloudSourcePtr cloud_e(new PointCloudSource);
    cloud_e->resize(pairs_a.size() * 2);
    auto it_pt = cloud_e->begin();
    for (const auto& pair : pairs_a) {
        const PointSource* pt1 = &((*corres_targ)[pair.index_match]);
        const PointSource* pt2 = &((*corres_targ)[pair.index_query]);

        // calculate intermediate points using both ratios from base (r1,r2)
        for (int i = 0; i < 2; i++, it_pt++) {
            it_pt->x = pt1->x + ratio[i] * (pt2->x - pt1->x);
            it_pt->y = pt1->y + ratio[i] * (pt2->y - pt1->y);
            it_pt->z = pt1->z + ratio[i] * (pt2->z - pt1->z);
        }
    }

    // initialize new kd tree of intermediate points from first point pair correspondences
    KdTreeReciprocalPtr tree_e(new KdTreeReciprocal);
    tree_e->setInputCloud(cloud_e);

    pcl::Indices ids;
    std::vector<float> dists_sqr;

    // loop over second point pair correspondences
    for (const auto& pair : pairs_b) {
        const PointTarget* pt1 = &((*corres_src)[pair.index_match]);
        const PointTarget* pt2 = &((*corres_src)[pair.index_query]);

        // calculate intermediate points using both ratios from base (r1,r2)
        for (const float& r : ratio) {
            PointTarget pt_e;
            pt_e.x = pt1->x + r * (pt2->x - pt1->x);
            pt_e.y = pt1->y + r * (pt2->y - pt1->y);
            pt_e.z = pt1->z + r * (pt2->z - pt1->z);

            // search for corresponding intermediate points
            tree_e->radiusSearch(pt_e, coincidation_limit_, ids, dists_sqr);
            for (const auto& id : ids) {
                pcl::Indices match_indices(4);

                match_indices[0] =
                    pairs_a[static_cast<int>(std::floor((id / 2.f)))].index_match;
                match_indices[1] =
                    pairs_a[static_cast<int>(std::floor((id / 2.f)))].index_query;
                match_indices[2] = pair.index_match;
                match_indices[3] = pair.index_query;

                // EDITED: added coarse check of match based on edge length (due to rigid-body )
                if (checkBaseMatch(match_indices, dist_base) < 0)
                    continue;

                matches.push_back(match_indices);
            }
        }
    }

    // return unsuccessful if no match was found
    return (!matches.empty() ? 0 : -1);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
checkBaseMatch(const pcl::Indices& match_indices, const float(&dist_ref)[4])
{
    float d0 =
        pcl::euclideanDistance((*input_)[match_indices[0]], (*input_)[match_indices[2]]);
    float d1 =
        pcl::euclideanDistance((*input_)[match_indices[0]], (*input_)[match_indices[3]]);
    float d2 =
        pcl::euclideanDistance((*input_)[match_indices[1]], (*input_)[match_indices[2]]);
    float d3 =
        pcl::euclideanDistance((*input_)[match_indices[1]], (*input_)[match_indices[3]]);

    // check edge distances of match w.r.t the base
    return (std::abs(d0 - dist_ref[0]) < max_edge_diff_ &&
        std::abs(d1 - dist_ref[1]) < max_edge_diff_ &&
        std::abs(d2 - dist_ref[2]) < max_edge_diff_ &&
        std::abs(d3 - dist_ref[3]) < max_edge_diff_)
        ? 0
        : -1;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
void
pcl_utils::M4PCS<FeatureType>::
handleMatches(const pcl::Indices& base_indices,
    std::vector<pcl::Indices>& matches,
    MatchingCandidates& candidates)
{
    candidates.resize(1);
    float fitness_score = std::numeric_limits<float>::max();

    // loop over all Candidate matches
    for (auto& match : matches) {
        Eigen::Matrix4f transformation_temp;
        pcl::Correspondences correspondences_temp;

        // determine corresondences between base and match according to their distance to
        // centroid
        linkMatchWithBase(base_indices, match, correspondences_temp);

        // check match based on residuals of the corresponding points after
        if (validateMatch(base_indices, match, correspondences_temp, transformation_temp) <
            0)
            continue;

        // check resulting  using a sub sample of the source point cloud and compare to
        // previous matches
        if (validateTransformation(transformation_temp, fitness_score) < 0)
            continue;

        // store best match as well as associated fitness_score and transformation
        candidates[0].fitness_score = fitness_score;
        candidates[0].transformation = transformation_temp;
        correspondences_temp.erase(correspondences_temp.end() - 1);
        candidates[0].correspondences = correspondences_temp;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
void
pcl_utils::M4PCS<FeatureType>::
linkMatchWithBase(const pcl::Indices& base_indices,
    pcl::Indices& match_indices,
    pcl::Correspondences& correspondences)
{
    // calculate centroid of base and target
    Eigen::Vector4f centre_base{ 0, 0, 0, 0 }, centre_match{ 0, 0, 0, 0 };
    pcl::compute3DCentroid(*corres_targ, base_indices, centre_base);
    pcl::compute3DCentroid(*corres_src, match_indices, centre_match);

    PointTarget centre_pt_base;
    centre_pt_base.x = centre_base[0];
    centre_pt_base.y = centre_base[1];
    centre_pt_base.z = centre_base[2];

    PointSource centre_pt_match;
    centre_pt_match.x = centre_match[0];
    centre_pt_match.y = centre_match[1];
    centre_pt_match.z = centre_match[2];

    // find corresponding points according to their distance to the centroid
    pcl::Indices copy = match_indices;

    auto it_match_orig = match_indices.begin();
    for (auto it_base = base_indices.cbegin(), it_base_e = base_indices.cend();
        it_base != it_base_e;
        it_base++, it_match_orig++) {
        float dist_sqr_1 =
            pcl::squaredEuclideanDistance((*corres_targ)[*it_base], centre_pt_base);
        float best_diff_sqr = std::numeric_limits<float>::max();
        int best_index = -1;

        for (const auto& match_index : copy) {
            // calculate difference of distances to centre point
            float dist_sqr_2 =
                pcl::squaredEuclideanDistance((*corres_src)[match_index], centre_pt_match);
            float diff_sqr = std::abs(dist_sqr_1 - dist_sqr_2);

            if (diff_sqr < best_diff_sqr) {
                best_diff_sqr = diff_sqr;
                best_index = match_index;
            }
        }

        // assign new correspondence and update indices of matched targets
        correspondences.push_back(pcl::Correspondence(best_index, *it_base, best_diff_sqr));
        *it_match_orig = best_index;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
validateMatch(const pcl::Indices& base_indices,
    const pcl::Indices& match_indices,
    const pcl::Correspondences& correspondences,
    Eigen::Matrix4f& transformation)
{
    // only use triplet of points to simplify process (possible due to planar case)
    pcl::Correspondences correspondences_temp = correspondences;
    correspondences_temp.erase(correspondences_temp.end() - 1);

    // estimate transformation between correspondence set
    transformation_estimation_->estimateRigidTransformation(
        *input_, *target_, correspondences_temp, transformation);

    // transform base points
    PointCloudSource match_transformed;
    pcl::transformPointCloud(*input_, match_indices, match_transformed, transformation);

    // calculate residuals of transformation and check against maximum threshold
    std::size_t nr_points = correspondences_temp.size();
    float mse = 0.f;
    for (std::size_t i = 0; i < nr_points; i++)
        mse += pcl::squaredEuclideanDistance(match_transformed.points[i],
            corres_targ->points[base_indices[i]]);

    mse /= nr_points;
    return (mse < max_mse_ ? 0 : -1);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
int
pcl_utils::M4PCS<FeatureType>::
validateTransformation(Eigen::Matrix4f& transformation, float& fitness_score)
{
    // transform source point cloud
    PointCloudSource source_transformed;
    pcl::transformPointCloud(
        *input_, *source_indices_, source_transformed, transformation);

    std::size_t nr_points = source_transformed.size();
    std::size_t terminate_value =
        fitness_score > 1 ? 0
        : static_cast<std::size_t>((1.f - fitness_score) * nr_points);

    float inlier_score_temp = 0;
    pcl::Indices ids;
    std::vector<float> dists_sqr;
    auto it = source_transformed.begin();

    for (std::size_t i = 0; i < nr_points; it++, i++) {
        // search for nearest point using kd tree search
        tree_->nearestKSearch(*it, 1, ids, dists_sqr);
        inlier_score_temp += (dists_sqr[0] < max_inlier_dist_sqr_ ? 1 : 0);

        // early terminating
        if (nr_points - i + inlier_score_temp < terminate_value)
            break;
    }

    // check current costs and return unsuccessful if larger than previous ones
    inlier_score_temp /= static_cast<float>(nr_points);
    float fitness_score_temp = 1.f - inlier_score_temp;

    if (fitness_score_temp > fitness_score)
        return (-1);

    fitness_score = fitness_score_temp;
    return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename FeatureType>
void
pcl_utils::M4PCS<FeatureType>::
finalCompute(const std::vector<MatchingCandidates>& candidates)
{
    // get best fitness_score over all tries
    int nr_candidates = static_cast<int>(candidates.size());
    int best_index = -1;
    float best_score = std::numeric_limits<float>::max();
    for (int i = 0; i < nr_candidates; i++) {
        const float& fitness_score = candidates[i][0].fitness_score;
        if (fitness_score < best_score) {
            best_score = fitness_score;
            best_index = i;
        }
    }

    // check if a valid candidate was available
    if (!(best_index < 0)) {
        fitness_score_ = candidates[best_index][0].fitness_score;
        final_transformation_ = candidates[best_index][0].transformation;
        *correspondences_ = candidates[best_index][0].correspondences;

        // here we define convergence if resulting fitness_score is below 1-threshold
        converged_ = fitness_score_ < score_threshold_;
    }
}

template<typename FeatureType>
inline void pcl_utils::M4PCS<FeatureType>::extractCorrespondentIdx(std::vector<pcl::index_t>& src_idx, std::vector<pcl::index_t>& targ_idx)
{
	/*pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> Estimator;
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	Estimator.setInputSource(this->input_);
	Estimator.setInputTarget(this->target_);
	Estimator.setSearchMethodSource(pcl::search::KdTree<FeatureType>::Ptr(new pcl::search::KdTree<FeatureType>));
	Estimator.setSearchMethodTarget(pcl::search::KdTree<FeatureType>::Ptr(new pcl::search::KdTree<Point>));
	Estimator.determineReciprocalCorrespondences(*correspondences);
	size_t CoorespondenceSize = correspondences->size();
	for (int i = 0; i < CoorespondenceSize; i++) {
		if (correspondences->operator[](i).index_match == -1) continue;
		corres_src_idx.push_back(correspondences->operator[](i).index_query);
		corres_targ_idx.push_back(correspondences->operator[](i).index_match);
	}*/
}

template<typename FeatureType>
inline std::pair<pcl_utils::PointCloudXYZ::Ptr, pcl_utils::PointCloudXYZ::Ptr> 
pcl_utils::M4PCS<FeatureType>::extractCorrespondentPoint(
	std::vector<pcl::index_t>& src_base_idx, 
	std::vector<pcl::index_t>& targ_base_idx) {

	pcl_utils::PointCloudXYZ::Ptr Src_base = pcl_utils::filter::slides(src, src_base_idx);
	pcl_utils::PointCloudXYZ::Ptr Targ_base = pcl_utils::filter::slides(targ, targ_base_idx);
	return std::make_pair(Src_base, Targ_base);
}

template<typename FeatureType>
inline Eigen::Matrix3f pcl_utils::M4PCS<FeatureType>::PCA(pcl_utils::PointCloudXYZ::Ptr pointCloud) {
	// 主成分分析
	// pointCloud: 点云
	// return: 主成分矩阵
	Eigen::Matrix3f covarianceMatrix;
	Eigen::Vector4f mean;
	pcl::computeMeanAndCovarianceMatrix(*pointCloud, covarianceMatrix, mean);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covarianceMatrix, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	return eigenVectorsPCA;
}

template<typename FeatureType>
inline void pcl_utils::M4PCS<FeatureType>::sortByPCA_Axis(
	pcl_utils::PointCloudXYZ::Ptr pointCloud, 
	Eigen::Matrix3f const& princepleMatrix) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	projectedCloud->points.reserve(pointCloud->points.size());

	// 投影后, 主要成分为x轴, 次要成分为y轴, 次次要成分为z轴
	std::vector<size_t> indices(pointCloud->points.size());
	std::iota(indices.begin(), indices.end(), 0);

	for (const auto& point : pointCloud->points) {
		Eigen::Vector3f p(point.x, point.y, point.z);
		Eigen::Vector3f projected = princepleMatrix.transpose() * p;
		projectedCloud->points.push_back(pcl::PointXYZ(projected(0), projected(1), projected(2)));
	}

	// 根据projectedCloud中的点对indices进行排序
	std::sort(indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
		const auto& p1 = projectedCloud->points[i1];
		const auto& p2 = projectedCloud->points[i2];
		return std::tie(p1.x, p1.y, p1.z) < std::tie(p2.x, p2.y, p2.z);
		});

	// 根据indices对pointCloud进行排序
	pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	sortedCloud->points.reserve(pointCloud->points.size());
	for (size_t i : indices) {
		sortedCloud->points.push_back(pointCloud->points[i]);
	}

	// 将排序后的点云赋值给原点云
	*pointCloud = *sortedCloud;
}



#endif // !__4PCS_H_
