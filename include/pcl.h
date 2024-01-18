#pragma once

#ifndef __PCL_H__
#define __PCL_H__

#include <array>
#include <atomic>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/vfh.h>  
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_types.h>
#include <pcl/features/3dsc.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//�ع�
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/ModelCoefficients.h>       
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>   
#include <pcl/surface/mls.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d.h> // fatal error: pcl/surface/on_nurbs/fitting_curve_2d.h: û���Ǹ��ļ���Ŀ¼
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_pdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_sdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_tdm.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h> 
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/timeb.h>
#include <thread>
#include <time.h>
#include <tuple>
#include <utility>
#include <vector>

#include <pcl/surface/on_nurbs/fitting_curve_2d_apdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_atdm.h>

#include <pcl/console/parse.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>

#include <pcl/console/parse.h>

#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>

#include <pcl/registration/icp.h>              //ICP�����ͷ�ļ�
#include <pcl/registration/icp_nl.h>           //������ICP ���ͷ�ļ�
#include <pcl/registration/transforms.h>     //�任������ͷ�ļ�
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>

#include <pcl/filters/approximate_voxel_grid.h>// �˲��ļ�ͷ
#include <pcl/registration/ndt.h>       // ndt��׼�ļ�ͷ

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/correspondence.h>//�����㷨 ��Ӧ��ʾ����ʵ��֮���ƥ�䣨���磬�㣬�������ȣ���
// ����
#include <pcl/features/normal_3d_omp.h>//����������
#include <pcl/features/shot_omp.h> //������ shot������ 0��1
// (Signature of Histograms of OrienTations)����ֱ��ͼ����
#include <pcl/features/board.h>
// ʶ��
#include <pcl/recognition/cg/hough_3d.h>//hough����
#include <pcl/recognition/cg/geometric_consistency.h> //����һ����

#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/ml/dt/decision_forest_trainer.h>
#include <pcl/ml/svm.h>


#endif