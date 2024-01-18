#include "pcl_utility.h"
#include "pcl_utility_template.hpp"


using namespace pcl_utils;

void io::loadTXTFile(string const& filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        vector<string> tokens{ istream_iterator<string>{iss}, istream_iterator<string>{} };
        cloud.push_back(pcl::PointXYZ(stod(tokens[0]), stod(tokens[1]), stod(tokens[2])));
    }
}

void io::loadTXTFile(string const& filename, pcl::PointCloud<pcl::PointNormal>& cloud) {
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        vector<string> tokens{ istream_iterator<string>{iss}, istream_iterator<string>{} };
        cloud.push_back(pcl::PointNormal(stod(tokens[0]), stod(tokens[1]), stod(tokens[2]), stod(tokens[3]), stod(tokens[4]), stod(tokens[5])));
    }
}

PointCloudXYZ::Ptr io::open(const string& pointCloud_file) {
    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    string extension = pointCloud_file.substr(pointCloud_file.find_last_of('.') + 1);
    if (extension == "pcd")
        pcl::io::loadPCDFile<pcl::PointXYZ>(pointCloud_file, *cloud);
    else if (extension == "ply")
        pcl::io::loadPLYFile<pcl::PointXYZ>(pointCloud_file, *cloud);
    else if ( extension == "txt") 
        pcl_utils::io::loadTXTFile(pointCloud_file, *cloud);
    else {
        std::runtime_error("Unsupported file format");
    }
    return cloud;
}


void io::PointCloud2TXT(PointCloudXYZ::Ptr pointCloud, string const& out_dir) {
    ofstream outfile(out_dir);
    for (size_t i = 0; i < pointCloud->points.size(); ++i) {
        outfile << pointCloud->points[i].x << "\t\t" << pointCloud->points[i].y << "\t\t" << pointCloud->points[i].z << endl;
    }
    outfile.close();
}

void io::PointCloud2RangeImage(PointCloudXYZ::Ptr pointCloud,
    Eigen::Translation3f sensorPose,
    pcl::RangeImage::CoordinateFrame coordinate_frame,
    std::string const& save_dir
) {
    float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //  1角度转弧度
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360角度转弧度
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180角度转弧度
    // 深度图所使用的坐标系
    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;
    Eigen::Affine3f sensor_Pose = (Eigen::Affine3f)sensorPose;
    pcl::RangeImage rangeImage; // 深度图对象
    rangeImage.createFromPointCloud(*pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
        sensor_Pose, coordinate_frame, noiseLevel, minRange, borderSize);
    float* ranges = rangeImage.getRangesArray();
    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);
    pcl::io::saveRgbPNGFile(save_dir, rgb_image, rangeImage.width, rangeImage.height);
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(rangeImage);
    while (!range_image_widget.wasStopped());
    system("pause");
}

void io::save(const string& filename, PointCloudXYZ::Ptr cloud) {
    string extension = filename.substr(filename.find_last_of('.') + 1);
    if (extension == "pcd")
        pcl::io::savePCDFileASCII(filename, *cloud);
    else if (extension == "ply")
        pcl::io::savePLYFileBinary(filename, *cloud);
    else {
        std::runtime_error("Unsupported file format");
    }
}

void io::save(vector<string> const& filename, vector<PointCloudXYZ::Ptr> clouds, string const& root_dir) {
    assert(filename.size() == clouds.size());
    for (size_t i = 0; i < filename.size(); ++i) {
        save(root_dir + filename[i], clouds[i]);
    }
}

void vis::Visualize(pcl::PolygonMesh mesh) {
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.addPolygonMesh(mesh, "my");
    viewer.addCoordinateSystem(50.0);
    viewer.initCameraParameters();
    viewer.spin();
}

void vis::
Visualize(ON_NurbsCurve& curve, ON_NurbsSurface& surface, pcl::visualization::PCLVisualizer& viewer) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
    for (std::size_t i = 0; i < curve_cloud->size() - 1; i++)
    {
        pcl::PointXYZRGB& p1 = curve_cloud->at(i);
        pcl::PointXYZRGB& p2 = curve_cloud->at(i + 1);
        std::ostringstream os;
        os << "line" << i;
        viewer.removeShape(os.str());
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < curve.CVCount(); i++)
    {
        ON_3dPoint p1;
        curve.GetCV(i, p1);

        double pnt[3];
        surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
        pcl::PointXYZRGB p2;
        p2.x = float(pnt[0]);
        p2.y = float(pnt[1]);
        p2.z = float(pnt[2]);

        p2.r = 255;
        p2.g = 0;
        p2.b = 0;

        curve_cps->push_back(p2);
    }
    viewer.removePointCloud("cloud_cps");
    viewer.addPointCloud(curve_cps, "cloud_cps");
}

search::OctTree_wrapper::OctTree_wrapper(PointCloudXYZ::Ptr cloud, float resolution) :
    octree(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution)) {
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
}

tuple<vector<int>, vector<float>>
search::kdTree_wrapper::kNNSearch(pcl::PointXYZ const& point, const unsigned int K, float radius){
    /**
    *   @brief K近邻搜索
    *   @param point 搜索点
    *   @param K K近邻
    *   @param radius 搜索半径
    *   @return tuple<vector<int>, vector<float>> 返回搜索到的点的索引和距离
    */
    assert(K > 0);
    vector<int> pointIdxKNNSearch(K);
    vector<float> pointKNNSquaredDistance(K);
    int res{};
    if (radius < 0) {
        res = kdtree.nearestKSearch(point, K, pointIdxKNNSearch, pointKNNSquaredDistance);
    }
    else {
        res = kdtree.radiusSearch(point, radius, pointIdxKNNSearch, pointKNNSquaredDistance);
    }
    assert(res > 0);
    return make_tuple(pointIdxKNNSearch, pointKNNSquaredDistance);
}

tuple<vector<int>, vector<float>>
search::OctTree_wrapper::kNNSearch(pcl::PointXYZ const& point, const unsigned int K, float radius) {

    assert(K > 0);
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    int res{};
    if (radius < 0) {
        res = octree.nearestKSearch(point, K, pointIdxKNNSearch, pointKNNSquaredDistance);
    }
    else {
        res = octree.radiusSearch(point, radius, pointIdxKNNSearch, pointKNNSquaredDistance);
    }
    assert(res > 0);
    return make_tuple(pointIdxKNNSearch, pointKNNSquaredDistance);

}

std::vector<int> search::OctTree_wrapper::voxelSearch(pcl::PointXYZ const& point) {
    vector<int> pointIdxVec;
    int res = octree.voxelSearch(point, pointIdxVec);
    assert(res > 0);
    return pointIdxVec;
}

pcl::PCLPointCloud2::Ptr
filter::Filtering(pcl::PCLPointCloud2::Ptr cloud, 
                  pcl::VoxelGrid<pcl::PCLPointCloud2>& filter,
                  Eigen::Vector3f const& voxSize) {
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    filter.setInputCloud(cloud);
    filter.setLeafSize(voxSize[0], voxSize[1], voxSize[2]);
    filter.filter(*cloud_filtered);
    return cloud_filtered;
}

PointCloudXYZ::Ptr
filter::Filtering(
    PointCloudXYZ::Ptr cloud, 
    pcl::VoxelGrid<pcl::PCLPointCloud2>& filter,
    Eigen::Vector3f const& voxSize
) {
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud, *cloud_blob);
    auto cloud_blob_filtered = Filtering(cloud_blob, filter, voxSize);
    PointCloudXYZ::Ptr outputCloud(new PointCloudXYZ);
    pcl::fromPCLPointCloud2(*cloud_blob, *outputCloud);
    return outputCloud;
}

PointCloudXYZ::Ptr
filter::Filtering(
    PointCloudXYZ::Ptr const cloud, 
    pcl::PassThrough<pcl::PointXYZ>& filter,
    string const& field, float dlimit, float ulimit
) {
    PointCloudXYZ::Ptr outputCloud(new PointCloudXYZ);
    filter.setInputCloud(cloud);
    filter.setFilterFieldName(field);
    filter.setFilterLimits(dlimit, ulimit);
    filter.filter(*outputCloud);
    return outputCloud;
}

PointCloudXYZ::Ptr
filter::Filtering(
    PointCloudXYZ::Ptr const cloud,
    pcl::PassThrough<pcl::PointXYZ>& filter,
    string const& field,
    Eigen::Vector2f const& limit
) {
    return Filtering(cloud, filter, field, limit[0], limit[1]);
}

PointCloudXYZ::Ptr
filter::Filtering(
    PointCloudXYZ::Ptr cloud,
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>& filter,
    unsigned int const K, float threshold
) {
    PointCloudXYZ::Ptr outputCloud(new PointCloudXYZ);
    assert(K > 0);
    filter.setInputCloud(cloud);
    filter.setMeanK(K);
    filter.setStddevMulThresh(threshold);
    filter.filter(*outputCloud);
    return outputCloud;
}

pcl::PCLPointCloud2::Ptr
filter::VoxelFilt(pcl::PCLPointCloud2::Ptr const cloud, Eigen::Vector3f const& voxSize) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    return Filtering(cloud, sor, voxSize);
}

pcl::PCLPointCloud2::Ptr
filter::VoxelFilt(pcl::PCLPointCloud2::Ptr const cloud, float x, float y, float z) {
    assert(x > 0 && y > 0 && z > 0);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    return Filtering(cloud, sor, { x, y, z });
}

pcl::PCLPointCloud2::Ptr
filter::VoxelFilt(pcl::PCLPointCloud2::Ptr const cloud, float x) {
    assert(x > 0);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    return Filtering(cloud, sor, { x, x, x });
}

PointCloudXYZ::Ptr
filter::VoxelFilt(PointCloudXYZ::Ptr const cloud, Eigen::Vector3f const& voxSize) {
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud, *cloud_blob);
    auto cloud_blob_filtered = VoxelFilt(cloud_blob, voxSize);
    PointCloudXYZ::Ptr outputCloud(new PointCloudXYZ);
    pcl::fromPCLPointCloud2(*cloud_blob, *outputCloud);
    return outputCloud;
}

PointCloudXYZ::Ptr
filter::VoxelFilt(PointCloudXYZ::Ptr const cloud, float x, float y, float z) {
    assert(x > 0 && y > 0 && z > 0);
    return VoxelFilt(cloud, { x, y, z });
}

PointCloudXYZ::Ptr
filter::VoxelFilt(PointCloudXYZ::Ptr const cloud, float x) {
    if (x < 0) {
        cout << "x < 0, set x > 0" << endl;
        exit(1);
    }
    return VoxelFilt(cloud, { x, x, x });
}

PointCloudXYZ::Ptr
filter::Passthrough(PointCloudXYZ::Ptr const cloud, string const& field, float dlimit, float ulimit) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    return Filtering(cloud, pass, field, dlimit, ulimit);
}

PointCloudXYZ::Ptr
filter::Passthrough(PointCloudXYZ::Ptr const cloud, string const& field, Eigen::Vector2f const& limit) {
    return Passthrough(cloud, field, limit[0], limit[1]);
}

PointCloudXYZ::Ptr
filter::StasticRemove(PointCloudXYZ::Ptr cloud, unsigned int const K, float threshold) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    return Filtering(cloud, sor, K, threshold);
}

PointCloudXYZ::Ptr
trans::TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Isometry3d const& T) {
    pcl::transformPointCloud(*input_cloud, *output_cloud, T.matrix());
    return output_cloud;
}

PointCloudXYZ::Ptr
trans::TransForm(
    PointCloudXYZ::Ptr input_cloud,
    PointCloudXYZ::Ptr output_cloud,
    Eigen::Vector3d const& s,
    Eigen::AngleAxisd rotation_vector) {
    Eigen::Isometry3d T;
    T.rotate(rotation_vector);
    T.pretranslate(s);
    return TransForm(input_cloud, output_cloud, T);
}

PointCloudXYZ::Ptr
trans::TransForm(
    PointCloudXYZ::Ptr input_cloud, 
    PointCloudXYZ::Ptr output_cloud, 
    Eigen::Vector3d const& s, 
    double alpha, 
    Eigen::Vector3d const& Axis) {
    return TransForm(input_cloud, output_cloud, s, Eigen::AngleAxisd(alpha, Axis));
}

PointCloudXYZ::Ptr
trans::TransForm(
    PointCloudXYZ::Ptr input_cloud, 
    PointCloudXYZ::Ptr output_cloud, 
    Eigen::Vector3d const& s, 
    double alpha, double x, double y, double z) {
    return TransForm(input_cloud, output_cloud, s, Eigen::AngleAxisd(alpha, Eigen::Vector3d(x, y, z)));
}

PointCloudXYZ::Ptr
trans::TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s, Eigen::Vector3d const& EulerAngle) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << s(0), s(1), s(2);
    transform.rotate(Eigen::AngleAxisd(EulerAngle(0), Eigen::Vector3d::UnitX()));
    transform.rotate(Eigen::AngleAxisd(EulerAngle(1), Eigen::Vector3d::UnitY()));
    transform.rotate(Eigen::AngleAxisd(EulerAngle(2), Eigen::Vector3d::UnitZ()));
    pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
    return output_cloud;
}

PointCloudXYZ::Ptr
trans::TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s, double Ax, double Ay, double Az) {
    return TransForm(input_cloud, output_cloud, s, Eigen::Vector3d(Ax, Ay, Az));
}

PointCloudXYZ::Ptr
trans::Translation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& s) {
    // 对input_cloud进行平移
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << s(0), s(1), s(2);
    pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
    return output_cloud;
}

PointCloudXYZ::Ptr
trans::Translation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, double x, double y, double z) {
    return Translation(input_cloud, output_cloud, Eigen::Vector3d(x, y, z));
}

PointCloudXYZ::Ptr
trans::Translation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, double s) {
    return Translation(input_cloud, output_cloud, Eigen::Vector3d(s, s, s));
}

PointCloudXYZ::Ptr
trans::Rotation(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Vector3d const& EulerAngle) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(Eigen::AngleAxisd(EulerAngle(0), Eigen::Vector3d::UnitX()));
    transform.rotate(Eigen::AngleAxisd(EulerAngle(1), Eigen::Vector3d::UnitY()));
    transform.rotate(Eigen::AngleAxisd(EulerAngle(2), Eigen::Vector3d::UnitZ()));
    pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
    return output_cloud;
}

Eigen::Vector3d trans::Quad2EulerA(Eigen::Quaterniond const& quaternion) {
    auto rotationMat = quaternion.toRotationMatrix();
    return rotationMat.eulerAngles(2, 1, 0);
}

Eigen::Quaterniond trans::EulerA2Quad(Eigen::Vector3d EulerAngle) {
    return Eigen::AngleAxisd(EulerAngle[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(EulerAngle[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(EulerAngle[2], Eigen::Vector3d::UnitX());
}

PointCloudXYZ::Ptr
trans::TransForm(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, Eigen::Quaterniond const& quaternion) {
    return Rotation(input_cloud, output_cloud, Quad2EulerA(quaternion));
}

PointCloudXYZ::Ptr
filter::slides(PointCloudXYZ::Ptr cloud, vector<int> const& inliers) {
    decltype(cloud) res(new PointCloudXYZ());
    pcl::copyPointCloud(*cloud, inliers, *res);
    return res;
}

PointCloudXYZ::Ptr
filter::projPlane(PointCloudXYZ::Ptr input_cloud, PointCloudXYZ::Ptr output_cloud, double a, double b, double c, double d) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = a;
    coefficients->values[1] = b;
    coefficients->values[2] = c;
    coefficients->values[3] = d;
    return Projection<pcl::SACMODEL_PLANE>(input_cloud, output_cloud, coefficients);
}

PointCloudXYZ::Ptr
filter::projPlane(PointCloudXYZ::Ptr input_cloud, double a, double b, double c, double d) {
    PointCloudXYZ::Ptr output(new PointCloudXYZ);
    return projPlane(input_cloud, output, a, b, c, d);
}

PointCloudXYZI::Ptr
filter::bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input, pcl::PointCloud<pcl::PointXYZI>::Ptr output, double std, double HalfSize) {
    // 双边滤波
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    // Apply the filter  
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(input);
    fbf.setSearchMethod(tree1);
    fbf.setStdDev(std);
    fbf.setHalfSize(HalfSize);
    fbf.filter(*output);
    return output;
}

PointCloudXYZI::Ptr
filter::bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input, double std, double HalfSize) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    return bilateralFilter(input, output, std, HalfSize);
}

PointCloudXYZ::Ptr
filter::RadiusOutlierRemoval(PointCloudXYZ::Ptr input, PointCloudXYZ::Ptr output, double radius, unsigned int K) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(input);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(K);
    outrem.filter(*output);
    return output;
}


PointCloudXYZ::Ptr
filter::RadiusOutlierRemoval(PointCloudXYZ::Ptr input, double radius, unsigned int K) {
    PointCloudXYZ::Ptr output(new PointCloudXYZ());
    return RadiusOutlierRemoval(input, output, radius, K);
}


void
feature::split(pcl::PointCloud<pcl::PointNormal>::Ptr input, PointCloudXYZ::Ptr outputPoint, PointCloudNormal::Ptr outputNormal) {
    if (outputPoint.get() == nullptr || outputNormal.get() == nullptr) {
        cerr << "OutPutCloud is nullptr" << endl;
        exit(1);
    }
    for (auto const& point : input->points) {
        outputPoint->push_back(pcl::PointXYZ(point.x, point.y, point.z));
        outputNormal->push_back(pcl::Normal(point.normal_x, point.normal_y, point.normal_z));
    }
}

pcl::PointCloud<pcl::Normal>::Ptr
feature::normal::Normal_Estimation(PointCloudXYZ::Ptr cloud, double radius) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius);
    ne.compute(*cloud_normals);
    // cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同的尺寸
    assert(cloud_normals->points.size() == cloud->points.size());
    return cloud_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
feature::normal::MLSNormalEstimation(PointCloudXYZ::Ptr cloud, double radius, int order) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);

    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(order);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.process(*normals);
    return normals;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr
feature::normal::PFHEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius) {
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
    pfh.setRadiusSearch(radius);
    pfh.compute(*pfhs);
    return pfhs;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr
feature::normal::PFHEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2) {
    assert(radius1 < radius2);
    auto normals = Normal_Estimation(cloud, radius1);
    return PFHEstimation(cloud, normals, radius2);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
feature::normal::FPFHEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius) {
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.setRadiusSearch(radius);
    fpfh.compute(*fpfhs);
    return fpfhs;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
feature::normal::FPFHEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2) {
    assert(radius1 < radius2);
    auto normals = Normal_Estimation(cloud, radius1);
    return FPFHEstimation(cloud, normals, radius2);
}


pcl::PointCloud<pcl::VFHSignature308>::Ptr
feature::VFHEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius) {
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    vfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
    vfh.setRadiusSearch(radius);
    vfh.compute(*vfhs);
    return vfhs;
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr
feature::VFHEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2) {
    assert(radius1 < radius2);
    auto normals = normal::Normal_Estimation(cloud, radius1);
    return VFHEstimation(cloud, normals, radius2);
}

pcl::PointCloud<int>::Ptr
feature::NarfKeyPoint(pcl::RangeImage const& range_image, float support_size) {
    pcl::RangeImageBorderExtractor range_image_border_extractor;    //申明深度图边缘提取器
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;    //获得特征提取的大小
    pcl::PointCloud<int>::Ptr keypoint_indices(new pcl::PointCloud<int>());
    narf_keypoint_detector.compute(*keypoint_indices);
    return keypoint_indices;
}

pcl::PointCloud<pcl::SHOT352>::Ptr pcl_utils::feature::SHOTEstimation(PointCloudXYZ::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius) {
    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloud);
	shot.setInputNormals(normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	shot.setSearchMethod(tree);
	pcl::PointCloud<pcl::SHOT352>::Ptr shots(new pcl::PointCloud<pcl::SHOT352>());
	shot.setRadiusSearch(radius);
	shot.compute(*shots);
	return shots;

}

pcl::PointCloud<pcl::SHOT352>::Ptr pcl_utils::feature::SHOTEstimation(PointCloudXYZ::Ptr cloud, double radius1, double radius2) {
    pcl::PointCloud<pcl::Normal>::Ptr normals = normal::Normal_Estimation<AVERAGE_3D_GRADIENT>(cloud, radius1, 10);
    return SHOTEstimation(cloud, normals, radius2);
}

PointCloudXYZ::Ptr
keypoints::SiftKeyPoints(PointCloudXYZ::Ptr srcCloud, float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast) {
    // convert the input point cloud to PointXYZI format
    pcl::PointCloud<pcl::PointXYZI>::Ptr srcCloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
    if (srcCloud->size() == 0) {
        cerr << "srcCloud is empty" << endl;
        exit(1);
    }
    // pcl::copyPointCloud(*srcCloud, *srcCloudXYZI);
    for (auto const& point : srcCloud->points) {
        srcCloudXYZI->push_back(pcl::PointXYZI(point.x, point.y, point.z));
    }
    pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
    sift.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>()));
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    sift.setInputCloud(srcCloudXYZI);
    sift.compute(*keypoints);
    PointCloudXYZ::Ptr result(new PointCloudXYZ);
    // pcl::copyPointCloud(*keypoints, *result);
    for (auto const& point : keypoints->points) {
        result->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    return result;
}

PointCloudXYZ::Ptr
segment::plannar_segmentation(PointCloudXYZ::Ptr cloud, double thresh) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    return planar_segmentation<pcl::SACMODEL_PLANE, pcl::SAC_RANSAC>(cloud, coefficients, inliers, thresh);
}

PointCloudXYZ::Ptr
surface::ConCaveHull_2d(PointCloudXYZ::Ptr cloud, double alpha) {
    PointCloudXYZ::Ptr output(new PointCloudXYZ);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.setAlpha(alpha);
    chull.reconstruct(*output);
    return output;
}

PointCloudXYZ::Ptr
surface::ConvexHull_2d(PointCloudXYZ::Ptr cloud) {
    PointCloudXYZ::Ptr output(new PointCloudXYZ);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.setDimension(2);
    chull.reconstruct(*output);
    return output;
}

pcl::PolygonMesh::Ptr
surface::greedy_projection(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, double radius) {
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);   //点云构建搜索树

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);                //存储最终三角化的网络模型

    gp3.setSearchRadius(radius);               //设置连接点之间的最大距离，（即是三角形最大边长）
    gp3.setMu(2.5);                             //设置最近邻距离的乘子，（用于设置三角形时球的半径）
    gp3.setMaximumNearestNeighbors(100);        //设置搜索的最近邻点的最大数量
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45
    gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10
    gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120
    gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致

    // Get result
    gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
    gp3.setSearchMethod(tree2);   //设置搜索方式
    gp3.reconstruct(*triangles);  //重建提取三角化

    return triangles;
}


pcl::PolygonMesh::Ptr
surface::greedy_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double radius) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    return greedy_projection(cloud_with_normals, radius);
}

void
surface::PointCloud2Vector3d(PointCloudXYZ::Ptr cloud, pcl::on_nurbs::vector_vec3d& data) {
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        auto& p = cloud->at(i);
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }
}

void
surface::bspline_fitting(PointCloudXYZ::Ptr cloud) {
    pcl::visualization::PCLVisualizer viewer("B-spline surface fitting");
    viewer.setSize(800, 600);
    pcl::PCLPointCloud2 cloud2;
    pcl::on_nurbs::NurbsDataSurface data;
    fromPCLPointCloud2(cloud2, *cloud);
    PointCloud2Vector3d(cloud, data.interior);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, handler, "cloud_cylinder");
    // parameters
    unsigned order(3);
    unsigned refinement(5);
    unsigned iterations(10);
    unsigned mesh_resolution(256);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);

    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);

    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fit.refine(0);
        fit.refine(1);
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce();
    }

    // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble(params);
        fit.solve();
        pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
        viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        viewer.spinOnce();
    }

    // parameters
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 200;
    curve_params.accuracy = 1e-3;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    pcl::on_nurbs::NurbsDataCurve2d curve_data;

    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back(true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);

    // curve fitting
    pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); // enable/disable debug output
    curve_fit.fitting(curve_params);
    vis::Visualize(curve_fit.m_nurbs, fit.m_nurbs, viewer);
    viewer.removePolygonMesh(mesh_id);
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
        mesh_resolution);
    viewer.addPolygonMesh(mesh, mesh_id);
    viewer.spin();
}

void
surface::PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec2d& data) {
    for (unsigned i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZ& p = cloud->at(i);
        if (!isnan(p.x) && !isnan(p.y))
            data.push_back(Eigen::Vector2d(p.x, p.y));
    }
}

void
surface::VisualizeCurve(ON_NurbsCurve& curve, double r, double g, double b, bool show_cps, pcl::visualization::PCLVisualizer viewer) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, cloud, 8);

    for (std::size_t i = 0; i < cloud->size() - 1; i++)
    {
        pcl::PointXYZRGB& p1 = cloud->at(i);
        pcl::PointXYZRGB& p2 = cloud->at(i + 1);
        std::ostringstream os;
        os << "line_" << r << "_" << g << "_" << b << "_" << i;
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, r, g, b, os.str());

    }

    if (show_cps)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cps(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < curve.CVCount(); i++)
        {
            ON_3dPoint cp;
            curve.GetCV(i, cp);

            pcl::PointXYZ p;
            p.x = float(cp.x);
            p.y = float(cp.y);
            p.z = float(cp.z);
            cps->push_back(p);
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cps, 255 * r, 255 * g, 255 * b);
        viewer.addPointCloud<pcl::PointXYZ>(cps, handler, "cloud_cps");

    }
}

void
surface::printUsage(const char* progName) {
    std::cout << "\n\nUsage: " << progName << " [options]\n\n"
        << "Options:\n"
        << "-------------------------------------------\n"
        << "-h           this help\n"
        << "-pd          point distance minimization\n"
        << "-td          tangent distance minimization\n"
        << "-sd          squared distance minimization\n"
        << "-apd         asymmetric point-distance-minimization\n"
        << "-asd         asymmetric squared-distance-minimization\n"
        << "\n\n";
}

void surface::fitting_curve_2d(int argc, char* argv[]) {
    pcl::visualization::PCLVisualizer viewer("Curve Fitting 2D");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(argv[1], *cloud);
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return;
    }

    bool pd(false), td(false), sd(false), apd(false),
        atd(false), asd(false);
    if (pcl::console::find_argument(argc, argv, "-pd") >= 0)
    {
        pd = true;
        std::cout << "pdm\n";
    }
    else if (pcl::console::find_argument(argc, argv, "-sd") >= 0)
    {
        sd = true;
        std::cout << "sdm\n";
    }
    else if (pcl::console::find_argument(argc, argv, "-apd") >= 0)
    {
        apd = true;
        std::cout << "apdm\n";
    }
    else if (pcl::console::find_argument(argc, argv, "-td") >= 0)
    {
        atd = true;
        std::cout << "tdm\n";
    }
    else if (pcl::console::find_argument(argc, argv, "-asd") >= 0)
    {
        asd = true;
        std::cout << "asdm\n";
    }
    else
    {
        printUsage(argv[0]);
        return;
    }

    // 空间交换：输入原始点为yoz平面的2D点云，需要转换为xoy平面上的2D点云
    pcl::PointCloud<pcl::PointXYZ>::iterator it_1;
    for (it_1 = cloud->begin(); it_1 != cloud->end();) {
        float x = it_1->x;
        float y = it_1->y;
        float z = it_1->z;

        it_1->x = z;
        it_1->y = y;
        it_1->z = x;

        ++it_1;
    }

    // 数据类型转换：convert to NURBS data structure
    pcl::on_nurbs::NurbsDataCurve2d data;
    PointCloud2Vector2d(cloud, data.interior);

    // 构建初始曲线
    unsigned order(3);//曲线阶数
    unsigned n_control_points(10);//曲线控制点个数
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dSDM::initNurbsCurve2D(order, data.interior, n_control_points);

    // curve fitting
    if (pd) {
        pcl::on_nurbs::FittingCurve2dPDM::Parameter curve_params;
        curve_params.smoothness = 0.000001;
        curve_params.rScale = 1.0;

        pcl::on_nurbs::FittingCurve2dPDM fit(&data, curve);//初始化曲线拟合对象  
        fit.assemble(curve_params);
        fit.solve();
        VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false, viewer);
    }
    else if (td) {
        pcl::on_nurbs::FittingCurve2dTDM::Parameter curve_params;
        curve_params.smoothness = 0.000001;
        curve_params.rScale = 1.0;

        pcl::on_nurbs::FittingCurve2dTDM fit(&data, curve);//初始化曲线拟合对象  
        fit.assemble(curve_params);
        fit.solve();
        VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false, viewer);
    }
    else if (sd) {
        pcl::on_nurbs::FittingCurve2dSDM::Parameter curve_params;//初始化曲线拟合对象
        curve_params.smoothness = 0.000001;
        curve_params.rScale = 1.0;

        pcl::on_nurbs::FittingCurve2dSDM fit(&data, curve);
        fit.assemble(curve_params);
        fit.solve();
        VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false, viewer);
    }
    else if (apd) {
        pcl::on_nurbs::FittingCurve2dAPDM::Parameter curve_params;
        curve_params.smoothness = 0.000001;
        curve_params.rScale = 1.0;

        pcl::on_nurbs::FittingCurve2dAPDM fit(&data, curve);//初始化曲线拟合对象
        fit.assemble(curve_params);
        fit.solve();
        VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false, viewer);
    }
    else if (asd) {
        pcl::on_nurbs::FittingCurve2dASDM::Parameter curve_params;
        curve_params.smoothness = 0.000001;
        curve_params.rScale = 1.0;

        pcl::on_nurbs::FittingCurve2dASDM fit(&data, curve); //初始化曲线拟合对象 
        fit.assemble(curve_params);                          //装配曲线参数
        fit.solve();                                         //拟合
        VisualizeCurve(fit.m_nurbs, 1.0, 0.0, 0.0, false, viewer);   //可视化拟合曲线
    }

    // visualize
    viewer.setSize(800, 600);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud");
    viewer.spin();

}

pcl::PolygonMesh
surface::recon_poisson(PointCloudXYZ::Ptr srcCloud) {
    auto normals = feature::normal::Normal_Estimation(srcCloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr Cloud_with_Normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*srcCloud, *normals, *Cloud_with_Normal);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(Cloud_with_Normal);
    //创建Poisson对象，并设置参数
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    //pn.setIndices();

    //设置搜索方法和输入点云
    pn.setSearchMethod(tree2);
    pn.setInputCloud(Cloud_with_Normal);
    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;
    //执行重构
    pn.performReconstruction(mesh);
    return mesh;
}

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr
registration::ICP(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, Eigen::Matrix4f& trans, int max_iter) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
    PointCloudXYZ::Ptr output(new PointCloudXYZ);
    icp->setInputSource(source);
    icp->setInputTarget(target);
    icp->setMaximumIterations(max_iter);
    icp->setTransformationEpsilon(1e-8);
    icp->setEuclideanFitnessEpsilon(1);
    icp->align(*output);
    trans = icp->transformation_;
    return icp;
}

pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Ptr
registration::ICP(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, Eigen::Matrix4f& trans, double thresh, int max_iter) {
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Ptr icp(new pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr Final(new pcl::PointCloud<pcl::PointNormal>());
    auto srcNormal = feature::normal::MLSNormalEstimation(source, 0.03);
    auto tarNormal = feature::normal::MLSNormalEstimation(target, 0.03);
    icp->setInputSource(srcNormal);
    icp->setInputTarget(tarNormal);
    icp->setMaximumIterations(max_iter);
    icp->setTransformationEpsilon(thresh);
    icp->setEuclideanFitnessEpsilon(0.0001);
    icp->setSearchMethodSource(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
    icp->setSearchMethodTarget(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>()));
    icp->setRANSACOutlierRejectionThreshold(0.0001);
    icp->setTransformationEpsilon(1e-8);
    icp->setUseReciprocalCorrespondences(true);
    icp->align(*Final);
    return icp;
}

void registration::pairAlign(
    const PointCloudXYZ::Ptr cloud_src, 
    const PointCloudXYZ::Ptr cloud_tgt, 
    PointCloudXYZ::Ptr output, 
    Eigen::Matrix4f& final_transform,
    double voxsize, bool downsample, int k, double eps) {
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloudXYZ::Ptr src(new PointCloudXYZ);   //存储滤波后的源点云
    PointCloudXYZ::Ptr tgt(new PointCloudXYZ);   //存储滤波后的目标点云
    pcl::VoxelGrid<pcl::PointXYZ> grid;         /////滤波处理对象
    if (downsample) {
        grid.setLeafSize(voxsize, voxsize, voxsize);    //设置滤波时采用的体素大小
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else {
        src = cloud_src;
        tgt = cloud_tgt;
    }
    // 计算表面的法向量和曲率
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;     //点云法线估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(k);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    registration::MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
    point_representation.setRescaleValues(alpha);

    //
    // 配准
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;   // 配准对象
    reg.setTransformationEpsilon(eps);   ///设置收敛判断条件，越小精度越大，收敛也越慢 
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm大于此值的点对不考虑
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(0.1);
    // 设置点表示
    reg.setPointRepresentation(std::make_shared<const MyPointRepresentation>(point_representation));

    reg.setInputSource(points_with_normals_src);   // 设置源点云
    reg.setInputTarget(points_with_normals_tgt);    // 设置目标点云
    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);////设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
    for (int i = 0; i < 30; ++i)   ////手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示
    {
        PCL_INFO("Iteration Nr. %d.\n", i);
        // 存储点云以便可视化
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();

    }
    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();//deidao
    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
    //add the source to the transformed target
    *output += *cloud_src;
    final_transform = targetToSource;
}


PointCloudXYZ::Ptr
registration::pairwise_incremental_registration(vector<PointCloudXYZ::Ptr> data, Eigen::Matrix4f& GlobalTransform,
    double voxsize, bool downsample, int k, double eps) {
    PointCloudXYZ::Ptr result(new PointCloudXYZ), source, target;
    GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pairTransform;
    for (size_t i = 1; i < data.size(); ++i) {
        source = data[i - 1];
        target = data[i];
        PointCloudXYZ::Ptr temp(new PointCloudXYZ);
        registration::pairAlign(source, target, temp, pairTransform, voxsize, true, k, eps);
        pcl::transformPointCloud(*temp, *result, GlobalTransform);
        GlobalTransform *= pairTransform;
    }
    return result;
}

PointCloudXYZ::Ptr
registration::NDT(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, Eigen::Matrix4f& finalTrans,
    double eps, double stepSize, double resolution, int max_iter) {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(eps);
    ndt.setStepSize(stepSize);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iter);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    PointCloudXYZ::Ptr Final(new PointCloudXYZ);
    ndt.align(*Final);
    finalTrans = ndt.getFinalTransformation();
    return Final;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
registration::Alignment(pcl::PointCloud<pcl::PointNormal>::Ptr object, pcl::PointCloud<pcl::PointNormal>::Ptr scene) {
    pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr object_downsampled(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointNormal>), scene_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
        scene_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
        object_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    // Downsample
    // 为了加快处理速度，我们使用PCL的：pcl::VoxelGrid类将对象和场景点云的采样率下采样至5 mm。
    pcl::VoxelGrid<pcl::PointNormal> grid;
    const float leaf = 0.005f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object_downsampled);
    grid.setInputCloud(scene);
    grid.filter(*scene_downsampled);

    // Estimate normals for scene
    pcl::console::print_highlight("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(scene_downsampled);
    nest.compute(*scene_normals);

    // Estimate features
    // 对于下采样点云中的每个点，我们现在使用PCL的pcl::FPFHEstimationOMP<>类来计算用于对齐过程中用于匹配的快速点特征直方图（FPFH）描述符。
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object_downsampled);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
    align.setInputSource(object_downsampled); // 对象
    align.setSourceFeatures(object_features); // 对象特征
    align.setInputTarget(scene_normals);  // 场景
    align.setTargetFeatures(scene_features); //  场景特征
    align.setMaximumIterations(50000); // 采样一致性迭代次数 Number of RANSAC iterations
    align.setNumberOfSamples(3); // 在对象和场景之间进行采样的点对应数。 至少需要3个点才能计算姿势。Number of points to sample for generating/prerejecting a pose 
    align.setCorrespondenceRandomness(5); //  使用的临近特征点的数目 Number of nearest features to use
    align.setSimilarityThreshold(0.9f); // 多边形边长度相似度阈值 Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(2.5f * leaf); // 判断是否为内点的距离阈值 Inlier threshold
    align.setInlierFraction(0.25f); //  接受位姿假设所需的内点比例 Required inlier fraction for accepting a pose hypothesis
    align.align(*object_aligned); // 对齐的对象存储在点云object_aligned中。 
    return object_aligned;
}

PointCloudXYZ::Ptr
registration::FPFHRegistration(PointCloudXYZ::Ptr source_cloud, PointCloudXYZ::Ptr target_cloud) {
    PointCloudXYZ::Ptr output_Cloud(new PointCloudXYZ);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    PointCloudXYZ::Ptr source_cloud_downsampled(new PointCloudXYZ);
    PointCloudXYZ::Ptr target_cloud_downsampled(new PointCloudXYZ);
    voxel_grid.setInputCloud(source_cloud);
    voxel_grid.filter(*source_cloud_downsampled);
    voxel_grid.setInputCloud(target_cloud);
    voxel_grid.filter(*target_cloud_downsampled);

    // Compute FPFH descriptors for the keypoints
    auto source_descriptors = pcl_utils::feature::normal::FPFHEstimation(source_cloud_downsampled, 0.05, 0.05);
    auto target_descriptors = pcl_utils::feature::normal::FPFHEstimation(target_cloud_downsampled, 0.05, 0.05);
    return registration::SACRegistration<pcl::FPFHSignature33>(source_cloud_downsampled, target_cloud_downsampled,
        *source_descriptors, *target_descriptors);
}

PointCloudXYZ::Ptr
registration::SiftRegistration(PointCloudXYZ::Ptr source_cloud, PointCloudXYZ::Ptr target_cloud) {
    PointCloudXYZ::Ptr output_Cloud(new PointCloudXYZ);
    auto src_cloud_keypoints = keypoints::SiftKeyPoints(source_cloud);
    auto trg_cloud_keypoints = keypoints::SiftKeyPoints(target_cloud);
    return SACRegistration<pcl::PointXYZ>(source_cloud, target_cloud,
        *src_cloud_keypoints, *trg_cloud_keypoints);
}

pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ>
registration::FourPCS(PointCloudXYZ::Ptr src, PointCloudXYZ::Ptr targ, 
    float Overlap,
    float delta,
    size_t maxComputationTime,
    size_t NumerOfSamples) {
    pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>());
    fpcs.setInputSource(src);
    fpcs.setInputTarget(targ);
    fpcs.setSourceNormals(feature::normal::Normal_Estimation(src));
    fpcs.setTargetNormals(feature::normal::Normal_Estimation(targ));
    fpcs.setSearchMethodSource(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    fpcs.setSearchMethodTarget(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    fpcs.setApproxOverlap(Overlap);
    fpcs.setDelta(delta);
    fpcs.setMaxComputationTime(maxComputationTime);
    fpcs.setNumberOfSamples(NumerOfSamples);
	fpcs.align(*Final);
	return fpcs;
}