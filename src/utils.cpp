#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include "utils.h"

using std::vector, std::cout, std::endl;

void DepthToPointCloud(cv::Mat & d_img, float fx, float fy, float cx, float cy, double z_min, Eigen::MatrixXf & cloud_array){
    int height = d_img.rows;
    int width = d_img.cols;
    cloud_array.setZero(height*width,3);
#ifdef ENABLE_OMP
omp_set_num_threads(8);
#pragma omp parallel for default(none) shared(d_img, height, width, fx, fy, cx, cy, z_min, cloud_array)
#endif
    for(int r=0; r< height; r++){
        for(auto c=0; c< width; c++){
            float z = d_img.at<float>(r, c);
            if(z>z_min){
                cloud_array(r*width+c,0) = (c-cx)*z/fx;
                cloud_array(r*width+c,1) = (r-cy)*z/fy;
                cloud_array(r*width+c,2) = z;
            }
        }
    }
}


void DrawPc(Eigen::MatrixXf cloud_array){
    std::vector<Eigen::Vector3d> points;
    for (int i = 0 ; i<cloud_array.rows(); ++i) {
        Eigen::Vector3f p = cloud_array.row(i);
        if (p.isZero()) continue;
        points.push_back(p.cast<double>());
    }
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_ = points;

    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D", 1200, 900);
    visualizer.AddGeometry(cloud);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}

void SavePc(Eigen::MatrixXf cloud_array){
    std::vector<Eigen::Vector3d> points;
    for (int i = 0 ; i<cloud_array.rows(); ++i) {
        Eigen::Vector3f p = cloud_array.row(i);
        if (p.isZero()) continue;
        points.push_back(p.cast<double>());
    }
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_ = points;

    open3d::io::WritePointCloudToPLY("/home/zlt/Desktop/pc.ply", *cloud, false);
}

void PrintVector(vector<int>& v)
{
        for (auto e : v)
        {
            cout << e << " ";
        }
        cout << endl;
}


PointsSum PointsSum::Add(const PointsSum &a, const PointsSum &b, const PointsSum &c, const PointsSum &d) {
    PointsSum result;
    result.sum_x = a.sum_x + b.sum_x + c.sum_x + d.sum_x;
    result.sum_y = a.sum_y + b.sum_y + c.sum_y + d.sum_y;
    result.sum_z = a.sum_z + b.sum_z + c.sum_z + d.sum_z;
    result.sum_xx = a.sum_xx + b.sum_xx + c.sum_xx + d.sum_xx;
    result.sum_yy = a.sum_yy + b.sum_yy + c.sum_yy + d.sum_yy;
    result.sum_zz = a.sum_zz + b.sum_zz + c.sum_zz + d.sum_zz;
    result.sum_xy = a.sum_xy + b.sum_xy + c.sum_xy + d.sum_xy;
    result.sum_xz = a.sum_xz + b.sum_xz + c.sum_xz + d.sum_xz;
    result.sum_yz = a.sum_yz + b.sum_yz + c.sum_yz + d.sum_yz;
    result.num_points = a.num_points + b.num_points + c.num_points + d.num_points;
    result.jump_cnt = a.jump_cnt + b.jump_cnt + c.jump_cnt + d.jump_cnt;
    return result;
}

PlaneParams PointsSum::FitPlane() const{
//    if (this->jump_cnt > 2) return{};
//    if (this->num_points < 50) return{};

    double mean[3];
    mean[0] = sum_x / num_points;
    mean[1] = sum_y / num_points;
    mean[2] = sum_z / num_points;
    // Expressing covariance as E[PP^t] + E[P]*E[P^T]
    double cov[3][3] = {
            {sum_xx - sum_x * sum_x / num_points, sum_xy - sum_x * sum_y / num_points, sum_xz - sum_x * sum_z / num_points},
            {0,                                   sum_yy - sum_y * sum_y / num_points, sum_yz - sum_y * sum_z / num_points},
            {0,                                   0,                                   sum_zz - sum_z * sum_z / num_points}
    };
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];

    // This uses QR decomposition for symmetric matrices
    Eigen::Map<Eigen::Matrix3d, 0, Eigen::Stride<0, 0>> cov_eigen = Eigen::Map<Eigen::Matrix3d>(cov[0], 3, 3);
//    cov_eigen = cov_eigen / num_points;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_eigen);
    Eigen::VectorXd v = es.eigenvectors().col(0);

    double d, normal[3];
    d = -(v[0] * mean[0] + v[1] * mean[1] + v[2] * mean[2]);
    // Enforce normal orientation
    if (d > 0) {
        normal[0] = v[0];
        normal[1] = v[1];
        normal[2] = v[2];
    } else {
        normal[0] = -v[0];
        normal[1] = -v[1];
        normal[2] = -v[2];
        d = -d;
    }
    double min_ev = es.eigenvalues()[0];
    //if (min_ev < 15) min_ev = 15;     // eigen 精度问题，最小值可能小于0，这里根据经验把最小值设为15

    double MSE, score;
    MSE = min_ev / num_points;
    score = es.eigenvalues()[1] / min_ev;
    return PlaneParams{normal[0], normal[1], normal[2], d, mean[0], mean[1], mean[2], MSE, score};
}

void Waste100us(){
//    sleep 10ms
    clock_t start_time = clock();
    while (clock() - start_time < 100);
}