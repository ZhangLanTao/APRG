#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include "utils.h"

using std::vector, std::cout, std::endl;

long g_program_counter;

void DepthToPointCloud(cv::Mat &d_img, float fx, float fy, float cx, float cy, double scale, double z_min,
                       Eigen::MatrixXd &cloud_array) {
    int height = d_img.rows;
    int width = d_img.cols;
    cloud_array.setZero(height*width,3);
#ifdef ENABLE_OMP
omp_set_num_threads(8);
#pragma omp parallel for default(none) shared(d_img, height, width, fx, fy, cx, cy, z_min, cloud_array, scale)
#endif
    for(int r=0; r< height; r++){
        for(auto c=0; c< width; c++){
            float z = scale *d_img.at<float>(r, c);
            if(z>z_min){
                cloud_array(r*width+c,0) = (c-cx)*z/fx;
                cloud_array(r*width+c,1) = (r-cy)*z/fy;
                cloud_array(r*width+c,2) = z;
            }
        }
    }
}


void ColorImgToArray(cv::Mat &color_img, Eigen::MatrixXd &color_array) {
    int height = color_img.rows;
    int width = color_img.cols;
    color_array.setZero(height*width,3);

    for(int r=0; r< height; r++){
        for(auto c=0; c< width; c++){
            color_array(r*width+c,0) = color_img.at<cv::Vec3b>(r, c)[2]/255.;
            color_array(r*width+c,1) = color_img.at<cv::Vec3b>(r, c)[1]/255.;
            color_array(r*width+c,2) = color_img.at<cv::Vec3b>(r, c)[0]/255.;
        }
    }
}

void DrawPc_o3d(cv::Mat &color_img, cv::Mat &d_img, double fx, double fy, double cx, double cy,
                double scale, double z_min) {
    auto width = d_img.cols;
    auto height = d_img.rows;

    open3d::geometry::Image color_img_o3d, d_img_o3d;
    // Allocate data buffer
    color_img_o3d.Prepare(width, height, 3, 1);
    d_img_o3d.Prepare(width, height, 1, 4);
    // mat is your cv::Mat depth image
    memcpy(color_img_o3d.data_.data(), color_img.data, d_img_o3d.data_.size());
    memcpy(d_img_o3d.data_.data(), d_img.data, d_img_o3d.data_.size());

    std::shared_ptr<open3d::geometry::RGBDImage> rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(color_img_o3d, d_img_o3d, 5, 1000/ scale, false);
    std::shared_ptr<open3d::geometry::PointCloud> cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, cx, cy));
    open3d::visualization::DrawGeometriesWithEditing({cloud});

}

void DrawPc(Eigen::MatrixXd original_cloud_array, cv::Mat color_img){
    Eigen::MatrixXd color_array;
    ColorImgToArray(color_img, color_array);
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> colors;
    for (int i = 0 ; i < original_cloud_array.rows(); ++i) {
        auto p = original_cloud_array.row(i);
        if (p.isZero()) continue;
        points.emplace_back(p.cast<double>());
        colors.emplace_back(color_array.row(i).cast<double>());
    }
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_ = points;
    cloud->colors_ = colors;

    open3d::visualization::DrawGeometries({cloud}, "PointCloud", 1600, 1200, 1440, 50);
}

void SavePc(const std::string filename, Eigen::MatrixXd cloud_array) {
    std::vector<Eigen::Vector3d> points;
    for (int i = 0 ; i<cloud_array.rows(); ++i) {
        auto p = cloud_array.row(i);
        if (p.isZero()) continue;
        points.emplace_back(p.cast<double>());
    }
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_ = points;

    open3d::io::WritePointCloudToPLY(filename, *cloud, true);
}

void PrintVector(vector<int>& v)
{
        for (auto e : v)
        {
            cout << e << " ";
        }
        cout << endl;
}


PointsSum PointsSum::AddFour(const PointsSum &a, const PointsSum &b, const PointsSum &c, const PointsSum &d) {
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

PlaneParams PointsSum::FitPlane(bool debug) const{
    g_program_counter++;
    double mean[3];
    mean[0] = sum_x / num_points;
    mean[1] = sum_y / num_points;
    mean[2] = sum_z / num_points;
    // Expressing covariance as E[PP^t] + E[P]*E[P^T]
    double cov[3][3] = {
            {sum_xx/num_points - sum_x/num_points * sum_x / num_points, sum_xy/num_points - sum_x/num_points * sum_y / num_points, sum_xz/num_points - sum_x/num_points * sum_z / num_points},
            {0,                                   sum_yy/num_points - sum_y/num_points * sum_y / num_points, sum_yz/num_points - sum_y/num_points * sum_z / num_points},
            {0,                                   0,sum_zz/num_points - sum_z/num_points * sum_z / num_points}
    };
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];

    Eigen::Map<Eigen::Matrix3d, 0, Eigen::Stride<0, 0>> cov_eigen = Eigen::Map<Eigen::Matrix3d>(cov[0], 3, 3);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_eigen);
    Eigen::VectorXd v = es.eigenvectors().col(0);

//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov_eigen,  Eigen::ComputeFullV);
//    Eigen::VectorXd v = svd.matrixV().col(2);

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
//    double min_sv = svd.singularValues()(svd.singularValues().size()-1);
//    double max_sv = svd.singularValues()(0);

    double MSE, score;
    MSE = min_ev;
//    score = svd.singularValues()(1) / min_sv;

    if (debug) {
        cout << "???\n";
        cout << "COV eigen: \n" << cov_eigen << endl;
//        cout << "SingularVals: \n" << svd.singularValues() << endl;
        cout << "MSE: " << MSE << "\n";
        cout <<"Score: "<<score<<"\n\n";
    }
    return PlaneParams{normal[0], normal[1], normal[2], d, mean[0], mean[1], mean[2], MSE, score, num_points, jump_cnt};
}

PointsSum& PointsSum::operator+=(const PointsSum &another) {
    this->sum_x += another.sum_x;
    this->sum_y += another.sum_y;
    this->sum_z += another.sum_z;
    this->sum_xx += another.sum_xx;
    this->sum_yy += another.sum_yy;
    this->sum_zz += another.sum_zz;
    this->sum_xy += another.sum_xy;
    this->sum_xz += another.sum_xz;
    this->sum_yz += another.sum_yz;
    this->num_points += another.num_points;
    this->jump_cnt += another.jump_cnt;
    return *this;
}

void Waste100us(){
//    sleep 10ms
    clock_t start_time = clock();
    while (clock() - start_time < 100);
}