#include <opencv2/opencv.hpp>
// #define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>

// #include "open3d/Open3D.h"
#include "utils.h"

using std::vector, std::cout, std::endl;

long g_total_time;

void DepthToPointCloud(cv::Mat &d_img, double fx, double fy, double cx, double cy, double scale, int z_min,
                       Eigen::MatrixXf &cloud_array) {
    int height = d_img.rows;
    int width  = d_img.cols;
    cloud_array.setZero(height * width, 3);
#ifdef ENABLE_OMP
    omp_set_num_threads(8);
#pragma omp parallel for default(none) shared(d_img, height, width, fx, fy, cx, cy, z_min, cloud_array, scale)
#endif
    for (int r = 0; r < height; r++) {
        for (auto c = 0; c < width; c++) {
            float z = scale * d_img.at<float>(r, c);
            if (z > z_min) {
                cloud_array(r * width + c, 0) = (c - cx) * z / fx;
                cloud_array(r * width + c, 1) = (r - cy) * z / fy;
                cloud_array(r * width + c, 2) = z;
            }
        }
    }
}

void ColorImgToArray(cv::Mat &color_img, Eigen::MatrixXf &color_array) {
    int height = color_img.rows;
    int width  = color_img.cols;
    color_array.setZero(height * width, 3);

    for (int r = 0; r < height; r++) {
        for (auto c = 0; c < width; c++) {
            color_array(r * width + c, 0) = color_img.at<cv::Vec3b>(r, c)[2] / 255.;
            color_array(r * width + c, 1) = color_img.at<cv::Vec3b>(r, c)[1] / 255.;
            color_array(r * width + c, 2) = color_img.at<cv::Vec3b>(r, c)[0] / 255.;
        }
    }
}

/* In the middle row and the middle column, count jumps
 *  input:
 *      Z_matrix : n*n block
 *      threshold: max distance to the line
 *  */
int CountJumps(const Eigen::Block<Eigen::MatrixXf> &Z_matrix, float threshold) {
    int jumps_count = 0;
    int valid_ind0 = 0, valid_ind1 = 0;
    float valid_z0 = 0, valid_z1 = 0;
    // horizontal search, get valid z and ind
    int middle = Z_matrix.rows() / 2;
    for (int i = 0; i < Z_matrix.cols(); ++i) {
        auto z = Z_matrix(middle, i);
        if (z != 0) {
            valid_z0   = z;
            valid_ind0 = i;
            break;
        }
    }
    for (int i = Z_matrix.cols() - 1; i >= 0; --i) {
        auto z = Z_matrix(middle, i);
        if (z != 0) {
            valid_z1   = z;
            valid_ind1 = i;
            break;
        }
    }
    if (valid_z0 == 0 or valid_z1 == 0)
        return 999;
    float k = (valid_z1 - valid_z0) / (valid_ind1 - valid_ind0);
    for (int i = valid_ind0; i < valid_ind1; ++i) {
        auto z = Z_matrix(middle, i);
        if (z == 0)
            continue;
        float predict_z = valid_z0 + k * (i - valid_ind0);
        if (abs(z - predict_z) > threshold)
            ++jumps_count;
    }

    // vertical search, get valid z and ind
    valid_ind0 = 0, valid_ind1 = 0;
    valid_z0 = 0, valid_z1 = 0;
    middle = Z_matrix.cols() / 2;
    for (int i = 0; i < Z_matrix.rows(); ++i) {
        auto z = Z_matrix(i, middle);
        if (z != 0) {
            valid_z0   = z;
            valid_ind0 = i;
            break;
        }
    }
    for (int i = Z_matrix.rows() - 1; i >= 0; --i) {
        auto z = Z_matrix(i, middle);
        if (z != 0) {
            valid_z1   = z;
            valid_ind1 = i;
            break;
        }
    }
    if (valid_z0 == 0 or valid_z1 == 0)
        return 999;
    k = (valid_z1 - valid_z0) / (valid_ind1 - valid_ind0);
    for (int i = valid_ind0; i < valid_ind1; ++i) {
        auto z = Z_matrix(i, middle);
        if (z == 0)
            continue;
        float predict_z = valid_z0 + k * (i - valid_ind0);
        if (abs(z - predict_z) > threshold)
            ++jumps_count;
    }

    return jumps_count;
}

// Fit plane using pre calculated sum
// &params: changed inplace
void FitPlaneAlreadyHaveSum(PlaneParams &params) {
    float mean[3];
    mean[0] = params.sum_x / params.num_points;
    mean[1] = params.sum_y / params.num_points;
    mean[2] = params.sum_z / params.num_points;
    // Expressing covariance as E[PP^t] + E[P]*E[P^T]
    float cov[3][3] = {{static_cast<float>(params.sum_xx / params.num_points -
                                           params.sum_x / params.num_points * params.sum_x / params.num_points),
                        static_cast<float>(params.sum_xy / params.num_points -
                                           params.sum_x / params.num_points * params.sum_y / params.num_points),
                        static_cast<float>(params.sum_xz / params.num_points -
                                           params.sum_x / params.num_points * params.sum_z / params.num_points)},
                       {0,
                        static_cast<float>(params.sum_yy / params.num_points -
                                           params.sum_y / params.num_points * params.sum_y / params.num_points),
                        static_cast<float>(params.sum_yz / params.num_points -
                                           params.sum_y / params.num_points * params.sum_z / params.num_points)},
                       {0, 0,
                        static_cast<float>(params.sum_zz / params.num_points -
                                           params.sum_z / params.num_points * params.sum_z / params.num_points)}};
    cov[1][0]       = cov[0][1];
    cov[2][0]       = cov[0][2];
    cov[2][1]       = cov[1][2];

    Eigen::Matrix3f cov_eigen;
    cov_eigen << cov[0][0], cov[0][1], cov[0][2], cov[1][0], cov[1][1], cov[1][2], cov[2][0], cov[2][1], cov[2][2];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov_eigen);
    Eigen::VectorXf v = es.eigenvectors().col(0);
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov_eigen,  Eigen::ComputeFullV);
    // Eigen::VectorXd v = svd.matrixV().col(2);

    float d, normal[3];
    d = -(v[0] * mean[0] + v[1] * mean[1] + v[2] * mean[2]);
    // change normal orientation
    if (d > 0) {
        normal[0] = v[0];
        normal[1] = v[1];
        normal[2] = v[2];
    } else {
        normal[0] = -v[0];
        normal[1] = -v[1];
        normal[2] = -v[2];
        d         = -d;
    }
    float min_ev = abs(es.eigenvalues()[0]);

    params.a      = normal[0];
    params.b      = normal[1];
    params.c      = normal[2];
    params.d      = d;
    params.mean_x = mean[0];
    params.mean_y = mean[1];
    params.mean_z = mean[2];
    params.MSE    = min_ev;
}

// Fit plane using points
PlaneParams FitPlane(const Eigen::Block<Eigen::MatrixXf> &sampled_X, const Eigen::Block<Eigen::MatrixXf> &sampled_Y,
                     const Eigen::Block<Eigen::MatrixXf> &sampled_Z, bool debug_print, bool debug_show_pc) {
    double sum_x   = sampled_X.sum();
    double sum_y   = sampled_Y.sum();
    double sum_z   = sampled_Z.sum();
    double sum_xx  = (sampled_X.array() * sampled_X.array()).sum();
    double sum_yy  = (sampled_Y.array() * sampled_Y.array()).sum();
    double sum_zz  = (sampled_Z.array() * sampled_Z.array()).sum();
    double sum_xy  = (sampled_X.array() * sampled_Y.array()).sum();
    double sum_xz  = (sampled_X.array() * sampled_Z.array()).sum();
    double sum_yz  = (sampled_Y.array() * sampled_Z.array()).sum();
    int num_points = sampled_X.rows() * sampled_X.cols();

    float mean[3];
    mean[0] = sum_x / num_points;
    mean[1] = sum_y / num_points;
    mean[2] = sum_z / num_points;
    // Expressing covariance as E[PP^t] + E[P]*E[P^T]
    double cov[3][3] = {{sum_xx / num_points - sum_x / num_points * sum_x / num_points,
                         sum_xy / num_points - sum_x / num_points * sum_y / num_points,
                         sum_xz / num_points - sum_x / num_points * sum_z / num_points},
                        {0, sum_yy / num_points - sum_y / num_points * sum_y / num_points,
                         sum_yz / num_points - sum_y / num_points * sum_z / num_points},
                        {0, 0, sum_zz / num_points - sum_z / num_points * sum_z / num_points}};
    cov[1][0]        = cov[0][1];
    cov[2][0]        = cov[0][2];
    cov[2][1]        = cov[1][2];

    Eigen::Matrix3f cov_eigen;
    cov_eigen << cov[0][0], cov[0][1], cov[0][2], cov[1][0], cov[1][1], cov[1][2], cov[2][0], cov[2][1], cov[2][2];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov_eigen);
    Eigen::VectorXf v = es.eigenvectors().col(0);
    //    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov_eigen,  Eigen::ComputeFullV);
    //    Eigen::VectorXd v = svd.matrixV().col(2);

    float d, normal[3];
    d = -(v[0] * mean[0] + v[1] * mean[1] + v[2] * mean[2]);
    // change normal orientation
    if (d > 0) {
        normal[0] = v[0];
        normal[1] = v[1];
        normal[2] = v[2];
    } else {
        normal[0] = -v[0];
        normal[1] = -v[1];
        normal[2] = -v[2];
        d         = -d;
    }
    float min_ev = abs(es.eigenvalues()[0]);
    // double min_sv = svd.singularValues()(svd.singularValues().size()-1);
    // double max_sv = svd.singularValues()(0);

    if (debug_print) {
        cout << "COV eigen: " << cov_eigen << endl;
        cout << "MSE: " << min_ev << "\n";
        cout << "Plane param a b c d: " << normal[0] << " " << normal[1] << " " << normal[2] << " " << d << endl;
    }
    if (debug_show_pc) {
        Eigen::MatrixXf temp_x, temp_y, temp_z;
        temp_x = sampled_X;
        temp_y = sampled_Y;
        temp_z = sampled_Z;
        temp_x.resize(num_points, 1);
        temp_y.resize(num_points, 1);
        temp_z.resize(num_points, 1);
        auto cloud_array = Eigen::MatrixXf(num_points, 3);
        cloud_array << temp_x, temp_y, temp_z;
        // DrawPc(cloud_array);
    }
    return {normal[0], normal[1], normal[2], d,      mean[0], mean[1], mean[2], min_ev, sum_x,
            sum_y,     sum_z,     sum_xx,    sum_yy, sum_zz,  sum_xy,  sum_xz,  sum_yz, num_points};
}

// Two planes are coplanar or not
bool IsCoplanar(const PlaneParams &p1, const PlaneParams &p2, double min_cos_angle_4_merge, int distance_thresh_mode,
                double max_merge_distance2_threshold) {
    double cos_angle = p1.a * p2.a + p1.b * p2.b + p1.c * p2.c;
    double distance2 = pow(p1.a * p2.mean_x + p1.b * p2.mean_y + p1.c * p2.mean_z + p1.d, 2);

    if (cos_angle < min_cos_angle_4_merge)
        return false;

    if (distance_thresh_mode == PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE && distance2 > max_merge_distance2_threshold)
        return false;

    if (distance_thresh_mode == PLANE_MERGE_THRESHMODE_DISTANCE_RATIO &&
        distance2 >
            max_merge_distance2_threshold * (p1.mean_x * p1.mean_x + p1.mean_y * p1.mean_y + p1.mean_z * p1.mean_z))
        return false;

    return true;
}

/**************************************** These are open3d related functions for debug *******************************/
// void DrawPc_o3d(cv::Mat &color_img, cv::Mat &d_img, double fx, double fy, double cx, double cy, double scale,
//                 double z_min) {
//   auto width = d_img.cols;
//   auto height = d_img.rows;

//   open3d::geometry::Image color_img_o3d, d_img_o3d;
//   // Allocate data buffer
//   color_img_o3d.Prepare(width, height, 3, 1);
//   d_img_o3d.Prepare(width, height, 1, 4);
//   // mat is your cv::Mat depth image
//   memcpy(color_img_o3d.data_.data(), color_img.data, d_img_o3d.data_.size());
//   memcpy(d_img_o3d.data_.data(), d_img.data, d_img_o3d.data_.size());

//   std::shared_ptr<open3d::geometry::RGBDImage> rgbd =
//       open3d::geometry::RGBDImage::CreateFromColorAndDepth(color_img_o3d, d_img_o3d, 5, 1000 / scale, false);
//   std::shared_ptr<open3d::geometry::PointCloud> cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(
//       *rgbd, open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, cx, cy));
//   open3d::visualization::DrawGeometriesWithEditing({cloud});
// }

// void DrawPc(Eigen::MatrixXf original_cloud_array, cv::Mat color_img) {
//   Eigen::MatrixXf color_array;
//   if (!color_img.empty())
//     ColorImgToArray(color_img, color_array);
//   std::vector<Eigen::Vector3d> points;
//   std::vector<Eigen::Vector3d> colors;
//   for (int i = 0; i < original_cloud_array.rows(); ++i) {
//     auto p = original_cloud_array.row(i);
//     if (p.isZero())
//       continue;
//     points.emplace_back(p.cast<double>());
//     if (!color_img.empty())
//       colors.emplace_back(color_array.row(i).cast<double>());
//   }
//   auto cloud = std::make_shared<open3d::geometry::PointCloud>();
//   cloud->points_ = points;
//   if (!color_img.empty())
//     cloud->colors_ = colors;

//   open3d::visualization::DrawGeometriesWithEditing({cloud}, "PointCloud", 1600, 1200, 1440, 50);
// }

// X Y Z are r*c matrixes
// void DrawPc(Eigen::MatrixXf X, Eigen::MatrixXf Y, Eigen::MatrixXf Z) {
//   std::vector<Eigen::Vector3d> points;
//   for (int i = 0; i < X.rows(); ++i) {
//     for (int j = 0; j < X.cols(); ++j) {
//       auto p = Z(i, j);
//       if (p == 0)
//         continue;
//       points.emplace_back(Eigen::Vector3d(X(i, j), Y(i, j), Z(i, j)));
//     }
//   }
//   auto cloud = std::make_shared<open3d::geometry::PointCloud>();
//   cloud->points_ = points;
//   open3d::visualization::DrawGeometriesWithEditing({cloud}, "PointCloud", 1600, 1200, 1440, 50);
// }

// void SavePc(const std::string &filename, Eigen::MatrixXf cloud_array) {
//   std::vector<Eigen::Vector3d> points;
//   for (int i = 0; i < cloud_array.rows(); ++i) {
//     auto p = cloud_array.row(i);
//     if (p.isZero())
//       continue;
//     points.emplace_back(p.cast<double>());
//   }
//   auto cloud = std::make_shared<open3d::geometry::PointCloud>();
//   cloud->points_ = points;

//   open3d::io::WritePointCloudToPLY(filename, *cloud, true);
// }