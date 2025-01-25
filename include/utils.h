#ifndef HIRACHICALPLANESEGMENTATION_UTILS_H
#define HIRACHICALPLANESEGMENTATION_UTILS_H

#include <opencv2/opencv.hpp>
// #define EIGEN_USE_MKL_ALL
#include "Params.h"

#include <Eigen/Dense>
// #include "open3d/Open3D.h"

using std::vector, std::cout, std::endl;

extern long g_total_time;

struct PlaneParams {
    float a = -1, b = -1, c = -1, d = -1, mean_x = -1, mean_y = -1, mean_z = -1, MSE = -1;
    double sum_x = 0, sum_y = 0, sum_z = 0, sum_xx = 0, sum_yy = 0, sum_zz = 0, sum_xy = 0, sum_xz = 0, sum_yz = 0;
    int num_points = 0;

    PlaneParams() = default;
    PlaneParams(float a, float b, float c, float d, float mean_x, float mean_y, float mean_z, float MSE, double sum_x,
                double sum_y, double sum_z, double sum_xx, double sum_yy, double sum_zz, double sum_xy, double sum_xz,
                double sum_yz, int num_points)
        : a(a), b(b), c(c), d(d), mean_x(mean_x), mean_y(mean_y), mean_z(mean_z), MSE(MSE), sum_x(sum_x), sum_y(sum_y),
          sum_z(sum_z), sum_xx(sum_xx), sum_yy(sum_yy), sum_zz(sum_zz), sum_xy(sum_xy), sum_xz(sum_xz), sum_yz(sum_yz),
          num_points(num_points) {}

    PlaneParams &operator+=(const PlaneParams &another) {
        sum_x += another.sum_x;
        sum_y += another.sum_y;
        sum_z += another.sum_z;
        sum_xx += another.sum_xx;
        sum_yy += another.sum_yy;
        sum_zz += another.sum_zz;
        sum_xy += another.sum_xy;
        sum_xz += another.sum_xz;
        sum_yz += another.sum_yz;
        num_points += another.num_points;
        return *this;
    }
};

/********** Point Cloud Convertion, visulization, io ****************/
void DepthToPointCloud(cv::Mat &d_img, double fx, double fy, double cx, double cy, double scale, int z_min,
                       Eigen::MatrixXf &cloud_array);
void ColorImgToArray(cv::Mat &color_img, Eigen::MatrixXf &color_array);
// void DrawPc(Eigen::MatrixXf original_cloud_array, cv::Mat color_img = cv::Mat());
// void DrawPc(Eigen::MatrixXf X, Eigen::MatrixXf Y, Eigen::MatrixXf Z);
// void DrawPc_o3d(cv::Mat &color_img, cv::Mat &d_img, double fx, double fy, double cx, double cy, double scale,
// double z_min);
// void SavePc(const std::string &filename, Eigen::MatrixXf cloud_array);

// two types of plane fitting
PlaneParams FitPlane(const Eigen::Block<Eigen::MatrixXf> &sampled_X, const Eigen::Block<Eigen::MatrixXf> &sampled_Y,
                     const Eigen::Block<Eigen::MatrixXf> &sampled_Z, bool debug_print = false,
                     bool debug_show_pc = false);
void FitPlaneAlreadyHaveSum(PlaneParams &params);

// count jumps in two lines, used for early rejection
int CountJumps(const Eigen::Block<Eigen::MatrixXf> &Z_matrix, float threshold);

// if two planes are coplanar
bool IsCoplanar(const PlaneParams &plane_params1, const PlaneParams &plane_params2,
                double min_cos_angle_4_merge         = cos(M_PI / 18),
                int distance_thresh_mode             = PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE,
                double max_merge_distance2_threshold = 20);

#endif // HIRACHICALPLANESEGMENTATION_UTILS_H