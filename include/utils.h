#ifndef HIRACHICALPLANESEGMENTATION_UTILS_H
#define HIRACHICALPLANESEGMENTATION_UTILS_H

#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include "open3d/Open3D.h"
#include "Params.h"

using std::vector, std::cout, std::endl;

extern long g_program_counter;
extern long g_total_time;

struct PlaneParams{
    float a=-1, b=-1, c=-1, d=-1, mean_x=-1, mean_y=-1, mean_z=-1, MSE=-1;                          // 基本的平面参数信息
    double sum_x=0, sum_y=0, sum_z=0, sum_xx=0, sum_yy=0, sum_zz=0, sum_xy=0, sum_xz=0, sum_yz=0;   // 用于拟合平面的求和信息，合并平面时使用
    int num_points=0;                                                                               // 点数，合并时使用
    PlaneParams() = default;
    PlaneParams &operator += (const PlaneParams &another){
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

/**********************************************点云转换、显示、存储等******************************************************/
void DepthToPointCloud(cv::Mat &d_img, double fx, double fy, double cx, double cy, double scale, int z_min, Eigen::MatrixXf &cloud_array);
void ColorImgToArray(cv::Mat &color_img, Eigen::MatrixXf &color_array);
void DrawPc(Eigen::MatrixXf original_cloud_array, cv::Mat color_img=cv::Mat());
void DrawPc(Eigen::MatrixXf X, Eigen::MatrixXf Y, Eigen::MatrixXf Z);
void DrawPc_o3d(cv::Mat &color_img, cv::Mat &d_img, double fx, double fy, double cx, double cy, double scale, double z_min);
void SavePc(const std::string& filename, Eigen::MatrixXf cloud_array);

// 两种拟合平面
PlaneParams FitPlane(const Eigen::Block<Eigen::MatrixXf>& sampled_X, const Eigen::Block<Eigen::MatrixXf>& sampled_Y, const Eigen::Block<Eigen::MatrixXf>& sampled_Z, bool debug_print=false, bool debug_show_pc=false);
void FitPlaneAlreadyHaveSum(PlaneParams &params);

// 横向纵向两条线拟合，跳变太多就不是平面
int CountJumps(const Eigen::Block<Eigen::MatrixXf> &Z_matrix, float threshold);

// 两个平面是否共面
bool IsCoplanar(const PlaneParams &plane_params1, const PlaneParams &plane_params2,
                double min_cos_angle_4_merge = cos(M_PI / 18),
                int distance_thresh_mode= PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE,
                double max_merge_distance2_threshold = 20);

#endif //HIRACHICALPLANESEGMENTATION_UTILS_H