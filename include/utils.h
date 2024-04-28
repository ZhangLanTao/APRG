#ifndef HIRACHICALPLANESEGMENTATION_UTILS_H
#define HIRACHICALPLANESEGMENTATION_UTILS_H

#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include "open3d/Open3D.h"

extern long g_program_counter;

enum PlanePatchThreshMode {
    PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE = 0,
    PLANE_PATCH_THRESHMODE_DISTANCE_RATIO = 1
};

enum PlaneMergeThreshMode {
    PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE = 0,
    PLANE_MERGE_THRESHMODE_DISTANCE_RATIO = 1
};


class PlaneParams {
public:
    double a, b, c, d, mean_x, mean_y, mean_z, MSE, score;
    int num_points, jump_cnt;

    PlaneParams(double a, double b, double c, double d, double mean_x, double mean_y, double mean_z ,double MSE, double score, int points, int jump_cnt) :
            a(a), b(b), c(c), d(d), mean_x(mean_x), mean_y(mean_y), mean_z(mean_z), MSE(MSE), score(score), num_points(points), jump_cnt(jump_cnt) {};
    PlaneParams() : a(0), b(0), c(0), d(0), mean_x(0), mean_y(0), mean_z(0), MSE(9999), score(0), num_points(0), jump_cnt(0) {};;

    static bool IsCoplanar(const PlaneParams &plane_params1, const PlaneParams &plane_params2,
                           double min_cos_angle_4_merge = cos(M_PI / 18),
                           int distance_thresh_mode= PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE,
                           double max_merge_distance2_threshold = 20) {
        double cos_angle = plane_params1.a*plane_params2.a+plane_params1.b*plane_params2.b+plane_params1.c*plane_params2.c;
        double distance2 = pow(plane_params1.a * plane_params2.mean_x + plane_params1.b * plane_params2.mean_y + plane_params1.c * plane_params2.mean_z + plane_params1.d, 2);

        if (cos_angle < min_cos_angle_4_merge) return false;

        if (distance_thresh_mode == PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE && distance2 > max_merge_distance2_threshold) return false;

        if (distance_thresh_mode == PLANE_MERGE_THRESHMODE_DISTANCE_RATIO &&
            distance2 > max_merge_distance2_threshold *
                        (plane_params1.mean_x * plane_params1.mean_x + plane_params1.mean_y * plane_params1.mean_y+ plane_params1.mean_z * plane_params1.mean_z))
            return false;

        return true;
    }

    bool IsPlane(int thresh_mode, double thresh) const {
        if (jump_cnt>2) return false;
        if (thresh_mode == PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE && MSE > thresh) return false;
        if (thresh_mode == PLANE_PATCH_THRESHMODE_DISTANCE_RATIO && MSE > thresh * (mean_x * mean_x + mean_y * mean_y + mean_z * mean_z)) return false;
        return true;
    }
};

class PointsSum {
public:
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    double sum_xx = 0;
    double sum_yy = 0;
    double sum_zz = 0;
    double sum_xy = 0;
    double sum_xz = 0;
    double sum_yz = 0;
    int num_points = 0;
    int jump_cnt = 0;

    PlaneParams FitPlane(bool debug) const;
    static PointsSum AddFour(const PointsSum &a, const PointsSum &b, const PointsSum &c, const PointsSum &d);
    PointsSum& operator+= (const PointsSum &another);
};

void DepthToPointCloud(cv::Mat &d_img, float fx, float fy, float cx, float cy, double scale, double z_min,
                       Eigen::MatrixXd &cloud_array);
void ColorImgToArray(cv::Mat &color_img, Eigen::MatrixXd &color_array);

void PrintVector(std::vector<int>& v);

void DrawPc(Eigen::MatrixXd original_cloud_array, cv::Mat color_img);

void DrawPc_o3d(cv::Mat &color_img, cv::Mat &d_img, double fx, double fy, double cx, double cy,
                double scale, double z_min);

void SavePc(const std::string filename, Eigen::MatrixXd cloud_array);

void Waste100us();

#endif //HIRACHICALPLANESEGMENTATION_UTILS_H