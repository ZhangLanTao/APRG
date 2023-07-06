#ifndef HIRACHICALPLANESEGMENTATION_UTILS_H
#define HIRACHICALPLANESEGMENTATION_UTILS_H

#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>

using namespace std;

class PlaneParams {
public:
    double a, b, c, d, mean_x, mean_y, mean_z, MSE, score;

    PlaneParams(double a, double b, double c, double d, double mean_x, double mean_y, double mean_z ,double MSE, double score) :
        a(a), b(b), c(c), d(d), mean_x(mean_x), mean_y(mean_y), mean_z(mean_z), MSE(MSE), score(score) {};
    PlaneParams() : a(0), b(0), c(0), d(0), mean_x(0), mean_y(0), mean_z(0), MSE(9999), score(0) {};;

    static bool IsCoplanar(const PlaneParams &plane_params1, const PlaneParams &plane_params2, float min_cos_angle_4_merge=cos(M_PI/36), float max_distance_4_merge=1.0) {
        double cos_angle = plane_params1.a*plane_params2.a+plane_params1.b*plane_params2.b+plane_params1.c*plane_params2.c;
        double distance_2 = pow(plane_params1.a*plane_params2.mean_x + plane_params1.b*plane_params2.mean_y + plane_params1.c*plane_params2.mean_z + plane_params1.d,2);
//        cout<<"cos_angle: "<<cos_angle<<endl;
//        cout<<"distance_2: "<<distance_2<<endl;

        if (cos_angle < min_cos_angle_4_merge|| distance_2 > max_distance_4_merge) return false;
        else return true;
    }

    bool IsPlane() const {
        if (MSE > 0.1 || c>-0.8)
            return false;
        return true;
    }
};

class PointsSum {
public:
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    float sum_xx = 0;
    float sum_yy = 0;
    float sum_zz = 0;
    float sum_xy = 0;
    float sum_xz = 0;
    float sum_yz = 0;
    int num_points = 0;
    int jump_cnt = 0;

    PlaneParams FitPlane() const;
    static PointsSum Add(const PointsSum &a, const PointsSum &b, const PointsSum &c, const PointsSum &d);
};

void DepthToPointCloud(cv::Mat & d_img, float fx, float fy, float cx, float cy, double z_min, Eigen::MatrixXf & cloud_array);

void PrintVector(vector<int>& v);

void DrawPc(Eigen::MatrixXf cloud_array);

void SavePc(Eigen::MatrixXf cloud_array);

void Waste100us();

#endif //HIRACHICALPLANESEGMENTATION_UTILS_H