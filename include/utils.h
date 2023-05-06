#ifndef HIRACHICALPLANESEGMENTATION_UTILS_H
#define HIRACHICALPLANESEGMENTATION_UTILS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std;

void DepthToPointCloud(cv::Mat & d_img, float fx, float fy, float cx, float cy, double z_min, Eigen::MatrixX3f & cloud_array);

void PrintVector(vector<int>& v);

void DrawPc(Eigen::MatrixXf cloud_array);

#endif //HIRACHICALPLANESEGMENTATION_UTILS_H