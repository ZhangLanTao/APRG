#ifndef HIERARCHICALPLANESEGMENTATION_HPS_H
#define HIERARCHICALPLANESEGMENTATION_HPS_H


#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include <utility>
#include "PointsSamplePyramid.h"
#include "SegStatePyramid.h"
#include "Params.h"


#define TRUE

class HPS {
private:
    /****************************************************通用性数据******************************************************/
    Params _params;     // 算法所有输入参数
    cv::Mat _color_img; // 输入彩色图
    cv::Mat _layer0_label_img, _layer0_label_img_eroded, _layer0_label_img_eroded_scaleup;  // 状态金字塔底层(即最小patch)的label图像
    cv::Mat _final_label_img;   // 像素级的最终label

    PointsSamplePyramid _points_sample_pyramid;         // 数据采样金字塔
    SegStatePyramid _seg_state_pyramid;                 // 分割状态记录金字塔

    /***************************************************区域生长相关*****************************************************/
    int _RG_region_count = 1;                           // 0表示无效，从1开始
    vector<int> _RG_region_areas{0};                    // 初始一个0占位
    vector<PlaneParams> _RG_region_plane_params{{}};    // 初始一个空占位

    /***************************************************像素细分相关*****************************************************/
    float* _BR_pixel_refine_min_distance_to_plane;         // 记录像素被分配到平面的时候，已知的最小距离，仅当更小时更新，为了提速，可以不适用

    cv::Mat _Debug_label_color_lut = cv::Mat(1, 256, CV_8UC3);    // 用于给输出染色的lut

    void _Init();

    /***********************************************整体流程的几个大步骤**************************************************/
    void _BuildPyramid();            // 构建数据采样金字塔
    void _RecursivePlaneSegment();   // 递归分割平面
    void _PatchwiseRegionGrowing();  // 块级区域生长
    void _BoundaryRefine();          // 像素级边界细化

    /***************************************************具体的小功能*****************************************************/
    bool _RG_GetNextSeed(vector<int> &seed_ind);                         // 获取下一个种子点
    void _RG_RegionGrowingFrom(const vector<int> &seed_ind, int label);  // 从一个种子点完成一个区域的生长
    vector<vector<int>> _RG_GetNeighbors(const vector<int> &ind);        // 获取一个patch的大小邻居


public:
    HPS(Params P):_params(P){ _Init();};

    void SetData(const cv::Mat &color_img, const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z);
    void Process();
    void GetResultReadable(cv::Mat &output);
    void Clear();

    void DebugShowAPatch_HIndex(vector<int> ind);
    void DebugShowResult(int waitkey = 0);
};

#endif //HIERARCHICALPLANESEGMENTATION_HPS_H
