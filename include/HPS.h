#ifndef HIRACHICALPLANESEGMENTATION_HPS_H
#define HIRACHICALPLANESEGMENTATION_HPS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "PlaneSeg.h"
#include "HirachicalIndex.h"
#include "PointsInfoPyramid.h"
#include "PatchSegmentResult.h"

using namespace std;


class HPS {
private:
    int m_img_hight;
    int m_img_width;

// 图像递归分割相关
    int m_block_num_x;  // 首先将整个图像分成 m_block_num_x * m_block_num_y 个正方形块
    int m_block_num_y;
    int m_num_tree_layers; // 递归分割的层数
    int *m_patchs_each_level;

// 每层block的尺寸
    int *m_block_sizes;

// 点云数据
    Eigen::MatrixXf m_cloud_array;
    Eigen::MatrixXf m_organized_pointcloud;

// 灰度图
    cv::Mat m_gray_img;

    PointsInfoPyramid m_points_sum_pyramid;

    PatchSegmentResult m_patch_segment_result;

    void OrganizePointCloud();

    void RecursivePlaneSegment();

    void GetHirachicalIndexFromRowCol(int row, int col, int *hierachical_index);

    vector<int> GetRowColFromHirachicalIndex(vector<int> hirachical_index);

    inline int GetOffsetOfHirachicalIndex(const int *hirachical_index){
        int begin = 0;
        for (int i = 0; i < m_num_tree_layers; i++) {
            begin += hirachical_index[i] * m_block_sizes[i] * m_block_sizes[i];
        }
        return begin;
    };

public:
    HPS(int img_hight, int img_width, int block_num_x, int block_num_y, int tree_level);
    void SetPointCloud(const Eigen::MatrixXf &cloud_array) { this->m_cloud_array = cloud_array; };
    void SetGrayImg(cv::Mat &img){ this->m_gray_img = img; };
    void Process();

    void DrawPatchByHierachicalIndex(const vector<int> &hierachical_index, cv::Mat &img_copy);

    ~HPS() {
        delete[] m_patchs_each_level;
        delete[] m_block_sizes;
    };
};


#endif //HIRACHICALPLANESEGMENTATION_HPS_H
