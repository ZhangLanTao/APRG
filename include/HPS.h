#ifndef HIRACHICALPLANESEGMENTATION_HPS_H
#define HIRACHICALPLANESEGMENTATION_HPS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <utility>
#include "PlaneSeg.h"
#include "HirachicalIndex.h"
#include "PointsSumPyramid.h"
#include "PatchSegmentResult.h"
#include "utils.h"

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
    int *m_patch_sizes;

// 点云数据
    Eigen::MatrixXf m_cloud_array;
    Eigen::MatrixXf m_organized_pointcloud;

// 灰度图
    cv::Mat m_gray_img;

// 递归分割中间变量和结果
    PointsSumPyramid m_points_sum_pyramid;
    PatchSegmentResult m_patch_segment_result;

// 最终得到的平面参数，索引为分割的label
    PlaneParams *m_plane_params;

    void OrganizePointCloud();

    void RecursivePlaneSegment();

    void PatchwiseRegionGrowing();

    void RecursiveRegionGrowing(const vector<int> &seed_index, unsigned short label);

    Eigen::MatrixXf GetPatchPoints(const vector<int> &ind);

    void GetHirachicalIndexFromRowCol(int row, int col, int *hierachical_index);

    void GetHirachicalIndexFromRowCol(int row, int col, vector<int> hirachical_index);

    vector<int> GetHirachicalIndexFromRowColLevel(int row, int col, int level);

    vector<int> GetRowColLevelFromHirachicalIndex(vector<int> hirachical_index);

    int GetOffsetOfHirachicalIndex(const int *hirachical_index) {
        int begin = 0;
        for (int i = 0; i < m_num_tree_layers; i++) {
            begin += hirachical_index[i] * m_patch_sizes[i] * m_patch_sizes[i];
        }
        return begin;
    };

    int GetOffsetOfHirachicalIndex(const vector<int> hirachical_index) {
        int begin = 0;
        for (int i = 0; i < hirachical_index.size(); i++) {
            begin += hirachical_index[i] * m_patch_sizes[i] * m_patch_sizes[i];
        }
        return begin;
    };
public:
    HPS(int img_hight, int img_width, int block_num_x, int block_num_y, int tree_level);

    inline void SetPointCloud(Eigen::MatrixXf cloud_array) { m_cloud_array = cloud_array; }; //  一定要值传递，否则性能下降，未知原因，猜测与eigen内部实现有关

    inline void SetGrayImg(cv::Mat &img) { m_gray_img = img; };

    void Process();

    void DrawPatchByHierachicalIndex(const vector<int> &hierachical_index, cv::Mat &img_copy, cv::Scalar color,
                                     int thickness);

    void DrawPatchSegmentResult(cv::Mat &img_copy);

    ~HPS() {
        delete[] m_patchs_each_level;
        delete[] m_patch_sizes;
    };

    vector<vector<int>> GetAllPossibleSmallerNeighborPatchIndexs(const vector<int> &ind);

    vector<vector<int>> GetPossibleNeighborPatchIndexs(vector<int> ind, unsigned short level);

};


#endif //HIRACHICALPLANESEGMENTATION_HPS_H
