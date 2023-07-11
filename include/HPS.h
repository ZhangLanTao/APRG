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
    uint16_t m_img_height;
    uint16_t m_img_width;

// 图像递归分割相关
    uint8_t m_block_num_x;  // 首先将整个图像分成 m_block_num_x * m_block_num_y 个正方形块
    uint8_t m_block_num_y;
    uint8_t m_num_tree_layers; // 递归分割的层数
    uint8_t *m_patches_each_level;

// 每层block的尺寸
    uint16_t *m_patch_sizes;

// 点云数据
    Eigen::MatrixXf m_cloud_array;
    Eigen::MatrixXf m_organized_pointcloud;

// 灰度图
    cv::Mat m_gray_img;

// 递归分割中间变量和结果
    PointsSumPyramid m_points_sum_pyramid;
    PatchSegmentResult m_patch_segment_result;
//    vector<vector<vector<short>>> m_sorted_patch_indexes; // 每个patch的索引，按照patch的大小排序，同等大小的patch之间无序

// 最终得到的平面参数，索引为分割的label
    PlaneParams *m_plane_params;

    void OrganizePointCloud();

    void RecursivePlaneSegment();

    void PatchwiseRegionGrowing();

    void RecursiveRegionGrowing(const vector<uint8_t> &seed_index, uint16_t label);

    Eigen::MatrixXf GetPatchPoints(const vector<uint8_t> &ind);

    void GetHierarchicalIndexFromRowCol(uint16_t row, uint16_t col, uint8_t *hierarchical_index);

    void GetHierarchicalIndexFromRowCol(uint16_t row, uint16_t col, vector<uint8_t> hierarchical_index);

    vector<uint8_t> GetHierarchicalIndexFromRowColLevel(uint16_t row, uint16_t col, uint8_t level);

    vector<uint16_t> GetRowColLevelFromHierarchicalIndex(vector<uint8_t> hierarchical_index);

    int GetOffsetOfHierarchicalIndex(const uint8_t *hierarchical_index) {
        int begin = 0;
        for (int i = 0; i < m_num_tree_layers; i++) {
            begin += hierarchical_index[i] * m_patch_sizes[i] * m_patch_sizes[i];
        }
        return begin;
    };

    int GetOffsetOfHierarchicalIndex(const vector<uint8_t> hierarchical_index) {
        int begin = 0;
        for (uint8_t i = 0; i < hierarchical_index.size(); i++) {
            begin += hierarchical_index[i] * m_patch_sizes[i] * m_patch_sizes[i];
        }
        return begin;
    };
public:
    HPS(uint16_t img_height, uint16_t img_width, uint8_t block_num_x, uint8_t block_num_y, uint8_t tree_level);

    inline void SetPointCloud(Eigen::MatrixXf cloud_array) { m_cloud_array = cloud_array; }; //  一定要值传递，否则性能下降，未知原因，猜测与eigen内部实现有关

    inline void SetGrayImg(cv::Mat &img) { m_gray_img = img; };

    void Process();

    void DrawPatchByHierarchicalIndex(const vector<uint8_t> &hierachical_index, cv::Mat &img_copy, cv::Scalar color,
                                     int thickness);

    void DrawPatchSegmentResult(cv::Mat &img_copy);

    ~HPS() {
        delete[] m_patches_each_level;
        delete[] m_patch_sizes;
    };

    vector<vector<uint8_t>> GetAllPossibleSmallerNeighborPatchIndexes(const vector<uint8_t> &ind);

    vector<vector<uint8_t>> GetPossibleNeighborPatchIndexes(const vector<uint8_t>& ind, uint8_t level);

};


#endif //HIRACHICALPLANESEGMENTATION_HPS_H
