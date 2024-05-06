#ifndef HIRACHICALPLANESEGMENTATION_HPS_H
#define HIRACHICALPLANESEGMENTATION_HPS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <utility>
#include "HirachicalIndex.h"
#include "PointsSumPyramid.h"
#include "PatchSegmentStatus.h"
#include "utils.h"

#define TRUE

class HPS {
private:
// 点云数据
    Eigen::MatrixXf _cloud_array;
    Eigen::MatrixXf _organized_pointcloud;

// 图像
    cv::Mat _color_img;
    cv::Mat _d_img;
    double _fx, _fy, _cx, _cy, _depth_scale, _z_min;
    int _img_height, _img_width;

// 数据排序相关，首先存好表格，实际排序时直接用索引
    int *_organize_data_cell_map;

// 图像递归分割相关
    int _block_num_x, _block_num_y;     // 首先将整个图像分成 _block_num_x * _block_num_y 个正方形块
    int _num_tree_layers;               // 递归分割的层数
    int *_num_patches_each_level;
    int *_patch_sizes;                  // 每层block的尺寸
    int _plane_thresh_mode;             // 递归分割的阈值模式
    double _plane_thresh;               // 递归分割的阈值

// 递归分割中间变量和结果
    PointsSumPyramid _points_sum_pyramid;
    PatchSegmentStatus _patch_segment_status;

// 区域生长参数
    bool _RG_filter_by_area;
    int _RG_min_area;
    int _plane_merge_thresh_mode=PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE;
    double _plane_merge_min_cos_angle = cos(M_PI / 18);
    double _plane_merge_max_distance2_thresh = 20;

// 像素细分参数
    int _merge_plane_NN_window_size = 5;
    double _point_coplane_thresh=30;

// 最终得到的平面参数，索引为分割的label
    vector<PlaneParams> _region_plane_params;
    vector<vector<vector<int>>> _region_indexes;
// 最终的分割label
    int _region_count;
    cv::Mat _min_patch_label_grid;
    cv::Mat _min_patch_label_grid_eroded;
    cv::Mat _final_pixel_label;

    cv::Mat _debug_color_label_lut;

    cv::Mat _change_label_lut;      // 合并、修改label时的映射表

    void InitOrganizeDataCellMap();

    void OrganizePointCloud();

    void RecursivePlaneSegment(bool debug);

    void PatchwiseRegionGrowing(bool debug);

    void RecursiveRegionGrowing(const vector<int> &seed_index, vector<vector<int>> &region_indexes);

    Eigen::MatrixXf GetPatchPoints(const vector<int> &ind);

    void GetHierarchicalIndexFromRowCol(int row, int col, int *hierarchical_index);

    void GetHierarchicalIndexFromRowCol(int row, int col, vector<int> &hierarchical_index);

    vector<int> GetHierarchicalIndexFromRowColLevel(int row, int col, int level);

    vector<int> GetRowColLevelFromHierarchicalIndex(vector<int> hierarchical_index);

    int GetOffsetOfHierarchicalIndex(const int *hierarchical_index) {
        int begin = 0;
        for (int i = 0; i < _num_tree_layers; i++) {
            begin += hierarchical_index[i] * _patch_sizes[i] * _patch_sizes[i];
        }
        return begin;
    };

    int GetOffsetOfHierarchicalIndex(const vector<int> hierarchical_index) {
        int begin = 0;
        for (int i = 0; i < hierarchical_index.size(); i++) {
            begin += hierarchical_index[i] * _patch_sizes[i] * _patch_sizes[i];
        }
        return begin;
    };

    vector<vector<int>> GetAllPossibleSmallerNeighborPatchIndexes(const vector<int> &ind);

    vector<vector<int>> GetPossibleNeighborPatchIndexes(const vector<int>& ind, int level);

    void BoundaryRefine(bool debug);

    PlaneParams GetRegionPlaneParameter(const vector<vector<int>>& indexes);

    void ConvertPatchSegmentStatusToGrid();

    vector<int> FindLabelsInNNeighbor(int row, int col, int num_neighbor);

    void RecordMergePlane(vector<int> labels);


public:
    HPS(int img_height, int img_width, int block_num_x, int block_num_y, int tree_level);

    void SetPlaneThresh(int thresh_mode, double thresh) { _plane_thresh_mode=thresh_mode; _plane_thresh = thresh; };

    void SetMergePlaneParams(int merge_NN_window_size) {_merge_plane_NN_window_size = merge_NN_window_size; };

    void SetPointCoplaneThresh(double point_coplane_thresh) { _point_coplane_thresh = point_coplane_thresh; };

    void SetCoplanarParams(double min_cos_angle, int plane_merge_thresh_mode, double max_distance2_merge) {
        _plane_merge_min_cos_angle = min_cos_angle;
        _plane_merge_thresh_mode = plane_merge_thresh_mode;
        _plane_merge_max_distance2_thresh = max_distance2_merge;
    };

    inline void SetPointCloud(Eigen::MatrixXf array) { _cloud_array = array;} //  一定要值传递，否则性能下降，未知原因，猜测与eigen内部实现有关

    inline void SetImg(cv::Mat &color_img, cv::Mat &d_img, double fx, double fy, double cx, double cy, double depth_scale, double z_min= 50) {
        _color_img = color_img;
        _d_img = d_img;
        _fx = fx;
        _fy = fy;
        _cx = cx;
        _cy = cy;
        _depth_scale = depth_scale;
        _z_min = z_min;
    };

    void Process();

    void GetResult(cv::Mat &result){ result = _final_pixel_label.clone();}

    void GetResultReadable(cv::Mat &result){
        result = _final_pixel_label.clone();
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
        cv::LUT(result, _debug_color_label_lut, result);
    }

    void DrawPatchByHierarchicalIndex(const vector<int> &hierachical_index, cv::Mat &image, const cv::Scalar& color,
                                     int thickness);

    void DrawPatchByHierarchicalIndex(const vector<vector<int>> &hierarchical_index, cv::Mat &image, const cv::Scalar &color,
                                      int thickness);

    void DrawPatchSegmentResult(cv::Mat &img_copy);

    void RefinePixelsAndMergePlane(const cv::Mat &mask_to_refine);

    void GetCoplanarPointsMask(Eigen::MatrixXd points, int plane_label, bool mask[]);

    inline void SetPatchwiseLabel(cv::Mat mat);

    int GetRegionArea(vector<vector<int>> indexes){
        int area = 0;
        for(auto i : indexes){
            area += 1<<(_num_tree_layers / i.size() - 1);
        }
        return area;
    };

    void SetMinAreaOfRG(int min_area){
        _RG_filter_by_area = true;
        _RG_min_area = min_area;
    };

    ~HPS() {
        delete[] _num_patches_each_level;
        delete[] _patch_sizes;
    };

    void SetPixelLabel(int label, int row, int col, bool mask[]);

    void ChangeLabel(int ori_label, int new_label);
};


#endif //HIRACHICALPLANESEGMENTATION_HPS_H
