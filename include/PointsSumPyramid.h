#ifndef HIRACHICALPLANESEGMENTATION_POINTSSUMPYRAMID_H
#define HIRACHICALPLANESEGMENTATION_POINTSSUMPYRAMID_H

//#define EIGEN_USE_MKL_ALL
#include<Eigen/Dense>
#include<vector>
#include "utils.h"

using std::vector;



class PointsSumPyramid {
private:
    uint8_t m_num_layers;
    int *m_num_elements_each_layer;
    int m_smallest_patch_width;
    PointsSum *m_data;
    void PreComputeSumOfArray(const Eigen::MatrixXf &cloud_array, PointsSum &points_sum_result);

    int CountJumps(const Eigen::MatrixXf &Z_matrix, float max_diff) const;
public:
    PointsSumPyramid();
    ~PointsSumPyramid();
    void Initialize(const uint8_t *num_childs_each_level, uint8_t max_index_layers, uint8_t smallest_patch_width);
    void PreComputeSum(const Eigen::MatrixXf &cloud_array);
    int GetOffsetOfLayerN(int n);

    PointsSum &Index(const uint8_t ind[], uint8_t n);
    PointsSum &Index(vector<uint8_t> ind);
};


#endif //HIRACHICALPLANESEGMENTATION_POINTSSUMPYRAMID_H
