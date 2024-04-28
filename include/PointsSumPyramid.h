#ifndef HIRACHICALPLANESEGMENTATION_POINTSSUMPYRAMID_H
#define HIRACHICALPLANESEGMENTATION_POINTSSUMPYRAMID_H

//#define EIGEN_USE_MKL_ALL
#include<Eigen/Dense>
#include<vector>
#include "utils.h"

using std::vector;



class PointsSumPyramid {
private:
    int m_num_layers;
    int *m_num_elements_each_layer;
    int m_smallest_patch_width;
    PointsSum *m_data;
    void PreComputeSumOfArray(const Eigen::MatrixXd &cloud_array, PointsSum &points_sum_result);

    int CountJumps(const Eigen::MatrixXd &Z_matrix, float max_diff) const;
public:
    PointsSumPyramid();
    ~PointsSumPyramid();
    void Initialize(const int *num_childs_each_level, int max_index_layers, int smallest_patch_width);
    void PreComputeSum(const Eigen::MatrixXd &cloud_array);
    int GetOffsetOfLayerN(int n);

    PointsSum &Index(const int ind[], int n);
    PointsSum &Index(vector<int> ind);
};


#endif //HIRACHICALPLANESEGMENTATION_POINTSSUMPYRAMID_H
