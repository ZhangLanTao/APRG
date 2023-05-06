#ifndef HIRACHICALPLANESEGMENTATION_POINTSINFOPYRAMID_H
#define HIRACHICALPLANESEGMENTATION_POINTSINFOPYRAMID_H

#include<Eigen/Dense>
#include<vector>

using std::vector;

struct PlaneParams {
    double a,b,c,d,MSE,score;
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

    static PointsSum Add(const PointsSum &a, const PointsSum &b, const PointsSum &c, const PointsSum &d);
    bool IsPlane();
    PlaneParams FitPlane();

};



class PointsInfoPyramid {
private:
    int m_num_layers;
    int *m_num_elements_each_layer;
    int m_smallest_patch_width;
    PointsSum *m_data;
    void PreComputeSumOfArray(const Eigen::MatrixXf &cloud_array, PointsSum &points_sum_result);

    int CountJumps(const Eigen::MatrixXf &Z_matrix, float max_diff) const;
public:
    PointsInfoPyramid();
    ~PointsInfoPyramid();
    void Initialize(const int *num_childs_each_level, int max_index_layers, int smallest_patch_width);
    void PreComputeSum(const Eigen::MatrixXf &cloud_array);
    int GetOffsetOfLayerN(int n);

    PointsSum &Index(const int ind[], int n);
    PointsSum &Index(vector<int> ind);
};


#endif //HIRACHICALPLANESEGMENTATION_POINTSINFOPYRAMID_H