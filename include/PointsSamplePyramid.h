#ifndef HIERARCHICALPLANESEGMENTATION_POINTSSAMPLEPYRAMID_H
#define HIERARCHICALPLANESEGMENTATION_POINTSSAMPLEPYRAMID_H

// #define EIGEN_USE_MKL_ALL
#include "Params.h"

#include <Eigen/Dense>

using std::vector;

// A layer of the data sample pyramid
class PointsSamplePyramid_layer {
  public:
    int _height{}, _width{};
    Eigen::MatrixXf _data_X, _data_Y, _data_Z;
    PointsSamplePyramid_layer() = default;
    void SetData(const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z, int height, int width);
    void GetPointCloud(Eigen::MatrixXf &cloud_array);
};

// Data sample pyramid
class PointsSamplePyramid {
  private:
    vector<PointsSamplePyramid_layer> _layers;
    int _num_layers{}, _bottom_layer_height{}, _bottom_layer_width{}, _block_num_x{}, _block_num_y{};

  public:
    PointsSamplePyramid() = default;
    void Init(Params params);
    void SetData(const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z);
    void BuildPyramid();
    Eigen::Block<Eigen::MatrixXf> GetSampledPoints(int row, int col, int layer, int xyz, int patch_size);

    void Clear();
};

#endif // HIERARCHICALPLANESEGMENTATION_POINTSSAMPLEPYRAMID_H
