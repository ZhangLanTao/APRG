#include "PointsSamplePyramid.h"

#include <chrono>
#include <iostream>

using std::cout, std::endl;

void PointsSamplePyramid::Init(Params params) {
    _num_layers          = params.num_pyramid_layers;
    _bottom_layer_height = params.img_height;
    _bottom_layer_width  = params.img_width;
    _block_num_x         = params.block_num_x;
    _block_num_y         = params.block_num_y;

    _layers         = vector<PointsSamplePyramid_layer>(_num_layers);
    int temp_height = params.img_height, temp_width = params.img_width;
    for (int i = 0; i < params.num_pyramid_layers; ++i) {
        _layers[i]._height = temp_height;
        _layers[i]._width  = temp_width;
        auto temp          = Eigen::MatrixXf(temp_height, temp_width);
        _layers[i]._data_X = Eigen::MatrixXf::Zero(temp_height, temp_width);
        _layers[i]._data_Y = Eigen::MatrixXf::Zero(temp_height, temp_width);
        _layers[i]._data_Z = Eigen::MatrixXf::Zero(temp_height, temp_width);
        temp_height /= 2;
        temp_width /= 2;
    }
}

void PointsSamplePyramid::SetData(const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z) {
    _layers[0].SetData(X, Y, Z, _bottom_layer_height, _bottom_layer_width);
}

void PointsSamplePyramid::BuildPyramid() {
    for (int i = 1; i < _num_layers; ++i) {
        for (int k = 0; k < _layers[i - 1]._width; k += 2) {
            for (int j = 0; j < _layers[i - 1]._height; j += 2) {
                _layers[i]._data_X(j / 2, k / 2) = _layers[i - 1]._data_X(j, k);
                _layers[i]._data_Y(j / 2, k / 2) = _layers[i - 1]._data_Y(j, k);
                _layers[i]._data_Z(j / 2, k / 2) = _layers[i - 1]._data_Z(j, k);
            }
        }
    }
}

Eigen::Block<Eigen::MatrixXf> PointsSamplePyramid::GetSampledPoints(int row, int col, int layer, int xyz,
                                                                    int patch_size) {
    switch (xyz) {
    case 0:
        return _layers[layer]._data_X.block(row, col, patch_size, patch_size);
    case 1:
        return _layers[layer]._data_Y.block(row, col, patch_size, patch_size);
    case 2:
        return _layers[layer]._data_Z.block(row, col, patch_size, patch_size);
    default:
        cout << "Error: xyz should be 0, 1 or 2" << endl;
        throw std::runtime_error("GetSampledPoints Index error");
    }
}

void PointsSamplePyramid::Clear() {
    for (int i = 0; i < _num_layers; ++i) {
        _layers[i]._data_X.setZero();
        _layers[i]._data_Y.setZero();
        _layers[i]._data_Z.setZero();
    }
}

void PointsSamplePyramid_layer::SetData(const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z,
                                        int height, int width) {
    _height = height;
    _width  = width;
    _data_X = X;
    _data_Y = Y;
    _data_Z = Z;
}

// Get the point cloud in the layer
void PointsSamplePyramid_layer::GetPointCloud(Eigen::MatrixXf &cloud_array) {
    cloud_array.resize(_height * _width, 3);
    for (int i = 0; i < _height; ++i) {
        for (int j = 0; j < _width; ++j) {
            cloud_array(i * _width + j, 0) = _data_X(i, j);
            cloud_array(i * _width + j, 1) = _data_Y(i, j);
            cloud_array(i * _width + j, 2) = _data_Z(i, j);
        }
    }
}
