#include <iostream>
#include "PointsSumPyramid.h"
#include "utils.h"

using std::vector, std::max;


PointsSumPyramid::PointsSumPyramid() {
    this->m_num_layers = 0;
    this->m_num_elements_each_layer = nullptr;
    this->m_smallest_patch_width = 0;
    this->m_data = nullptr;
}

void PointsSumPyramid::Initialize(const int *num_childs_each_level, const int max_index_layers, const int smallest_patch_width) {
    this->m_num_layers = max_index_layers;
    this->m_num_elements_each_layer = new int[max_index_layers];
    this->m_smallest_patch_width = smallest_patch_width;

    int num_items_layer_i(1), total_size(0);

    for (int i = 0; i < max_index_layers; i++) {
        num_items_layer_i *= num_childs_each_level[i];
        m_num_elements_each_layer[i] = num_items_layer_i;
        total_size += num_items_layer_i;
    }
    m_data = new PointsSum[total_size];
}

PointsSum &PointsSumPyramid::Index(vector<int> ind){
    return Index(ind.data(), ind.size());
}

PointsSum &PointsSumPyramid::Index(const int *ind, int n){
    int offset = GetOffsetOfLayerN(n-1);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n-i-1);
    }
    return m_data[offset];
}
int PointsSumPyramid::GetOffsetOfLayerN(int n) {
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += m_num_elements_each_layer[i];
    }
    return offset;
}

void PointsSumPyramid::PreComputeSum(const Eigen::MatrixXf &cloud_array) {
    int offset_this_layer;
    int num_minimum_patches = m_num_elements_each_layer[m_num_layers - 1];
    int num_patch_points = cloud_array.rows() / num_minimum_patches;

    //    计算最小块的和，后续计算PointsSum结构体的和，不用重复计算了
    offset_this_layer = GetOffsetOfLayerN(m_num_layers - 1);

#pragma omp parallel for shared(cloud_array, offset_this_layer, num_minimum_patches, num_patch_points) default(none)
    for (int i = 0; i < num_minimum_patches; i++) {
        PreComputeSumOfArray(cloud_array.block(i * num_patch_points, 0, num_patch_points, 3),
                             m_data[offset_this_layer + i]);
    }

    for (int layer = m_num_layers - 2; layer >= 0; layer--) {
        int num_patches = m_num_elements_each_layer[layer];
        offset_this_layer = GetOffsetOfLayerN(layer);
        int offset_next_layer = GetOffsetOfLayerN(layer + 1);
        for (int i = 0; i < num_patches; i++) {
            m_data[offset_this_layer + i] = PointsSum::Add(m_data[offset_next_layer + 4 * i],
                                                           m_data[offset_next_layer + 4 * i + 1],
                                                           m_data[offset_next_layer + 4 * i + 2],
                                                           m_data[offset_next_layer + 4 * i + 3]);
        }
    }
}

void PointsSumPyramid::PreComputeSumOfArray(const Eigen::MatrixXf &cloud_array, PointsSum &points_sum_result) {
    Eigen::MatrixXf X_matrix, Y_matrix, Z_matrix;
    X_matrix = cloud_array.col(0);
    Y_matrix = cloud_array.col(1);
    Z_matrix = cloud_array.col(2);

    points_sum_result.num_points = (Z_matrix.array() > 0).count();
    points_sum_result.sum_x = X_matrix.sum();
    points_sum_result.sum_y = Y_matrix.sum();
    points_sum_result.sum_z = Z_matrix.sum();
    points_sum_result.sum_xx = (X_matrix.array() * X_matrix.array()).sum();
    points_sum_result.sum_yy = (Y_matrix.array() * Y_matrix.array()).sum();
    points_sum_result.sum_zz = (Z_matrix.array() * Z_matrix.array()).sum();
    points_sum_result.sum_xy = (X_matrix.array() * Y_matrix.array()).sum();
    points_sum_result.sum_xz = (X_matrix.array() * Z_matrix.array()).sum();
    points_sum_result.sum_yz = (Y_matrix.array() * Z_matrix.array()).sum();
    points_sum_result.jump_cnt = CountJumps(Z_matrix, 100);

}


int PointsSumPyramid::CountJumps(const Eigen::MatrixXf &Z_matrix, float max_diff) const {
    // Check for discontinuities using cross search
    int jumps_count = 0;
    int num_pts_per_cell = Z_matrix.rows();
    int i = m_smallest_patch_width * (m_smallest_patch_width / 2);
    int j = i + m_smallest_patch_width;
    float z, z_last(max(Z_matrix(i), Z_matrix(i + 1))); /* handles missing pixels on the borders*/
    i++;
    // Scan horizontally through the middle
    while (i < j) {
        z = Z_matrix(i);
        if (z > 0 && abs(z - z_last) < max_diff) {
            z_last = z;
        } else {
            if (z > 0)
                jumps_count++;
        }
        i++;
    }

    // Scan vertically through the middle
    i = m_smallest_patch_width / 2;
    j = num_pts_per_cell - i;
    z_last = max(Z_matrix(i), Z_matrix(i + m_smallest_patch_width));  /* handles missing pixels on the borders*/
    i += m_smallest_patch_width;
    while (i < j) {
        z = Z_matrix(i);
        if (z > 0 && abs(z - z_last) < max_diff) {
            z_last = z;
        } else {
            if (z > 0)
                jumps_count++;
        }
        i += m_smallest_patch_width;
    }
    return jumps_count;
}

PointsSumPyramid::~PointsSumPyramid() {
    delete[] m_num_elements_each_layer;
    delete[] m_data;
}