#include <iostream>
#include "PointsInfoPyramid.h"
#include "utils.h"

using std::vector, std::max;


PointsInfoPyramid::PointsInfoPyramid() {
    this->m_num_layers = 0;
    this->m_num_elements_each_layer = nullptr;
    this->m_smallest_patch_width = 0;
    this->m_data = nullptr;
}

void PointsInfoPyramid::Initialize(const int *num_childs_each_level, const int max_index_layers, const int smallest_patch_width) {
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

PointsSum &PointsInfoPyramid::Index(vector<int> ind){
    return Index(ind.data(), ind.size());
}

PointsSum &PointsInfoPyramid::Index(const int *ind, int n){
    int offset = GetOffsetOfLayerN(n-1);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n-i-1);
    }
    return m_data[offset];
}
int PointsInfoPyramid::GetOffsetOfLayerN(int n) {
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += m_num_elements_each_layer[i];
    }
    return offset;
}

void PointsInfoPyramid::PreComputeSum(const Eigen::MatrixXf &cloud_array) {
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

void PointsInfoPyramid::PreComputeSumOfArray(const Eigen::MatrixXf &cloud_array, PointsSum &points_sum_result) {
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


int PointsInfoPyramid::CountJumps(const Eigen::MatrixXf &Z_matrix, float max_diff) const {
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

PointsSum PointsSum::Add(const PointsSum &a, const PointsSum &b, const PointsSum &c, const PointsSum &d) {
    PointsSum result;
    result.sum_x = a.sum_x + b.sum_x + c.sum_x + d.sum_x;
    result.sum_y = a.sum_y + b.sum_y + c.sum_y + d.sum_y;
    result.sum_z = a.sum_z + b.sum_z + c.sum_z + d.sum_z;
    result.sum_xx = a.sum_xx + b.sum_xx + c.sum_xx + d.sum_xx;
    result.sum_yy = a.sum_yy + b.sum_yy + c.sum_yy + d.sum_yy;
    result.sum_zz = a.sum_zz + b.sum_zz + c.sum_zz + d.sum_zz;
    result.sum_xy = a.sum_xy + b.sum_xy + c.sum_xy + d.sum_xy;
    result.sum_xz = a.sum_xz + b.sum_xz + c.sum_xz + d.sum_xz;
    result.sum_yz = a.sum_yz + b.sum_yz + c.sum_yz + d.sum_yz;
    result.num_points = a.num_points + b.num_points + c.num_points + d.num_points;
    result.jump_cnt = a.jump_cnt + b.jump_cnt + c.jump_cnt + d.jump_cnt;
    return result;
}

PlaneParams PointsSum::FitPlane() const {
    if (this->jump_cnt > 2) return {};
    if (this->num_points < 50) return {};

    double mean[3];
    mean[0] = sum_x / num_points;
    mean[1] = sum_y / num_points;
    mean[2] = sum_z / num_points;
    // Expressing covariance as E[PP^t] + E[P]*E[P^T]
    double cov[3][3] = {
            {sum_xx - sum_x * sum_x / num_points, sum_xy - sum_x * sum_y / num_points, sum_xz - sum_x * sum_z / num_points},
            {0,                                   sum_yy - sum_y * sum_y / num_points, sum_yz - sum_y * sum_z / num_points},
            {0,                                   0,                                   sum_zz - sum_z * sum_z / num_points}
    };
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];

    // This uses QR decomposition for symmetric matrices
    Eigen::Map<Eigen::Matrix3d, 0, Eigen::Stride<0, 0>> cov_eigen = Eigen::Map<Eigen::Matrix3d>(cov[0], 3, 3);
//    cov_eigen = cov_eigen / num_points;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_eigen);
    Eigen::VectorXd v = es.eigenvectors().col(0);

    double d, normal[3];
    d = -(v[0] * mean[0] + v[1] * mean[1] + v[2] * mean[2]);
    // Enforce normal orientation
    if (d > 0) {
        normal[0] = v[0];
        normal[1] = v[1];
        normal[2] = v[2];
    } else {
        normal[0] = -v[0];
        normal[1] = -v[1];
        normal[2] = -v[2];
        d = -d;
    }
    double min_ev = es.eigenvalues()[0];
    if (min_ev < 15) min_ev = 15;     // eigen 精度问题，最小值可能小于0，这里根据经验把最小值设为15

    double MSE, score;
    MSE = min_ev / num_points;
    score = es.eigenvalues()[1] / min_ev;
    return PlaneParams{normal[0], normal[1], normal[2], d, MSE, score};
}





PointsInfoPyramid::~PointsInfoPyramid() {
    delete[] m_num_elements_each_layer;
    delete[] m_data;
}