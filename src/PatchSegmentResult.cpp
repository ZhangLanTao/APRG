#include <cmath>
#include "PatchSegmentResult.h"
#include "PointsInfoPyramid.h"

PatchSegmentResult::PatchSegmentResult() {
    this->m_index_layers = 0;
    this->m_num_smallest_patches = 0;
    this->m_offsets_each_level = nullptr;
    this->m_data_patch_level = nullptr;
    this->m_data_patch_label = nullptr;
    this->m_data_plane_params = nullptr;
}

PatchSegmentResult::PatchSegmentResult(PatchSegmentResult &patch_segment_result) {
    this->m_index_layers = patch_segment_result.m_index_layers;
    this->m_num_smallest_patches = patch_segment_result.m_num_smallest_patches;
    this->m_offsets_each_level = new int[m_index_layers];
    this->m_data_patch_level = new unsigned char [m_num_smallest_patches];
    this->m_data_patch_label = new unsigned short [m_num_smallest_patches];
    this->m_data_plane_params = new PlaneParams[m_num_smallest_patches];
    for (int i = 0; i < m_index_layers; i++) {
        this->m_offsets_each_level[i] = patch_segment_result.m_offsets_each_level[i];
    }
    for (int i = 0; i < m_num_smallest_patches; i++) {
        this->m_data_patch_level[i] = patch_segment_result.m_data_patch_level[i];
        this->m_data_patch_label[i] = patch_segment_result.m_data_patch_label[i];
        this->m_data_plane_params[i] = patch_segment_result.m_data_plane_params[i];
    }
}

void PatchSegmentResult::Initialize(const int *num_childs_each_level, const int index_layers) {
    this->m_index_layers = index_layers;
    this->m_num_smallest_patches = 12 * (1 << (2 * (index_layers - 1)));   // 这里固定死了num_childs_each_level为{12，4，4，4，...}，所以计算式子是固定的 12*4^(index_layers-2)
    this->m_data_patch_level = new unsigned char [m_num_smallest_patches]{};
    this->m_data_patch_label = new unsigned short [m_num_smallest_patches]{};
    this->m_data_plane_params = new PlaneParams[m_num_smallest_patches];

    this->m_offsets_each_level = new int[index_layers];
    for (int i = 0; i < index_layers; i++) {
        this->m_offsets_each_level[i] = 1<<(2*(index_layers-i-1));   // 4^(index_layers-i-1)
    }
}


PatchSegmentResult::~PatchSegmentResult() {
    delete[] m_data_patch_level;
    delete[] m_data_patch_label;
    delete[] m_data_plane_params;
    delete[] m_offsets_each_level;
}

void PatchSegmentResult::SetPatchSegmentResult(vector<int> ind, int value, PlaneParams plane_params) {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i - 1);
    }
    m_data_patch_level[offset] = value;
    m_data_plane_params[offset] = plane_params;
}

void PatchSegmentResult::SetPatchLabel(vector<int> ind, int label) {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i);
    }
    m_data_patch_label[offset] = label;
}

int PatchSegmentResult::GetPatchSegmentResult(vector<int> ind) const {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i);
    }
    return m_data_patch_level[offset];
}

PlaneParams PatchSegmentResult::GetPatchPlaneParameter(vector<int> ind) const {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i);
    }
    return m_data_plane_params[offset];
}