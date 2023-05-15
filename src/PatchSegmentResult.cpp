#include <cmath>
#include "PatchSegmentResult.h"
#include "PointsInfoPyramid.h"

PatchSegmentResult::PatchSegmentResult() {
    this->m_index_layers = 0;
    this->m_num_items = 0;
    this->m_offsets_each_level = nullptr;
    this->m_data_segment = nullptr;
    this->m_data_plane_params = nullptr;
}

void PatchSegmentResult::Initialize(const int *num_childs_each_level, const int index_layers) {
    this->m_index_layers = index_layers;
    this->m_num_items = 12 * (1<<(2*(index_layers-1)));   // 12*4^(index_layers-2)
    this->m_data_segment = new int[m_num_items];
    this->m_data_plane_params = new PlaneParams[m_num_items];

    this->m_offsets_each_level = new int[index_layers];
    for (int i = 0; i < index_layers; i++) {
        this->m_offsets_each_level[i] = 1<<(2*(index_layers-i-1));   // 4^(index_layers-i-1)
    }
}


PatchSegmentResult::~PatchSegmentResult() {
    delete[] m_data_segment;
    delete[] m_data_plane_params;
    delete[] m_offsets_each_level;
}

void PatchSegmentResult::Set(vector<int> ind, int value, PlaneParams plane_params) {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i);
    }
    m_data_segment[offset] = value;
    m_data_plane_params[offset] = plane_params;
}

int PatchSegmentResult::GetSegmentResult(vector<int> ind) const {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i);
    }
    return m_data_segment[offset];
}

PlaneParams PatchSegmentResult::GetPlaneParameter(vector<int> ind) const {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * pow(4, n - i);
    }
    return m_data_plane_params[offset];
}