#include <cmath>
#include "PatchSegmentResult.h"
#include "PointsSumPyramid.h"

PatchSegmentResult::PatchSegmentResult() {
    this->m_index_layers = 0;
    this->m_num_smallest_patches = 0;
    this->m_offsets_each_level = nullptr;
    this->m_data_patch_level = nullptr;
    this->m_data_patch_label = nullptr;
    this->m_data_plane_params = nullptr;
    this->m_data_patch_visited = nullptr;
}

PatchSegmentResult::PatchSegmentResult(PatchSegmentResult &patch_segment_result) {
    this->m_index_layers = patch_segment_result.m_index_layers;
    this->m_num_smallest_patches = patch_segment_result.m_num_smallest_patches;
    this->m_offsets_each_level = new int[m_index_layers];
    this->m_data_patch_level = new unsigned short [m_num_smallest_patches];
    this->m_data_patch_label = new unsigned short [m_num_smallest_patches];
    this->m_data_plane_params = new PlaneParams[m_num_smallest_patches];
    this->m_data_patch_visited = new bool [m_num_smallest_patches];
    for (int i = 0; i < m_index_layers; i++) {
        this->m_offsets_each_level[i] = patch_segment_result.m_offsets_each_level[i];
    }
    for (int i = 0; i < m_num_smallest_patches; i++) {
        this->m_data_patch_level[i] = patch_segment_result.m_data_patch_level[i];
        this->m_data_patch_label[i] = patch_segment_result.m_data_patch_label[i];
        this->m_data_plane_params[i] = patch_segment_result.m_data_plane_params[i];
        this->m_data_patch_visited[i] = patch_segment_result.m_data_patch_visited[i];
    }
}

void PatchSegmentResult::Initialize(const int *num_childs_each_level, const int index_layers) {
    this->m_index_layers = index_layers;
    int num(1);
    for (int i = 0; i < index_layers; i++) num *= num_childs_each_level[i];
    this->m_num_smallest_patches = num;
    this->m_data_patch_level = new unsigned short [m_num_smallest_patches]{};
    this->m_data_patch_label = new unsigned short [m_num_smallest_patches]{};
    this->m_data_plane_params = new PlaneParams[m_num_smallest_patches];
    this->m_data_patch_visited = new bool[m_num_smallest_patches]{};

    this->m_offsets_each_level = new int[index_layers];
    for (int i = 0; i < index_layers; i++) {
        this->m_offsets_each_level[i] = 1<<(2*(index_layers-i-1));   // 4^(index_layers-i-1)
    }
}


PatchSegmentResult::~PatchSegmentResult() {
    delete[] m_offsets_each_level;
    delete[] m_data_patch_level;
    delete[] m_data_patch_label;
    delete[] m_data_plane_params;
    delete[] m_data_patch_visited;
}

int PatchSegmentResult::GetOffsetOfHIndex(const vector<int> &ind) const {
    int n = ind.size();
    int offset(0);
    for (int i = 0; i < n; i++) {
        offset += ind[i] * m_offsets_each_level[i];
    }
    return offset;
}

void PatchSegmentResult::SetPatchSegmentResult(vector<int> ind, int value, PlaneParams plane_params) {
    int offset = GetOffsetOfHIndex(ind);
    m_data_patch_level[offset] = value;
    m_data_plane_params[offset] = plane_params;
}

void PatchSegmentResult::SetPatchLabel(vector<int> ind, int label) {
    int offset = GetOffsetOfHIndex(ind);
    m_data_patch_label[offset] = label;
}

int PatchSegmentResult::GetPatchLabel(vector<int> ind) {
    int offset = GetOffsetOfHIndex(ind);
    return m_data_patch_label[offset];
}

bool PatchSegmentResult::IsLabeled(vector<int> ind) {
    int offset = GetOffsetOfHIndex(ind);
    return m_data_patch_label[offset] != 0;
}

unsigned short PatchSegmentResult::GetPatchSegmentLevel(vector<int> ind) const {
    int offset = GetOffsetOfHIndex(ind);
    return m_data_patch_level[offset];
}

PlaneParams PatchSegmentResult::GetPatchPlaneParameter(vector<int> ind) const {
    int offset = GetOffsetOfHIndex(ind);
    return m_data_plane_params[offset];
}

vector<int> PatchSegmentResult::GetRemainingLargestPatchIndex() const {
    int largest_patch_level = 999;
    int largest_patch_offset = -1;
    for (int i = 0; i < m_num_smallest_patches; i++) {
        if (m_data_patch_level[i] == 0) continue;
        if (m_data_patch_label[i]) continue;
        if (m_data_patch_level[i] < largest_patch_level) {
            largest_patch_level = m_data_patch_level[i];
            largest_patch_offset = i;
        }
    }
    if (largest_patch_offset == -1) return {};
    else return GetHierarchicalIndexFromOffsetAndLevel(largest_patch_offset, largest_patch_level);
}

vector<int> PatchSegmentResult::GetHierarchicalIndexFromOffsetAndLevel(int offset, unsigned short level) const {
    vector<int> ind(level);
    for (int i = 0; i < level; i++) {
        ind[i] = offset / m_offsets_each_level[i];
        offset = offset % m_offsets_each_level[i];
    }
    return ind;
}

void PatchSegmentResult::SetPatchVisited(vector<int> ind) {
    int offset = GetOffsetOfHIndex(ind);
    m_data_patch_visited[offset] = true;
}

void PatchSegmentResult::ResetVisited() {
    memset(m_data_patch_visited,false, sizeof(m_data_patch_visited)/sizeof(bool));
}

bool PatchSegmentResult::IsVisited(vector<int> ind) const {
    int offset = GetOffsetOfHIndex(ind);
    return m_data_patch_visited[offset];
}

vector<int> PatchSegmentResult::GetMostSimilarPatchIndex(vector<int> ind) const{
    for (int i = 0; i < ind.size(); ++i) {
        if (Has(vector<int>(ind.begin(), ind.begin()+i+1)))
            return vector<int>(ind.begin(), ind.begin()+i+1);
    }
    return vector<int>();
}

