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


void PatchSegmentResult::Initialize(const uint8_t *num_childs_each_level, const uint8_t index_layers) {
    this->m_index_layers = index_layers;
    uint16_t num(1);
    for (uint8_t i = 0; i < index_layers; i++) num *= num_childs_each_level[i];
    this->m_num_smallest_patches = num;
    this->m_data_patch_level = new uint8_t[m_num_smallest_patches]{};
    this->m_data_patch_label = new uint16_t[m_num_smallest_patches]{};
    this->m_data_plane_params = new PlaneParams[m_num_smallest_patches];
    this->m_data_patch_visited = new bool[m_num_smallest_patches]{};

    this->m_offsets_each_level = new uint16_t[index_layers];
    for (uint8_t i = 0; i < index_layers; i++) {
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

int PatchSegmentResult::GetOffsetOfHIndex(const vector<uint8_t> &ind) const {
    uint8_t n = ind.size();
    uint16_t offset(0);
    for (uint8_t i = 0; i < n; i++) {
        offset += ind[i] * m_offsets_each_level[i];
    }
    return offset;
}

void PatchSegmentResult::SetPatchSegmentResult(const vector<uint8_t>& ind, uint8_t value, PlaneParams plane_params) {
    uint16_t offset = GetOffsetOfHIndex(ind);
    m_data_patch_level[offset] = value;
    m_data_plane_params[offset] = plane_params;
}

void PatchSegmentResult::SetPatchLabel(const vector<uint8_t>& ind, uint16_t label) {
    uint16_t offset = GetOffsetOfHIndex(ind);
    m_data_patch_label[offset] = label;
}

int PatchSegmentResult::GetPatchLabel(const vector<uint8_t>& ind) {
    uint16_t offset = GetOffsetOfHIndex(ind);
    return m_data_patch_label[offset];
}

bool PatchSegmentResult::IsLabeled(const vector<uint8_t>& ind) {
    int offset = GetOffsetOfHIndex(ind);
    return m_data_patch_label[offset] != 0;
}

uint8_t PatchSegmentResult::GetPatchSegmentLevel(const vector<uint8_t>& ind) const {
    uint16_t offset = GetOffsetOfHIndex(ind);
    return m_data_patch_level[offset];
}

PlaneParams PatchSegmentResult::GetPatchPlaneParameter(const vector<uint8_t>& ind) const {
    uint16_t offset = GetOffsetOfHIndex(ind);
    return m_data_plane_params[offset];
}
// 真实的寻找最大块函数，效率不高，意义不大，暂时不用
//vector<int> PatchSegmentResult::GetRemainingLargestPatchIndex() const {
//    int largest_patch_level = 999;
//    int largest_patch_offset = -1;
//    for (int i = 0; i < m_num_smallest_patches; i++) {
//        if (m_data_patch_level[i] == 0) continue;
//        if (m_data_patch_label[i]) continue;
//        if (m_data_patch_level[i] < largest_patch_level) {
//            largest_patch_level = m_data_patch_level[i];
//            largest_patch_offset = i;
//        }
//    }
//    if (largest_patch_offset == -1) return {};
//    else return GetHierarchicalIndexFromOffsetAndLevel(largest_patch_offset, largest_patch_level);
//}


// 由于找最大块作为种子点的作用不大，所以暂时随便找一个未分类的块就作为种子点，待继续优化
vector<uint8_t> PatchSegmentResult::GetRemainingLargestPatchIndex(int& start_from_i) const {
    int largest_patch_offset = -1;
    int largest_patch_level = 0;
    for (uint16_t i = start_from_i; i < m_num_smallest_patches; i++) {
        if (m_data_patch_level[i] == 0) continue;
        if (m_data_patch_label[i]) continue;

        largest_patch_level = m_data_patch_level[i];
        largest_patch_offset = i;
        start_from_i = i+1;
        break;
    }

    if (largest_patch_offset == -1) return {};
    else return GetHierarchicalIndexFromOffsetAndLevel(largest_patch_offset, largest_patch_level);
}

vector<uint8_t> PatchSegmentResult::GetHierarchicalIndexFromOffsetAndLevel(uint16_t offset, unsigned short level) const {
    vector<uint8_t> ind(level);
    for (int i = 0; i < level; i++) {
        ind[i] = offset / m_offsets_each_level[i];
        offset = offset % m_offsets_each_level[i];
    }
    return ind;
}

void PatchSegmentResult::SetPatchVisited(vector<uint8_t> ind) {
    uint16_t offset = GetOffsetOfHIndex(ind);
    m_data_patch_visited[offset] = true;
}

void PatchSegmentResult::ResetVisited() {
    memset(m_data_patch_visited,false, sizeof(m_data_patch_visited)/sizeof(bool));
}

bool PatchSegmentResult::IsVisited(const vector<uint8_t>& ind) const {
    uint16_t offset = GetOffsetOfHIndex(ind);
    return m_data_patch_visited[offset];
}

vector<uint8_t> PatchSegmentResult::GetMostSimilarPatchIndex(vector<uint8_t> ind) const{
    for (int i = 0; i < ind.size(); ++i) {
        if (Has(vector<uint8_t>(ind.begin(), ind.begin()+i+1)))
            return {ind.begin(), ind.begin()+i+1};
    }
    return {};
}

