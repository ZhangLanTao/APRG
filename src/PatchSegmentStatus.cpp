#include <cmath>
#include "PatchSegmentStatus.h"
#include "PointsSumPyramid.h"

PatchSegmentStatus::PatchSegmentStatus() {
    this->num_layers = 0;
    this->num_smallest_patches = 0;
    this->offsets_each_level = nullptr;
    this->data_patch_level = nullptr;
    this->data_patch_label = nullptr;
    this->data_plane_params = nullptr;
    this->data_patch_visited = nullptr;
}


void PatchSegmentStatus::Initialize(const int *num_childs_each_level, const int index_layers) {
    this->num_layers = index_layers;
    int num(1);
    for (int i = 0; i < index_layers; i++) num *= num_childs_each_level[i];
    this->num_smallest_patches = num;
    this->data_patch_level = new int[num_smallest_patches]{};
    this->data_patch_label = new int[num_smallest_patches]{};
    this->data_plane_params = new PlaneParams[num_smallest_patches];
    this->data_patch_visited = new bool[num_smallest_patches]{};

    this->unlabeled_patch_index_grouped_by_level_smallest_first.resize(index_layers);
    for (int i = 0; i < index_layers; i++) {
        this->unlabeled_patch_index_grouped_by_level_smallest_first[i].reserve(num_smallest_patches);
    }

    this->offsets_each_level = new int[index_layers];
    for (int i = 0; i < index_layers; i++) {
        this->offsets_each_level[i] = 1 << (2 * (index_layers - i - 1));   // 4^(index_layers-i-1)
    }
}


PatchSegmentStatus::~PatchSegmentStatus() {
    delete[] offsets_each_level;
    delete[] data_patch_level;
    delete[] data_patch_label;
    delete[] data_plane_params;
    delete[] data_patch_visited;
}

int PatchSegmentStatus::GetOffsetOfHIndex(const vector<int> &ind) const {
    int offset(0);
    for (int i = 0; i < ind.size(); i++) {
        offset += ind[i] * offsets_each_level[i];
    }
    return offset;
}

// 真实的寻找最大块函数，效率不高，意义不大，暂时不用
//vector<int> PatchSegmentStatus::GetRemainingLargestPatchIndex() const {
//    int largest_patch_level = 999;
//    int largest_patch_offset = -1;
//    for (int i = 0; i < num_smallest_patches; i++) {
//        if (data_patch_level[i] == 0) continue;
//        if (data_patch_label[i]) continue;
//        if (data_patch_level[i] < largest_patch_level) {
//            largest_patch_level = data_patch_level[i];
//            largest_patch_offset = i;
//        }
//    }
//    if (largest_patch_offset == -1) return {};
//    else return GetHierarchicalIndexFromOffsetAndLevel(largest_patch_offset, largest_patch_level);
//}

// 由于找最大块作为种子点的作用不大，所以暂时随便找一个未分类的块就作为种子点，待继续优化
//vector<int> PatchSegmentStatus::GetRemainingLargestPatchIndex(int& start_from_i) const {
//    int largest_patch_offset = -1;
//    int largest_patch_level = 0;
//    for (int i = start_from_i; i < num_smallest_patches; i++) {
//        if (data_patch_level[i] == 0) continue;
//        if (data_patch_label[i]) continue;
//
//        largest_patch_level = data_patch_level[i];
//        largest_patch_offset = i;
//        start_from_i = i+1;
//        break;
//    }
//
//    if (largest_patch_offset == -1) return {};
//    else return GetHierarchicalIndexFromOffsetAndLevel(largest_patch_offset, largest_patch_level);
//}

// 所有块已经排序之后,依次挑选剩余的最大的块作为种子点
vector<int> PatchSegmentStatus::GetRemainingLargestPatchIndex() {
    while(true){
        if(unlabeled_patch_index_sorted_by_level_smallest_first.empty()) return {};
        auto ind = unlabeled_patch_index_sorted_by_level_smallest_first.back();
        unlabeled_patch_index_sorted_by_level_smallest_first.pop_back();
        if(IsLabeled(ind)) continue;
        return ind;
    }
}

vector<int> PatchSegmentStatus::GetHierarchicalIndexFromOffsetAndLevel(int offset, unsigned short level) const {
    vector<int> ind(level);
    for (int i = 0; i < level; i++) {
        ind[i] = offset / offsets_each_level[i];
        offset = offset % offsets_each_level[i];
    }
    return ind;
}

vector<int> PatchSegmentStatus::GetMostSimilarPatchIndex(vector<int> ind) const{
    for (int i = 0; i < ind.size(); ++i) {
        if (Has(vector<int>(ind.begin(), ind.begin()+i+1)))
            return {ind.begin(), ind.begin()+i+1};
    }
    return {};
}

void PatchSegmentStatus::SortUnlabeledPatchIndex_SmallestFirst() {
    for(auto indexes : unlabeled_patch_index_grouped_by_level_smallest_first){
        unlabeled_patch_index_sorted_by_level_smallest_first.insert(
                unlabeled_patch_index_sorted_by_level_smallest_first.end(), indexes.begin(), indexes.end());
    }
}

