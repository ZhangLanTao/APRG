#ifndef HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
#define HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H

#include <vector>
#include "PointsSumPyramid.h"

using std::vector;

class PatchSegmentStatus {
private:
    int num_layers;
    int num_smallest_patches;
    int *offsets_each_level;
    int *data_patch_level;
    int *data_patch_label;
    PlaneParams *data_plane_params;
    bool *data_patch_visited;

    vector<vector<vector<int>>> unlabeled_patch_index_grouped_by_level_smallest_first;
    vector<vector<int>> unlabeled_patch_index_sorted_by_level_smallest_first;

    vector<int> GetHierarchicalIndexFromOffsetAndLevel(int offset, unsigned short level) const;

    int GetOffsetOfHIndex(const vector<int> &ind) const;
public:
    PatchSegmentStatus();

    void Initialize(const int *num_childs_each_level, int max_index_layers);

    inline void AddPatch(const vector<int>& ind, int level, PlaneParams plane_params){
        int offset = GetOffsetOfHIndex(ind);
        data_patch_level[offset] = level;
        data_plane_params[offset] = plane_params;
        unlabeled_patch_index_grouped_by_level_smallest_first[num_layers - level].push_back(ind);
    };

    void SetPatchLabel(const vector<int>& ind, int label){
        int offset = GetOffsetOfHIndex(ind);
        data_patch_label[offset] = label;
    };

    void SetPatchLabel(const vector<vector<int>>& ind, int label){
        for (auto i : ind) {
            SetPatchLabel(i, label);
        }
    };

    int GetPatchLabel(const vector<int>& ind){
        int offset = GetOffsetOfHIndex(ind);
        return data_patch_label[offset];
    };

    bool IsLabeled(const vector<int>& ind){
        int offset = GetOffsetOfHIndex(ind);
        return data_patch_label[offset] != 0;
    };

    int GetPatchSegmentLevel(const vector<int>& ind) const{
        int offset = GetOffsetOfHIndex(ind);
        return data_patch_level[offset];
    };

    PlaneParams GetPatchPlaneParameter(const vector<int>& ind) const{
        int offset = GetOffsetOfHIndex(ind);
        return data_plane_params[offset];
    };

    void SortUnlabeledPatchIndex_SmallestFirst();

    vector<int> GetRemainingLargestPatchIndex();

    void SetPatchVisited(const vector<int>& ind)
    {
        int offset = GetOffsetOfHIndex(ind);
        data_patch_visited[offset] = true;
    };

    bool IsVisited(const vector<int>& ind) const{
        int offset = GetOffsetOfHIndex(ind);
        return data_patch_visited[offset];
    };

    void ResetVisited() {memset(data_patch_visited, false, num_smallest_patches*sizeof(false));};

    bool Has(const vector<int> &ind) const{
        int level = GetPatchSegmentLevel(ind);
        return level == ind.size();
    };

    vector<int> GetMostSimilarPatchIndex(vector<int> ind) const;

    ~PatchSegmentStatus();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
