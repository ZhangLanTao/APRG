#ifndef HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
#define HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H

#include <vector>
#include "PointsSumPyramid.h"

using std::vector;

class PatchSegmentResult {
private:
    int m_index_layers;
    int m_num_smallest_patches;
    int *m_offsets_each_level;
    int *m_data_patch_level;
    int *m_data_patch_label;
    PlaneParams *m_data_plane_params;
    bool *m_data_patch_visited;

    vector<int> GetHierarchicalIndexFromOffsetAndLevel(int offset, unsigned short level) const;

public:
    PatchSegmentResult();

    void Initialize(const int *num_childs_each_level, int max_index_layers);

    int GetOffsetOfHIndex(const vector<int> &ind) const;

    void SetPatchSegmentResult(const vector<int>& ind, int value, PlaneParams plane_params);

    void SetPatchLabel(const vector<int>& ind, int label);

    int GetPatchLabel(const vector<int>& ind);

    bool IsLabeled(const vector<int>& ind);

    int GetPatchSegmentLevel(const vector<int>& ind) const;

    PlaneParams GetPatchPlaneParameter(const vector<int>& ind) const;

    vector<int> GetRemainingLargestPatchIndex(int& start_from_i) const;

    void SetPatchVisited(vector<int> ind);

    bool IsVisited(const vector<int>& ind) const;

    void ResetVisited();

    inline bool Has(const vector<int> &ind) const{
        int level = GetPatchSegmentLevel(ind);
        return level == ind.size();
    };

    vector<int> GetMostSimilarPatchIndex(vector<int> ind) const;

    ~PatchSegmentResult();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
