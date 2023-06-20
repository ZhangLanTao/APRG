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
    unsigned short *m_data_patch_level;
    unsigned short *m_data_patch_label;
    PlaneParams *m_data_plane_params;
    bool *m_data_patch_visited;

    vector<int> GetHierachicalIndexFromOffsetAndLevel(int offset, unsigned short level) const;

public:
    PatchSegmentResult();

    PatchSegmentResult(PatchSegmentResult &patch_segment_result);

    void Initialize(const int *num_childs_each_level, int max_index_layers);

    int GetOffsetOfHIndex(const vector<int> &ind) const;

    void SetPatchSegmentResult(vector<int> ind, int value, PlaneParams plane_params);

    void SetPatchLabel(vector<int> ind, int label);

    int GetPatchLabel(vector<int> ind);

    bool IsLabled(vector<int> ind);

    unsigned short GetPatchSegmentLevel(vector<int> ind) const;

    PlaneParams GetPatchPlaneParameter(vector<int> ind) const;

    vector<int> GetRemainingLargestPatchIndex() const;

    void SetPatchVisited(vector<int> ind);

    bool IsVisited(vector<int> ind) const;

    void ResetVisited();

    inline bool Has(const vector<int>& ind) const{
        unsigned short level = GetPatchSegmentLevel(ind);
        return level == ind.size();
    };

    vector<int> GetMostSimilarPatchIndex(vector<int> ind) const;

    ~PatchSegmentResult();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
