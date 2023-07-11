#ifndef HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
#define HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H

#include <vector>
#include "PointsSumPyramid.h"

using std::vector;

class PatchSegmentResult {
private:
    uint8_t m_index_layers;
    uint16_t m_num_smallest_patches;
    uint16_t *m_offsets_each_level;
    uint8_t *m_data_patch_level;
    uint16_t *m_data_patch_label;
    PlaneParams *m_data_plane_params;
    bool *m_data_patch_visited;

    vector<uint8_t> GetHierarchicalIndexFromOffsetAndLevel(uint16_t offset, unsigned short level) const;

public:
    PatchSegmentResult();

    void Initialize(const uint8_t *num_childs_each_level, uint8_t max_index_layers);

    int GetOffsetOfHIndex(const vector<uint8_t> &ind) const;

    void SetPatchSegmentResult(const vector<uint8_t>& ind, uint8_t value, PlaneParams plane_params);

    void SetPatchLabel(const vector<uint8_t>& ind, uint16_t label);

    int GetPatchLabel(const vector<uint8_t>& ind);

    bool IsLabeled(const vector<uint8_t>& ind);

    uint8_t GetPatchSegmentLevel(const vector<uint8_t>& ind) const;

    PlaneParams GetPatchPlaneParameter(const vector<uint8_t>& ind) const;

    vector<uint8_t> GetRemainingLargestPatchIndex(int& start_from_i) const;

    void SetPatchVisited(vector<uint8_t> ind);

    bool IsVisited(const vector<uint8_t>& ind) const;

    void ResetVisited();

    inline bool Has(const vector<uint8_t> &ind) const{
        uint8_t level = GetPatchSegmentLevel(ind);
        return level == ind.size();
    };

    vector<uint8_t> GetMostSimilarPatchIndex(vector<uint8_t> ind) const;

    ~PatchSegmentResult();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
