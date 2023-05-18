#ifndef HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
#define HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H

#include <vector>
#include "PointsInfoPyramid.h"

using std::vector;

class PatchSegmentResult {
private:
    int m_index_layers;
    int m_num_smallest_patches;
    int *m_offsets_each_level;
    unsigned char *m_data_patch_level;
    unsigned short *m_data_patch_label;
    PlaneParams *m_data_plane_params;
public:
    PatchSegmentResult();
    PatchSegmentResult(PatchSegmentResult &patch_segment_result);
    void Initialize(const int *num_childs_each_level, int max_index_layers);
    void SetPatchSegmentResult(vector<int> ind, int value, PlaneParams plane_params);
    void SetPatchLabel(vector<int> ind, int label);
    int GetPatchSegmentResult(vector<int> ind) const;
    PlaneParams GetPatchPlaneParameter(vector<int> ind) const;

    ~PatchSegmentResult();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
