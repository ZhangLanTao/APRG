#ifndef HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
#define HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H

#include <vector>
#include "PointsInfoPyramid.h"

using std::vector;

class PatchSegmentResult {
private:
    int m_index_layers;
    int m_num_items;
    int *m_offsets_each_level;
    int *m_data_segment;
    PlaneParams *m_data_plane_params;
public:
    PatchSegmentResult();
    void Initialize(const int *num_childs_each_level, int max_index_layers);
    void Set(vector<int> ind, int value, PlaneParams plane_params);
    int GetSegmentResult(vector<int> ind) const;
    PlaneParams GetPlaneParameter(vector<int> ind) const;

    ~PatchSegmentResult();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
