#ifndef HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
#define HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H

#include<vector>

using std::vector;

class PatchSegmentResult {
private:
    int m_index_layers;
    int m_num_items;
    int *m_offsets_each_level;
    int *data;
public:
    PatchSegmentResult();
    void Initialize(const int *num_childs_each_level, int max_index_layers);
    void Set(vector<int> ind, int value);
    int Get(vector<int> ind) const;
    ~PatchSegmentResult();
};


#endif //HIRACHICALPLANESEGMENTATION_PATCHSEGMENTRESULT_H
