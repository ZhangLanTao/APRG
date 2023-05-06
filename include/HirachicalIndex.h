#ifndef HIRACHICALPLANESEGMENTATION_HIRACHICALINDEX_H
#define HIRACHICALPLANESEGMENTATION_HIRACHICALINDEX_H


class HirachicalIndex {
public:
    int *items_each_level;
    int indexed_size;
public:
    explicit HirachicalIndex(const int* items_each_level, const int max_index_level);
    ~HirachicalIndex();
    int GetIndex(const int ind[], int n);
    inline int GetIndexedSize() const{return indexed_size;};
};


#endif //HIRACHICALPLANESEGMENTATION_HIRACHICALINDEX_H
