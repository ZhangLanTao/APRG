#include "HirachicalIndex.h"
#include <vector>
HirachicalIndex::HirachicalIndex(const int* items_each_level, const int max_index_level) {
    this->items_each_level = new int[max_index_level];
    indexed_size = 1;
    for (int i = max_index_level; i >= 0; i--) {
        indexed_size *= items_each_level[i];
        this->items_each_level[i] = indexed_size;
    }
}

HirachicalIndex::~HirachicalIndex() {
    delete[] this->items_each_level;
}

int HirachicalIndex::GetIndex(const int ind[], int n) {
    int offset = 0;
    for (int i = 0; i < n; i++) {
        offset += ind[i] * items_each_level[i];
    }
    return offset;
}

