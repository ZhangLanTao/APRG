#ifndef HIERARCHICALPLANESEGMENTATION_SEGSTATEPYRAMID_H
#define HIERARCHICALPLANESEGMENTATION_SEGSTATEPYRAMID_H

#include <vector>
#include "Params.h"
#include "utils.h"

/*  patch用到的所有信息
 *  _is_plane: 是否是平面
 *  _plane_params: 平面参数(包含求和信息)
 *  _RG_visited: 是否被区域生长访问过，一个区域生长完成之后，需要被重置
 *  _label: 归属于区域的标签
 *  _original_ind: 记录此patch的来源是哪个patch()
 * */
struct PatchState {
    bool _is_plane = false;
    PlaneParams _plane_params;
    bool _RG_visited = false;
    int _label = -1;
    vector<int> _original_ind = vector<int>({-1,-1,-1});
};


class SegStatePyramid_layer {
public:
    vector<vector<PatchState>> _patch_states;
    int _height, _width, _layer_n;  // _layer_n 记录自己处于金字塔的第几层
    void Init(int height, int width, int layer_n);
    void SetPlaneParams(int row, int col, const PlaneParams &params, const vector<int> &original_ind);
    void Clear(){
        for (int i = 0; i < _height; ++i) {
            for (int j = 0; j < _width; ++j) {
                _patch_states[i][j] = PatchState();
            }
        }
    };

    void ResetVisited(){
        for (int i = 0; i < _height; ++i) {
            for (int j = 0; j < _width; ++j) {
                _patch_states[i][j]._RG_visited = false;
            }
        }
    };

    vector<PatchState> & operator[] (int i){ return _patch_states[i]; };
};

class SegStatePyramid {
private:
    vector<SegStatePyramid_layer> _layers;
    int _num_layers, _block_num_x, _block_num_y;

public:
    void Init(Params _params);
    void AddPatch(std::vector<int> ind, PlaneParams params);
    void CompletePyramid();

    void Clear(){
        for (int i = 0; i < _num_layers; ++i) {
            _layers[i].Clear();
        }
    };

    void ResetVisited() {
        for (int i = 0; i < _num_layers; ++i) {
            _layers[i].ResetVisited();
        }
    };

    void SetLabel(const vector<int> &ind, int label) {
        for (int i = ind[0], scale = 1; i >= 0; --i) {
            for (int j = 0; j < scale; ++j) {
                for (int k = 0; k < scale; ++k) {
                    _layers[i][ scale * ind[1] + j ][ scale * ind[2] +k ]._label = label;
                }
            }
            scale *= 2;
        }
    };

    void SetVisited(const vector<int> &ind, bool value) {
        for (int i = ind[0], scale = 1; i >= 0; --i) {
            for (int j = 0; j < scale; ++j) {
                for (int k = 0; k < scale; ++k) {
                    _layers[i][ scale * ind[1] + j ][ scale * ind[2] +k ]._RG_visited = value;
                }
            }
            scale *= 2;
        }
    };

    SegStatePyramid_layer & operator[] (int i) { return _layers[i]; };
    PatchState & operator[] (const std::vector<int> &ind) { return _layers[ind[0]][ind[1]][ind[2]]; };
};


#endif //HIERARCHICALPLANESEGMENTATION_SEGSTATEPYRAMID_H
