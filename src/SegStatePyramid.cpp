#include "SegStatePyramid.h"

using std::cout, std::endl, std::vector;

void SegStatePyramid::Init(Params params) {
    _num_layers  = params.num_pyramid_layers;
    _block_num_x = params.block_num_x;
    _block_num_y = params.block_num_y;

    _layers         = vector<SegStatePyramid_layer>(_num_layers);
    int temp_height = params.block_num_y, temp_width = params.block_num_x;
    for (int i = params.num_pyramid_layers - 1; i >= 0; --i) {
        _layers[i].Init(temp_height, temp_width, i);
        temp_height *= 2;
        temp_width *= 2;
    }
}

// Add a confirmed patch to SegStatePyramid
// input:
//     ind: index of the patch
//     params: plane parameters of the patch
void SegStatePyramid::AddPatch(vector<int> ind, PlaneParams params) {
    int layer = _num_layers - ind.size();
    int row = (ind[0] / _block_num_x), col = (ind[0] % _block_num_x);
    for (int i = 1; i < ind.size(); ++i) {
        row *= 2;
        row += (ind[i] / 2);
        col *= 2;
        col += (ind[i] % 2);
    }
    _layers[layer].SetPlaneParams(row, col, params, {layer, row, col});
}

// Write the parameters of the upper layer to the corresponding position of the lower layer
void SegStatePyramid::CompletePyramid() {
    for (int i = _num_layers - 2; i >= 0; --i) {
        // 将第i+1层的参数写到i层对应的位置
        for (int j = 0; j < _layers[i]._height; ++j) {
            for (int k = 0; k < _layers[i]._width; ++k) {
                if (_layers[i + 1][j / 2][k / 2]._is_plane) {
                    _layers[i].SetPlaneParams(j, k, _layers[i + 1][j / 2][k / 2]._plane_params,
                                              _layers[i + 1][j / 2][k / 2]._original_ind);
                }
            }
        }
    }
}

void SegStatePyramid_layer::Init(int height, int width, int layer_n) {
    _height       = height;
    _width        = width;
    _layer_n      = layer_n;
    _patch_states = vector<vector<PatchState>>(height);
    for (int i = 0; i < height; ++i) {
        _patch_states[i] = vector<PatchState>(width);
    }
}

inline void SegStatePyramid_layer::SetPlaneParams(int row, int col, const PlaneParams &params,
                                                  const vector<int> &original_ind) {
    _patch_states[row][col]._is_plane     = true;
    _patch_states[row][col]._original_ind = original_ind;
    _patch_states[row][col]._plane_params = params;
}
