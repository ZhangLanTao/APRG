#include "HPS.h"
#include "utils.h"
#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include <iostream>

using std::cout, std::endl;

void HPS::_Init() {
    _points_sample_pyramid.Init(_params);
    _seg_state_pyramid.Init(_params);
    _color_img = cv::Mat::zeros(_params.img_height, _params.img_width, CV_8UC3);
    _layer0_label_img = cv::Mat::zeros(_seg_state_pyramid[0]._height, _seg_state_pyramid[0]._width, CV_8UC1);
    _layer0_label_img_eroded = cv::Mat::zeros(_seg_state_pyramid[0]._height, _seg_state_pyramid[0]._width, CV_8UC1);
    _layer0_label_img_eroded_scaleup = cv::Mat::zeros(_params.img_height, _params.img_width, CV_8UC1);
    _final_label_img = cv::Mat::zeros(_params.img_height, _params.img_width, CV_8UC1);

    _BR_pixel_refine_min_distance_to_plane = (float*)malloc(_params.img_height * _params.img_width * sizeof(float));

    _Debug_label_color_lut.at<cv::Vec3d>(0, 0) = 0;
    for (int i = 1; i < 256; i++) {
        _Debug_label_color_lut.at<cv::Vec3b>(0, i) = cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
    }
}

void HPS::SetData(const cv::Mat &color_img, const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z) {
    _color_img = color_img.clone();
    _points_sample_pyramid.SetData(X, Y, Z);
}

inline void HPS::_BuildPyramid() {
    _points_sample_pyramid.BuildPyramid();
}

void HPS::_RecursivePlaneSegment() {
    vector<vector<int>> wait_to_process;
    wait_to_process.reserve(10 * _params.num_pyramid_layers);
    for (int i = _params.block_num_x * _params.block_num_y - 1; i >= 0; i--) {
        wait_to_process.emplace_back(vector<int>{i});
    }

    while (!wait_to_process.empty()) {
        auto this_ind = wait_to_process.back();
        wait_to_process.pop_back();

        // 获取index对应的patch的点云，写成函数需要用vector返回三个矩阵，vector的操作浪费时间
        const int min_patch_size = (_params.img_height / _params.block_num_y) >> (_params.num_pyramid_layers - 1);
        int row = min_patch_size * (this_ind[0] / _params.block_num_x), col =
                min_patch_size * (this_ind[0] % _params.block_num_x);
        for (int i = 1; i < this_ind.size(); ++i) {
            row *= 2;
            col *= 2;
            row += min_patch_size * (this_ind[i] / 2);
            col += min_patch_size * (this_ind[i] % 2);
        }
        int layer = _params.num_pyramid_layers - this_ind.size();
        auto sampled_X = _points_sample_pyramid.GetSampledPoints(row, col, layer, 0, min_patch_size);
        auto sampled_Y = _points_sample_pyramid.GetSampledPoints(row, col, layer, 1, min_patch_size);
        auto sampled_Z = _points_sample_pyramid.GetSampledPoints(row, col, layer, 2, min_patch_size);

        auto jumps = CountJumps(sampled_Z, _params.PS_jump_max_distance2);
        PlaneParams plane_param;

        if (jumps > _params.PS_max_allowed_jump_cnt) goto NOT_PLANE__GO_NEXT_LAYER;
        plane_param = FitPlane(sampled_X, sampled_Y, sampled_Z);

//        if (false){
//            DebugShowResult();
//            DebugShowAPatch_HIndex(this_ind);
//            int key = cv::waitKey();
//            if (key == 's') FitPlane(sampled_X, sampled_Y, sampled_Z, true, true);
//        }

        if (_params.PS_plane_thresh_mode == PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE and
            plane_param.MSE > _params.PS_plane_thresh)
            goto NOT_PLANE__GO_NEXT_LAYER;
        if (_params.PS_plane_thresh_mode == PLANE_PATCH_THRESHMODE_DISTANCE_RATIO and plane_param.MSE >
                                                                                      _params.PS_plane_thresh *
                                                                                      (plane_param.mean_x *
                                                                                       plane_param.mean_x +
                                                                                       plane_param.mean_y *
                                                                                       plane_param.mean_y +
                                                                                       plane_param.mean_z *
                                                                                       plane_param.mean_z))
            goto NOT_PLANE__GO_NEXT_LAYER;


        // 经过所有判断，认为这个patch是一个，平面此patch到此为止
        _seg_state_pyramid.AddPatch(this_ind, plane_param);
        continue;

        NOT_PLANE__GO_NEXT_LAYER:
        if (this_ind.size() < _params.num_pyramid_layers) {
            for (int j = 0; j < 4; j++) {
                vector<int> new_ind = this_ind;
                new_ind.emplace_back(j);
                wait_to_process.emplace_back(new_ind);
            }
        }
    }

    _seg_state_pyramid.CompletePyramid();
}

void HPS::_PatchwiseRegionGrowing() {
    vector<int> seed_ind{_params.num_pyramid_layers - 1, 0, 0};
    while (_RG_GetNextSeed(seed_ind)) { // RG_GetNextSeed有下一个种子，返回true,seed_ind为下一个种子
        _RG_region_areas.emplace_back();
        _RG_region_plane_params.emplace_back();
        _RG_RegionGrowingFrom(seed_ind, _RG_region_count);
        ++_RG_region_count;
    }
}

bool HPS::_RG_GetNextSeed(vector<int> &seed_ind) {
    int &layer = seed_ind[0], &row = seed_ind[1], &col = seed_ind[2];

    GET_NEXT_ONE:
    ++col;
    if (col == _seg_state_pyramid[layer]._width) {
        col = 0;
        ++row;
        if (row == _seg_state_pyramid[layer]._height) {
            row = 0;
            --layer;
            if (layer < 0) {        // 已完成所有种子遍历
                seed_ind.clear();
                return false;
            }
        }
    }

    if (_seg_state_pyramid[seed_ind]._label!=-1 or !_seg_state_pyramid[seed_ind]._is_plane) goto GET_NEXT_ONE;                    // 如果seed对应的已经是平面了(labeled)，或它不是平面，则继续找下一个
    return true;
}

void HPS::_RG_RegionGrowingFrom(const vector<int> &seed_ind, int label) {
    vector<vector<int>> wait_to_process = {seed_ind};
    int done_count = 0;
    int region_area = 0;
    PlaneParams region_plane_param;

    while (done_count != wait_to_process.size()){
        auto &seed = wait_to_process[done_count];
        ++ done_count;

        _seg_state_pyramid.SetVisited(seed, true);
        _seg_state_pyramid.SetLabel(seed, label);
        region_area += 1 << seed[0];
        // 画seed
//        auto img_copy = _color_img.clone();
//        const int min_patch_size = (_params.img_height / _params.block_num_y) >> (_params.num_pyramid_layers - 1);
//        auto size_scaled = min_patch_size << seed[0];
//        int row = seed[1], col = seed[2];
//        auto rect = cv::Rect(size_scaled * col, size_scaled * row, size_scaled, size_scaled);
//        cv::rectangle(img_copy, rect, cv::Scalar(0, 255, 0), 1);

        PlaneParams this_plane_param = _seg_state_pyramid[seed]._plane_params;
        vector<vector<int>> neighbor_inds = _RG_GetNeighbors(seed);

        for (auto &neighbor_ind: neighbor_inds) {
            if (_seg_state_pyramid[neighbor_ind]._RG_visited) continue;
            _seg_state_pyramid.SetVisited(neighbor_ind, true);
            auto &neighbor_plane_param = _seg_state_pyramid[neighbor_ind]._plane_params;
            if (IsCoplanar(this_plane_param, neighbor_plane_param, _params.RG_plane_merge_min_cos_angle,
                           _params.RG_plane_merge_thresh_mode, _params.RG_plane_merge_max_distance2)) {
                // 确定邻居属于同一个region
                _seg_state_pyramid.SetLabel(neighbor_ind, label);
                region_area += 1 << neighbor_ind[0];
                region_plane_param += neighbor_plane_param;

                // 接下来的位置
                wait_to_process.emplace_back(neighbor_ind);
                // 画邻居
//                size_scaled = min_patch_size << neighbor_ind[0];
//                row = neighbor_ind[1], col = neighbor_ind[2];
//                rect = cv::Rect(size_scaled * col, size_scaled * row, size_scaled, size_scaled);
//                cv::rectangle(img_copy, rect, cv::Scalar(255, 0, 0), 1);
//                cv::imshow("img-copy", img_copy);
            }
        }
    }

    // 恢复 visited 状态
    for (auto &ind: wait_to_process) {
        _seg_state_pyramid.SetVisited(ind, false);
    }
    _RG_region_areas[label] = region_area;

    // region够大的话重算region平面参数
    if (region_area>= _params.RG_min_area){
        FitPlaneAlreadyHaveSum(region_plane_param);
        _RG_region_plane_params[label] = region_plane_param;
    }
    else{
        _RG_region_plane_params[label] = _seg_state_pyramid[seed_ind]._plane_params;
    }
}

vector<vector<int>> HPS::_RG_GetNeighbors(const vector<int> &ind) {
    vector<vector<int>> result;
    result.reserve(10);
    int layer = ind[0], row = ind[1], col = ind[2];
    int row_in_layer0 = row << layer, col_in_layer0 = col << layer;
    int this_patch_size = 1 << layer;

    // check boarder
    bool up = true, down = true, left = true, right = true;
    if (row == 0) up = false;
    if (row == _seg_state_pyramid[layer]._height - 1) down = false;
    if (col == 0) left = false;
    if (col == _seg_state_pyramid[layer]._width - 1) right = false;
    if (up) {
        for (int col_temp = col_in_layer0; col_temp < col_in_layer0 + this_patch_size;) {
            if (not _seg_state_pyramid[0][row_in_layer0 - 1][col_temp]._is_plane) {
                ++col_temp;
                continue;
            }

            const auto &serched_original_ind = _seg_state_pyramid[0][row_in_layer0 - 1][col_temp]._original_ind;
            if (_seg_state_pyramid[serched_original_ind]._RG_visited or
                _seg_state_pyramid[serched_original_ind]._label!=-1) {
                col_temp += 1 << serched_original_ind[0];
                continue;
            }
            result.emplace_back(serched_original_ind);
            col_temp += 1 << serched_original_ind[0];
        }
    }
    if (down) {
        for (int col_temp = col_in_layer0; col_temp < col_in_layer0 + this_patch_size;) {
            if (not _seg_state_pyramid[0][row_in_layer0 + this_patch_size][col_temp]._is_plane) {
                ++col_temp;
                continue;
            }

            const auto &serched_original_ind = _seg_state_pyramid[0][row_in_layer0 +
                                                                     this_patch_size][col_temp]._original_ind;
            if (_seg_state_pyramid[serched_original_ind]._RG_visited or
                _seg_state_pyramid[serched_original_ind]._label!=-1) {
                col_temp += 1 << serched_original_ind[0];
                continue;
            }

            result.emplace_back(serched_original_ind);
            col_temp += 1 << serched_original_ind[0];
        }
    }
    if (left) {
        for (int row_temp = row_in_layer0; row_temp < row_in_layer0 + this_patch_size;) {
            if (not _seg_state_pyramid[0][row_temp][col_in_layer0 - 1]._is_plane) {
                ++row_temp;
                continue;
            }

            const auto &serched_original_ind = _seg_state_pyramid[0][row_temp][col_in_layer0 - 1]._original_ind;
            if (_seg_state_pyramid[serched_original_ind]._RG_visited or
                _seg_state_pyramid[serched_original_ind]._label!=-1) {
                row_temp += 1 << serched_original_ind[0];
                continue;
            }

            result.emplace_back(serched_original_ind);
            row_temp += 1 << serched_original_ind[0];
        }
    }
    if (right) {
        for (int row_temp = row_in_layer0; row_temp < row_in_layer0 + this_patch_size;) {
            if (not _seg_state_pyramid[0][row_temp][col_in_layer0 + this_patch_size]._is_plane) {
                ++row_temp;
                continue;
            }

            const auto &serched_original_ind = _seg_state_pyramid[0][row_temp][col_in_layer0 +
                                                                               this_patch_size]._original_ind;
            if (_seg_state_pyramid[serched_original_ind]._RG_visited or
                _seg_state_pyramid[serched_original_ind]._label!=-1) {
                row_temp += 1 << serched_original_ind[0];
                continue;
            }
            row_temp += 1 << serched_original_ind[0];
            result.emplace_back(serched_original_ind);
        }
    }
    return result;
}


void HPS::_BoundaryRefine() {
    // 记录patch被哪个label细分，用于判断region邻接
    cv::Mat patch_already_refined_by_label = cv::Mat::zeros(_layer0_label_img.size(), CV_8UC1);
    // 共面的region合并，用lut改变label
    cv::Mat change_label_lut = cv::Mat(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++) {
        change_label_lut.at<uchar>(0, i) = i;
    }

    // 记录像素被细分到region时的距离，只有发现更小的距离时才换label
//    std::memset(_BR_pixel_refine_min_distance_to_plane, 100, _params.img_height*_params.img_width*sizeof(float)); /* = to really high float*/

    // convert label to image
    auto &bottom_layer = _seg_state_pyramid[0];

    for (int i = 0; i < bottom_layer._height; ++i) {
        for (int j = 0; j < bottom_layer._width; ++j) {
            auto label = bottom_layer[i][j]._label;
            if (_RG_region_areas[label] >= _params.RG_min_area)
            _layer0_label_img.at<uchar>(i, j) = label;
        }
    }

    cv::Mat mask = cv::Mat(_layer0_label_img.size(), CV_8UC1);
    cv::Mat mask_diff = cv::Mat(_layer0_label_img.size(), CV_8UC1);
    cv::Mat BR_mask_eroded = cv::Mat::zeros(_layer0_label_img.size(), CV_8UC1);
    cv::Mat mask_dilated = cv::Mat(_layer0_label_img.size(), CV_8UC1);
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    for (int label = 1; label < _RG_region_count; ++label) {
        if (_RG_region_areas[label] < _params.RG_min_area) continue;
        mask = 0, mask_diff = 0, mask_dilated = 0;

        mask.setTo(1, _layer0_label_img == label);
        // 腐蚀操作，为了加速可以省略不做，仅做膨胀
        cv::erode(mask, BR_mask_eroded, morph_kernel);
        _layer0_label_img_eroded.setTo(label, BR_mask_eroded);
        // If completely eroded ignore plane
        // int num_remain = cv::countNonZero(BR_mask_eroded);
        // if (num_remain==0){
        // continue;
        // }

        // Dilate to obtain borders
        cv::dilate(mask, mask_dilated, morph_kernel, cv::Point(-1, -1), 2);
        mask_diff = mask_dilated - BR_mask_eroded;
        // mask_diff = mask_dilated - mask;

        const int min_patch_size = (_params.img_height / _params.block_num_y) >> (_params.num_pyramid_layers - 1);
        float nx = _RG_region_plane_params[label].a;
        float ny = _RG_region_plane_params[label].b;
        float nz = _RG_region_plane_params[label].c;
        float d = _RG_region_plane_params[label].d;

        for (int cell_row = 0; cell_row < mask_diff.rows; ++cell_row) {
            for (int cell_col = 0; cell_col < mask_diff.cols; ++cell_col) {
                if (mask_diff.at<uchar>(cell_row, cell_col) != 0){
                    auto l = patch_already_refined_by_label.at<uchar>(cell_row, cell_col);
                    if (l != 0){        // 若这个块已被别的region l细分，则判断是否合并
                        auto param1 = _RG_region_plane_params[l];
                        auto param2 = _RG_region_plane_params[label];
                        if (IsCoplanar(param1, param2,_params.RG_plane_merge_min_cos_angle, _params.RG_plane_merge_thresh_mode, _params.RG_plane_merge_max_distance2)){
                            uchar target, from;
                            if(_RG_region_areas[label]>_RG_region_areas[l]){
                                target = label;
                                from = l;
                            }
                            else{
                                target = l;
                                from = label;
                            }
                            change_label_lut.at<uchar>(0, from) = target;
                        }
                    }
                    else patch_already_refined_by_label.at<uchar>(cell_row, cell_col) = label;

                    // 计算距离，给像素安排label
                    auto X = _points_sample_pyramid.GetSampledPoints(min_patch_size*cell_row, min_patch_size*cell_col, 0, 0, min_patch_size);
                    auto Y = _points_sample_pyramid.GetSampledPoints(min_patch_size*cell_row, min_patch_size*cell_col, 0, 1, min_patch_size);
                    auto Z = _points_sample_pyramid.GetSampledPoints(min_patch_size*cell_row, min_patch_size*cell_col, 0, 2, min_patch_size);
                    auto distances_cell_stacked = (X*nx + Y*ny + Z*nz).array()+d;
                    // write pixel label

                    int offset = (cell_row * mask_diff.cols + cell_col) * min_patch_size * min_patch_size;
                    for (int r = 0, count=0; r < distances_cell_stacked.rows(); ++r) {
                        for (int c = 0; c < distances_cell_stacked.cols(); ++c, ++count) {
                            float dist = pow(distances_cell_stacked(r, c), 2);
//                            auto &min_dist = _BR_pixel_refine_min_distance_to_plane[offset+count];
                            if (dist < _params.BR_point_coplane_max_distance2) {
                                _final_label_img.at<uchar>(cell_row * min_patch_size + r,
                                                          cell_col * min_patch_size + c) = label;
//                                min_dist = dist;
                            }
                        }
                    }
                }
            }
        }
    }

    cv::resize(_layer0_label_img_eroded, _layer0_label_img_eroded_scaleup, cv::Size(_params.img_width, _params.img_height), 0, 0, cv::INTER_NEAREST);
    _final_label_img.setTo(0, _layer0_label_img_eroded_scaleup != 0);
    _final_label_img += _layer0_label_img_eroded_scaleup;

    // 递归找到最终的target
    for (int l=0; l<change_label_lut.cols; ++l) {
        auto &target = change_label_lut.at<uchar>(0, l);
        while (change_label_lut.at<uchar>(0, target) != target) {
            target = change_label_lut.at<uchar>(0, target);
        }
    }

    cv::LUT(_final_label_img, change_label_lut, _final_label_img);
//
//    cv::Mat img_show = _final_label_img.clone();
//    cv::resize(patch_label_img, img_show, cv::Size(_params.img_width, _params.img_height), 0, 0, cv::INTER_NEAREST);
//    cv::applyColorMap(img_show, img_show, cv::COLORMAP_JET);
//    cv::imshow("patch_label_img", img_show);
//    cv::waitKey();

}


void HPS::Process() {
    g_program_counter = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    _BuildPyramid();
    auto t2 = std::chrono::high_resolution_clock::now();
    _RecursivePlaneSegment();
    auto t3 = std::chrono::high_resolution_clock::now();
    _PatchwiseRegionGrowing();
    auto t4 = std::chrono::high_resolution_clock::now();
    _BoundaryRefine();
    auto t5 = std::chrono::high_resolution_clock::now();

    auto time_elapsed1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    auto time_elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    auto time_elapsed3 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    auto time_elapsed4 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
    auto time_elapsed5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t1).count();
    cout << "_BuildPyramid: " << time_elapsed1 << "us" << endl;
    cout << "_RecursivePlaneSegment: " << time_elapsed2 << "us" << endl;
    cout << "_PatchwiseRegionGrowing: " << time_elapsed3 << "us" << endl;
    cout << "_BoundaryRefine: " << time_elapsed4 << "us" << endl;
    cout << "Total: " << time_elapsed5 << "us" << endl;
    cout << "FPS: " << 1e6/time_elapsed5 << endl;
    g_total_time += time_elapsed5;
}

void HPS::GetResultReadable(cv::Mat &output) {
    output = _final_label_img.clone();
    cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
    cv::LUT(output, _Debug_label_color_lut, output);
}

void HPS::Clear() {
    _points_sample_pyramid.Clear();
    _seg_state_pyramid.Clear();
    _layer0_label_img = 255;
    _final_label_img = 255;
    _RG_region_count = 0;
    _RG_region_areas.clear();
    _RG_region_plane_params.clear();
}

void HPS::DebugShowResult(int waitkey) {
    cv::Mat show;
    const int min_patch_size = (_params.img_height / _params.block_num_y) >> (_params.num_pyramid_layers - 1);

    GetResultReadable(show);
    show /= 2;

    for (int i = 0; i < _params.num_pyramid_layers; ++i) {
        auto this_layer = _seg_state_pyramid[i];
        auto size_scaled = min_patch_size << i;
        for (int j = 0; j < this_layer._height; ++j) {
            for (int k = 0; k < this_layer._width; ++k) {
                if (this_layer[j][k]._is_plane and this_layer[j][k]._original_ind[0] == i) {
                    auto rect = cv::Rect(size_scaled * k, size_scaled * j, size_scaled, size_scaled);
                    auto label = this_layer[j][k]._label;
                    auto color = _Debug_label_color_lut.at<cv::Vec3b>(0, label);
                    cv::rectangle(show, rect, color, 1);
                }
            }
        }
    }

    cv::addWeighted(show, 0.6, _color_img, 0.4, 0, show);
//    cv::resize(show, show, cv::Size(0, 0), 2, 2, cv::INTER_NEAREST);
    cv::imshow("show", show);
    cv::waitKey(waitkey);
}

void HPS::DebugShowAPatch_HIndex(vector<int>(ind)) {
    int layer = _params.num_pyramid_layers - ind.size();
    const int min_patch_size = (_params.img_height / _params.block_num_y) >> (_params.num_pyramid_layers - 1);
    auto size_scaled = min_patch_size << layer;
    int row = (ind[0] / _params.block_num_x), col = (ind[0] % _params.block_num_x);
    for (int i = 1; i < ind.size(); ++i) {
        row *= 2;
        col *= 2;
        row += (ind[i] / 2);
        col += (ind[i] % 2);
    }
    cv::Mat show = _color_img.clone();
    auto rect = cv::Rect(size_scaled * col, size_scaled * row, size_scaled, size_scaled);
    cv::rectangle(show, rect, cv::Scalar(255, 0, 0), 1);
    cv::imshow("show", show);
}
