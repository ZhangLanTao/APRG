#include "HPS.h"
#include <iostream>
#include "open3d/Open3D.h"
//#include <utility>

using std::endl, std::cout;

HPS::HPS(int img_height, int img_width, int block_num_x, int block_num_y, int tree_level) :
        _img_height(img_height), _img_width(img_width),
        _block_num_x(block_num_x), _block_num_y(block_num_y),
        _num_tree_layers(tree_level) {

    _organized_pointcloud.setZero(_img_height * _img_width, 3);

    _num_patches_each_level = new int[tree_level];
    _num_patches_each_level[0] = block_num_x * block_num_y;
    for (int i = 1; i < tree_level; i++) _num_patches_each_level[i] = 4;

    _patch_sizes = new int[tree_level];
    int temp = img_width / block_num_x;
    _patch_sizes[0] = temp;
    for (int i = 1; i < tree_level; ++i) {
        temp /= 2;
        _patch_sizes[i] = temp;
    }

    _organize_data_cell_map = new int[img_height * img_width];
    InitOrganizeDataCellMap();

    _points_sum_pyramid.Initialize(_num_patches_each_level, _num_tree_layers, _patch_sizes[_num_tree_layers - 1]);
    _patch_segment_status.Initialize(_num_patches_each_level, _num_tree_layers);
    _region_count = 0;

    int grid_shape_width = img_width / _patch_sizes[_num_tree_layers - 1];
    int grid_shape_height = img_height / _patch_sizes[_num_tree_layers - 1];
    _min_patch_label_grid =  cv::Mat::zeros(grid_shape_height, grid_shape_width, CV_8UC1);
    _min_patch_label_grid_eroded =  cv::Mat::zeros(grid_shape_height, grid_shape_width, CV_8UC1);
    _final_pixel_label = cv::Mat::zeros(img_height, img_width, CV_8UC1);

    _RG_filter_by_area = false;
    _RG_min_area = 999;

    _debug_color_label_lut = cv::Mat(1, 256, CV_8UC3);
    _debug_color_label_lut.at<cv::Vec3d>(0, 0) = 0;
    for (int i = 1; i < 256; i++) {
        _debug_color_label_lut.at<cv::Vec3b>(0, i) = cv::Vec3b(rand() % 256, rand() % 256, rand() % 256);
    }
    _change_label_lut = cv::Mat(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++) {
        _change_label_lut.at<uchar>(0, i) = i;
    }
}


void HPS::Process() {
    auto t0 = std::chrono::high_resolution_clock::now();
    OrganizePointCloud();
    auto t1 = std::chrono::high_resolution_clock::now();
    _points_sum_pyramid.PreComputeSum(_organized_pointcloud);
    auto t2 = std::chrono::high_resolution_clock::now();
//    g_program_counter = 0;
    RecursivePlaneSegment(false);
//    cout << g_program_counter << endl;
    auto t3 = std::chrono::high_resolution_clock::now();
    PatchwiseRegionGrowing(false);
    auto t4 = std::chrono::high_resolution_clock::now();
    BoundaryRefine(false);
    auto t5 = std::chrono::high_resolution_clock::now();

//    Waste100us();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout<<"OrganizePointCloud time: "<<time_elapsed<<endl;
    time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    std::cout<<"PreComputeSum time: "<<time_elapsed<<endl;
    time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    std::cout<<"RecursivePlaneSegment time: "<<time_elapsed<<endl;
    time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    std::cout<<"PatchwiseRegionGrowing time: "<<time_elapsed<<endl;
    time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
    std::cout<<"BoundaryRefine time: "<<time_elapsed<<endl<<endl;


}

void HPS::GetHierarchicalIndexFromRowCol(int row, int col, int *hierarchical_index) {
//     判断row col 能否被block_size整除
//    if (row % _patch_sizes[_num_tree_layers - 1] != 0 || col % _patch_sizes[_num_tree_layers - 1] != 0) {
//        std::cout << "row or col can not be divided by block_size" << endl;
//        return;
//    }
    hierarchical_index[0] = row / _patch_sizes[0] * 4 + col / _patch_sizes[0];
    row = row % _patch_sizes[0];
    col = col % _patch_sizes[0];
    for (int i = 1; i < _num_tree_layers; i++) {
        hierarchical_index[i] = row / _patch_sizes[i] * 2 + col / _patch_sizes[i];
        row = row % _patch_sizes[i];
        col = col % _patch_sizes[i];
    }
}

void HPS::GetHierarchicalIndexFromRowCol(int row, int col, vector<int> &hierarchical_index) {
    // 判断row col 能否被block_size整除
    if (row % _patch_sizes[_num_tree_layers - 1] != 0 || col % _patch_sizes[_num_tree_layers - 1] != 0) {
        std::cout << "row or col can not be divided by block_size" << endl;
        return;
    }
    hierarchical_index[0] = row / _patch_sizes[0] * 4 + col / _patch_sizes[0];
    row = row % _patch_sizes[0];
    col = col % _patch_sizes[0];
    for (int i = 1; i < _num_tree_layers; i++) {
        hierarchical_index[i] = row / _patch_sizes[i] * 2 + col / _patch_sizes[i];
        row = row % _patch_sizes[i];
        col = col % _patch_sizes[i];
    }
}

vector<int> HPS::GetHierarchicalIndexFromRowColLevel(int row, int col, int level) {
    vector<int> hirachical_index(level + 1);
    hirachical_index[0] = row / _patch_sizes[0] * 4 + col / _patch_sizes[0];
    row = row % _patch_sizes[0];
    col = col % _patch_sizes[0];
    for (int i = 1; i <= level; i++) {
        hirachical_index[i] = row / _patch_sizes[i] * 2 + col / _patch_sizes[i];
        row = row % _patch_sizes[i];
        col = col % _patch_sizes[i];
    }
    return hirachical_index;
}

vector<int> HPS::GetRowColLevelFromHierarchicalIndex(vector<int> hierarchical_index) {
    int row = 0, col = 0;
    int level = hierarchical_index.size() - 1;
    row += (hierarchical_index[0] / 4) * _patch_sizes[0];
    col += (hierarchical_index[0] % 4) * _patch_sizes[0];
    for (int i = 1; i < hierarchical_index.size(); i++) {
        row += (hierarchical_index[i] / 2) * _patch_sizes[i];
        col += (hierarchical_index[i] % 2) * _patch_sizes[i];
    }
    return {row, col, level};
}

void HPS::InitOrganizeDataCellMap() {
    // 遍历所有最小块的格点
    int min_block_size = _patch_sizes[_num_tree_layers - 1];

    for (int r = 0; r < _img_height; r += min_block_size) {
        for (int c = 0; c < _img_width; c += min_block_size) {
            int hirachical_index[_num_tree_layers];
            GetHierarchicalIndexFromRowCol(r, c, hirachical_index); // 获取当前格点的层次索引
            int dst_offset = GetOffsetOfHierarchicalIndex(hirachical_index);  // 层次索引对应在organized_pointcloud中的偏移量（起始位置）
            for (int rr = 0; rr < min_block_size; ++rr) {
                for (int cc = 0; cc < min_block_size; ++cc) {
                    _organize_data_cell_map[(r+rr) * _img_width + c + cc] = dst_offset+rr*min_block_size+cc;
                }
            }
        }
    }
}


void HPS::OrganizePointCloud() {
    // 遍历所有最小块的格点
    int min_block_size = _patch_sizes[_num_tree_layers - 1];
    int dst_offset = 0;
#ifdef ENABLE_OMP
#pragma omp parallel for firstprivate(min_block_size, dst_offset) default(none) num_threads(8)
#endif
    for (int r = 0; r < _img_height; r += min_block_size) {
        for (int c = 0; c < _img_width; c += min_block_size) {
            // 拷贝一个最小块的点云（并且是一行一行地）到organized_pointcloud
            for (int rr = 0; rr < min_block_size; rr++) {
                dst_offset = _organize_data_cell_map[(r+rr) * _img_width + c];
                _organized_pointcloud.block(dst_offset, 0, min_block_size, 3) =
                        _cloud_array.block((r + rr) * _img_width + c, 0, min_block_size, 3);

            }
        }
    }
}
//void HPS::OrganizePointCloud() {
//#ifdef ENABLE_OMP
//#pragma omp parallel for shared(min_block_size) default(none) num_threads(8)
//#endif
//    for (int i = 0; i < _img_width*_img_height; ++i) {
//        auto dst = _organize_data_cell_map[i];
//        _organized_pointcloud.row(dst) = _cloud_array.row(i);
//    }
//}
//void HPS::OrganizePointCloud() {
//    int mxn = _img_width*_img_height;
//    int mxn2 = 2*mxn;
//    int id;
//#ifdef ENABLE_OMP
//#pragma omp parallel for firstprivate(id, mxn, mxn2) shared(cout) default(none) num_threads(8)
//#endif
//    for(int r=0; r< _img_height; r++){
//        for(int c=0; c< _img_width; c++){
//            int i = r*_img_width+c;
//            id = _organize_data_cell_map[i];
//            *(_organized_pointcloud.data() + id) = *(_cloud_array.data() + i);
//            *(_organized_pointcloud.data() + mxn + id) = *(_cloud_array.data() + mxn + i);
//            *(_organized_pointcloud.data() + mxn2 + id) = *(_cloud_array.data() + mxn2 + i);
//        }
//    }
//}

void HPS::RecursivePlaneSegment(bool debug=false) {
// 读取计算points_sum_pyramid的数据，判断，把结果写到patch_segment_result里
#ifdef ENABLE_OMP
omp_set_num_threads(8);
omp_lock_t lock;
omp_init_lock(&lock);
#pragma omp parallel for default(none) shared(lock, _patch_segment_status)
#endif
    for (int i = 0; i < _num_patches_each_level[0]; i++) {
        vector<vector<int>> wait_to_process;
        wait_to_process.reserve(4 * _num_tree_layers);

        wait_to_process.emplace_back(vector<int>{i});
        while (!wait_to_process.empty()) {
            auto this_ind = wait_to_process.back();
            wait_to_process.pop_back();
            PointsSum point_sum = _points_sum_pyramid.Index(this_ind);

            PlaneParams plane_params = point_sum.FitPlane(false);
#ifdef FALSE
            cout<<"MSE:  "<<plane_params.MSE<<endl;
            cv::Mat image_show = _color_img.clone();
            DrawPatchByHierarchicalIndex(this_ind, image_show, cv::Scalar(255,0,0), 2);
            cv::imshow("this patch", image_show);
            auto key = cv::waitKey();
            if (key == 's') {
                auto patch_points = GetPatchPoints(this_ind);
                DrawPc(patch_points);
                point_sum.FitPlane(true);
            }
#endif

            if (plane_params.IsPlane(_plane_thresh_mode, _plane_thresh)) {
#ifdef ENABLE_OMP
                omp_set_lock(&lock);
#endif
                _patch_segment_status.AddPatch(this_ind, this_ind.size(), plane_params);
#ifdef ENABLE_OMP
                omp_unset_lock(&lock);
#endif
            } else {
                if (this_ind.size() < _num_tree_layers) {
                    for (int j = 0; j < 4; j++) {
                        vector<int> new_ind = this_ind;
                        new_ind.emplace_back(j);
                        wait_to_process.emplace_back(new_ind);
                    }
                }
            }
        }
    }
    if (debug){
        auto show_img = _color_img.clone();
        DrawPatchSegmentResult(show_img);
        cv::imshow("Patch Segment Result(press 's' to show pointcloud)", show_img);
        auto key= cv::waitKey();
        if (key == 's'){
            DrawPc(_cloud_array, show_img);
        }
    }
    _patch_segment_status.SortUnlabeledPatchIndex_SmallestFirst();
}

void HPS::PatchwiseRegionGrowing(bool debug=false) {
    int cnt = 0;
    _region_plane_params.emplace_back();  // 由于0代表默认未分配的label，所以这里添加一个空的占位，保证label和plane_params的索引一致
    _region_indexes.emplace_back();       // 同上，占零位
    while (1) {
        vector<int> seed_index = _patch_segment_status.GetRemainingLargestPatchIndex();
        if (seed_index.empty()) break;

        vector<vector<int>> indexes;
        indexes.emplace_back(seed_index);
        _patch_segment_status.ResetVisited();

        RecursiveRegionGrowing(seed_index, indexes);

        if(_RG_filter_by_area and GetRegionArea(indexes) < _RG_min_area);
        else{    // 当前得到的区域面积不小，则新增一个新的连通区域
            ++cnt;
            _patch_segment_status.SetPatchLabel(indexes, cnt);
            _region_plane_params.push_back(GetRegionPlaneParameter(indexes));
            _region_indexes.push_back(indexes);
        }
    }
    _region_count = cnt;
    ConvertPatchSegmentStatusToGrid();              // 从此之后可以使用 _min_patch_label_grid
    cv::resize(_min_patch_label_grid, _final_pixel_label, _final_pixel_label.size(), 0, 0, cv::INTER_NEAREST);

    if (debug){
        cv::Mat show_img = cv::Mat::zeros(_img_height, _img_width, CV_8UC3);
        for (int i=1; i <= _region_count; i++){     // 用_region_indexes，而不用_min_patch_label_grid，主要为了方便画边界
            auto color = _debug_color_label_lut.at<cv::Vec3b>(i);
            DrawPatchByHierarchicalIndex(_region_indexes[i], show_img, color, -1);
            DrawPatchByHierarchicalIndex(_region_indexes[i], show_img, (0,0,255), 2);
        }
        DrawPatchSegmentResult(show_img);
        cv::addWeighted(_color_img, 0.3, show_img, 0.7, 0, show_img);
        cv::imshow("Patchwise Region Growing Result (press 's' to show pointclond)", show_img);
        auto key = cv::waitKey();
        if (key == 's'){
            DrawPc(_cloud_array, show_img);
        }
    }
}

void HPS::RecursiveRegionGrowing(const vector<int> &seed_index, vector<vector<int>> &indexes) {
    _patch_segment_status.SetPatchVisited(seed_index);
//    _patch_segment_status.SetPatchLabel(seed_index, label);
    vector<vector<int>> possible_neighbor_indexs = GetAllPossibleSmallerNeighborPatchIndexes(seed_index);
    vector<vector<int>> wait_to_process;
    for (const auto& possible_neighbor_index : possible_neighbor_indexs) {
//        if (!m_patch_segment_result.Has(possible_neighbor_index)) continue;
        vector<int> most_similar_patch_index = _patch_segment_status.GetMostSimilarPatchIndex(possible_neighbor_index);
        if (most_similar_patch_index.empty()) continue;
        if (_patch_segment_status.IsVisited(most_similar_patch_index)) continue;
        if (_patch_segment_status.IsLabeled(most_similar_patch_index)) continue;

        _patch_segment_status.SetPatchVisited(most_similar_patch_index);

        PlaneParams seed_plane_params = _patch_segment_status.GetPatchPlaneParameter(seed_index);
        PlaneParams neighbor_plane_params = _patch_segment_status.GetPatchPlaneParameter(most_similar_patch_index);
#ifdef False
        DrawPatchByHierarchicalIndex(most_similar_patch_index, _color_img, g_color, 3);
        cv::imshow("neighbor", _color_img);
        cv::waitKey(1);
#endif
        if (PlaneParams::IsCoplanar(seed_plane_params, neighbor_plane_params,
                                    _plane_merge_min_cos_angle, _plane_merge_thresh_mode, _plane_merge_max_distance2_thresh)) {
            indexes.emplace_back(most_similar_patch_index);
            wait_to_process.emplace_back(most_similar_patch_index);
#ifdef False
            DrawPatchByHierarchicalIndex(most_similar_patch_index, debug_img_show, g_color, -1);
//            cv::imshow("RG", debug_img_show);
//            cv::waitKey(1);
#endif
        }
    }
    for(const auto& new_seed : wait_to_process){
        RecursiveRegionGrowing(new_seed, indexes);
    }

}


vector<vector<int>> HPS::GetAllPossibleSmallerNeighborPatchIndexes(const vector<int>& ind) {
    vector<vector<int>> neighbors;
    for (int i = ind.size()-1; i < _num_tree_layers; i++) {
        vector<vector<int>> neighbors_level_i = GetPossibleNeighborPatchIndexes(ind, i);
        for (const auto& neighbor:neighbors_level_i) {
            neighbors.emplace_back(neighbor);
        }
    }
    return neighbors;
}

vector<vector<int>> HPS::GetPossibleNeighborPatchIndexes(const vector<int>& ind, int level_i) {
    vector<vector<int>> neighbors;
    vector<int> r_c_level_index= GetRowColLevelFromHierarchicalIndex(ind);
    int r(r_c_level_index[0]), c(r_c_level_index[1]);
    int level(r_c_level_index[2]);
    int r_nb_u, r_nb_d, c_nb_l, c_nb_r;

    if (r >= _patch_sizes[level_i]) {
        r_nb_u = r - _patch_sizes[level_i];
        for (int c_nb_u = c - _patch_sizes[level_i]; c_nb_u <= c + _patch_sizes[level]; c_nb_u+=_patch_sizes[level_i]) {
            if ((0<=c_nb_u) and (c_nb_u < _img_width)){
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_u, c_nb_u, level_i));
            }
        }
    }

    r_nb_d = r + _patch_sizes[level];
    if (r + _patch_sizes[level] < _img_height) {
        for (int c_nb_d = c - _patch_sizes[level_i]; c_nb_d <= c + _patch_sizes[level]; c_nb_d+=_patch_sizes[level_i]) {
            if ((0<=c_nb_d) and (c_nb_d < _img_width)){
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_d, c_nb_d, level_i));
            }
        }
    }

    if (c >= _patch_sizes[level_i]) {
        c_nb_l = c - _patch_sizes[level_i];
        for (int r_nb_l = r; r_nb_l < r + _patch_sizes[level]; r_nb_l+=_patch_sizes[level_i]) {
            if (r_nb_l < _img_height){
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_l, c_nb_l, level_i));
            }
        }
    }

    c_nb_r = c + _patch_sizes[level];
    if (c_nb_r < _img_width) {
        for (int r_nb_r = r; r_nb_r < r + _patch_sizes[level]; r_nb_r+=_patch_sizes[level_i]) {
            if (r_nb_r < _img_height) {
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_r, c_nb_r, level_i));
            }
        }
    }
    return neighbors;
}


void HPS::DrawPatchByHierarchicalIndex(const vector<int> &hierarchical_index, cv::Mat &image, const cv::Scalar& color, int thickness) {
    vector<int> row_col_level = GetRowColLevelFromHierarchicalIndex(hierarchical_index);
    int patch_size = _patch_sizes[row_col_level[2]];
    cv::rectangle(image, cv::Rect(row_col_level[1], row_col_level[0], patch_size, patch_size), color, thickness);
}

void HPS::DrawPatchByHierarchicalIndex(const vector<vector<int>> &hierarchical_index, cv::Mat &image, const cv::Scalar& color, int thickness) {
    for (const auto& index : hierarchical_index) {
        DrawPatchByHierarchicalIndex(index, image, color, thickness);
    }
}

void HPS::DrawPatchSegmentResult(cv::Mat &img_copy) {
    vector<vector<int>> wait_to_draw;
    wait_to_draw.reserve(4 * _num_tree_layers);
    for (int i = 0; i < _num_patches_each_level[0]; i++) {
        wait_to_draw.emplace_back(vector<int>{i});
        while (!wait_to_draw.empty()) {
            auto this_ind = wait_to_draw.back();
            wait_to_draw.pop_back();
            if (_patch_segment_status.GetPatchSegmentLevel(this_ind) == this_ind.size()) {
                DrawPatchByHierarchicalIndex(this_ind, img_copy, cv::Scalar(0, 0, 255), 1);
            } else {
                if (this_ind.size() < _num_tree_layers) {
                    for (int j = 0; j < 4; j++) {
                        vector<int> new_ind = this_ind;
                        new_ind.emplace_back(j);
                        wait_to_draw.emplace_back(new_ind);
                    }
                }
            }
        }
    }
}

Eigen::MatrixXf HPS::GetPatchPoints(const vector<int> &ind) {
    int offset = GetOffsetOfHierarchicalIndex(ind);
    int patch_size = _patch_sizes[ind.size() - 1];
    return _organized_pointcloud.block(offset, 0, patch_size * patch_size, 3);
}

PlaneParams HPS::GetRegionPlaneParameter(const vector<vector<int>>& indexes) {
    PointsSum region_sum;
    for(const auto& ind : indexes){
        region_sum += _points_sum_pyramid.Index(ind);
    }
    return region_sum.FitPlane(false);
}

void HPS::BoundaryRefine(bool debug=false) {
    // 从已有的区域周围开始，将周围一圈的patch作为待细分区域
    cv::Mat mask_to_refine_region = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_to_refine_all = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_region = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_dilated = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_eroded = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat kernel_rect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    mask_to_refine_all.setTo(0);
    for(int label=1; label <= _region_count; label++) {
        mask_to_refine_region.setTo(0);
        mask_region.setTo(0);
        mask_region.setTo(255, _min_patch_label_grid == label);
        cv::dilate(mask_region, mask_dilated, kernel_rect,cv::Point(-1,-1),1);
        cv::erode(mask_region, mask_eroded, kernel_rect);
        mask_to_refine_region = mask_dilated - mask_eroded;
        cv::bitwise_or(mask_to_refine_region, mask_to_refine_all, mask_to_refine_all);
//        _min_patch_label_grid_eroded.setTo(label, mask_eroded);
    }

    // debug 查看待细分的所有像素
    if (debug){
        cv::Mat show_img1;
        cv::resize(mask_to_refine_all, show_img1, _color_img.size(), 0, 0, cv::INTER_NEAREST);
        cv::cvtColor(show_img1, show_img1, cv::COLOR_GRAY2BGR);

        cv::Mat show_img2;
        cv::resize(_min_patch_label_grid, show_img2, _color_img.size(), 0, 0, cv::INTER_NEAREST);
        cv::cvtColor(show_img2, show_img2, cv::COLOR_GRAY2BGR);
        cv::LUT(show_img2, _debug_color_label_lut, show_img2);
        cv::imshow("_min_patch_label_grid", show_img2);

        cv::addWeighted(show_img1, 0.3, show_img2, 0.7, 0, show_img1);
        cv::addWeighted(_color_img, 0.2, show_img1, 0.8, 0, show_img1);
        cv::imshow("Boundary Refine Mask", show_img1);
        cv::waitKey(1);
    }

    // mask里的像素细分
    RefinePixelsAndMergePlane(mask_to_refine_all);

    if (debug){
        cv::Mat show_img;
        cv::cvtColor(_final_pixel_label, show_img, cv::COLOR_GRAY2BGR);
        cv::LUT(show_img, _debug_color_label_lut, show_img);
        cv::addWeighted(_color_img, 0.2, show_img, 0.8, 0, show_img);
        cv::imshow("Final Pixel Label", show_img);
        cv::waitKey();
    }
}

/**********************************************************************************************************************
老版本，从已有的区域周围开始，将周围一圈的所有点判断共面，分配label，但问题在于一个像素可能会被两个区域邻接，导致反复刷漆，效果差
void HPS::BoundaryRefine() {
    ConvertPatchSegmentStatusToGrid();
    cv::Mat mask = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_eroded = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_dilated = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat mask_to_refine = cv::Mat(_min_patch_label_grid.size(), CV_8UC1);
    cv::Mat kernel_rect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    for(int label=1; label <= _region_count; label++){
        mask.setTo(0);
        mask.setTo(255, _min_patch_label_grid == label);

        cv::erode(mask, mask_eroded, kernel_rect);
        cv::dilate(mask, mask_dilated, kernel_rect);
        mask_to_refine = mask_dilated - mask_eroded;

#ifdef TRUE
        cv::Mat temp;
        cv::resize(mask_to_refine, temp, debug_img_show.size(), 0, 0, cv::INTER_NEAREST);
        cv::cvtColor(temp, temp, cv::COLOR_GRAY2BGR);
        cv::addWeighted(_color_img, 0.5, temp, 0.5, 0, temp);
        cv::imshow("mask_to_refine", temp);
        cv::imshow("debug_img_show", debug_img_show);
//        cv::waitKey(0);
#endif
        _min_patch_label_grid_eroded.setTo(label, mask_eroded);
        RefinePixelsAndMergePlane(mask_to_refine, label);
    }
}
**********************************************************************************************************************/

void HPS::ConvertPatchSegmentStatusToGrid() {
    for (int label=1; label <= _region_count; label++) {
        vector<vector<int>> indexes = _region_indexes[label];
        for(const auto& index : indexes){
            vector<int> row_col_level = GetRowColLevelFromHierarchicalIndex(index);
            int row = row_col_level[0] / _patch_sizes[_num_tree_layers - 1];
            int col = row_col_level[1] / _patch_sizes[_num_tree_layers - 1];
            int draw_grid_size = _patch_sizes[row_col_level[2]] / _patch_sizes[_num_tree_layers - 1];
            cv::rectangle(_min_patch_label_grid, cv::Rect(col, row, draw_grid_size, draw_grid_size), label, -1);
        }
    }
}

vector<int> HPS::FindLabelsInNNeighbor(int row, int col, int num_neighbor=3) {
    vector<int> result{};
    for (int r = row-num_neighbor/2; r <= row+num_neighbor/2; r++){
        for (int c = col-num_neighbor/2; c <= col+num_neighbor/2; c++){
            if (r>=0 and r < _min_patch_label_grid.rows and c >= 0 and c < _min_patch_label_grid.cols){
                int label = *_min_patch_label_grid.ptr<uchar>(r, c);
                if (label!=0) result.emplace_back(label);
            }
        }
    }
    sort(result.begin(), result.end());
    result.erase(unique(result.begin(), result.end()), result.end());
    return result;
}

void HPS::RefinePixelsAndMergePlane(const cv::Mat &mask_to_refine) {
    int min_patch_size = _patch_sizes[_num_tree_layers - 1];
    for (int r = 0; r < mask_to_refine.rows; ++r) {
        for (int c = 0; c < mask_to_refine.cols; ++c) {
            if (! *mask_to_refine.ptr<uchar>(r, c)) continue;

            vector<int> neighbor_label = FindLabelsInNNeighbor(r, c, _merge_plane_NN_window_size);

            // 判断共面、合并label
            RecordMergePlane(neighbor_label);

            // 计算当前patch的像素点到所有邻接平面的距离
            Eigen::MatrixXf distances_to_planes(min_patch_size*min_patch_size, neighbor_label.size());
            for (int i = 0; i < neighbor_label.size(); ++i) {
                PlaneParams plane_params = _region_plane_params[neighbor_label[i]];
                auto points = GetPatchPoints(GetHierarchicalIndexFromRowColLevel(r*min_patch_size, c*min_patch_size, _num_tree_layers - 1));
                distances_to_planes.col(i) = Eigen::abs(points.col(0).array()*plane_params.a
                                                        + points.col(1).array()*plane_params.b
                                                        + points.col(2).array()*plane_params.c
                                                        + plane_params.d);
            }

            // arg min
            Eigen::VectorX<Eigen::Index> argmin{distances_to_planes.rows()};
            for(auto row=0; row<distances_to_planes.rows(); row++){
                distances_to_planes.row(row).minCoeff(&argmin[row]);
            }

            // set pixel labels
            int offset_r, offset_c;
            offset_r = r * min_patch_size;
            offset_c = c * min_patch_size;
            for (int i = 0; i < min_patch_size * min_patch_size; ++i) {
                if (distances_to_planes(i, argmin[i]) < _point_coplane_thresh){
                    auto pixel_label = neighbor_label[argmin[i]];
                    _final_pixel_label.at<uchar>(offset_r + i / min_patch_size, offset_c + i % min_patch_size) = pixel_label;
                }
            }
        }
    }
    cv::LUT(_final_pixel_label, _change_label_lut, _final_pixel_label);
}

void HPS::RecordMergePlane(vector<int> labels) {
    if(labels.size() < 2) return;
    int label_size = labels.size();
    for (int i = 0; i < label_size-1; ++i) {
        for (int j = i+1; j <= label_size-1; ++j) {
            if(PlaneParams::IsCoplanar(_region_plane_params[labels[i]], _region_plane_params[labels[j]],
                                       _plane_merge_min_cos_angle, _plane_merge_thresh_mode, _plane_merge_max_distance2_thresh)){
                int target, from;
                if (_region_plane_params[labels[i]].num_points >= _region_plane_params[labels[j]].num_points){
                    target=labels[i];
                    from=labels[j];
                }
                else{
                    target=labels[j];
                    from=labels[i];
                }
                // 递归找到最终的target
                while(_change_label_lut.at<uchar>(0, target) != target){
                    target = _change_label_lut.at<uchar>(0, target);
                }
                _change_label_lut.at<uchar>(0, from) = target;
            }
        }
    }
}
