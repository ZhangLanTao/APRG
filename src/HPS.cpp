#include "HPS.h"
#include "utils.h"
#include "PointsSumPyramid.h"
#include "PatchSegmentResult.h"
#include "PlaneSeg.h"
#include <iostream>

HPS::HPS(int img_height, int img_width, int block_num_x, int block_num_y, int tree_level) :
        m_img_height(img_height), m_img_width(img_width),
        m_block_num_x(block_num_x), m_block_num_y(block_num_y),
        m_num_tree_layers(tree_level) {

    m_patches_each_level = new int[tree_level];
    m_patches_each_level[0] = m_block_num_x * m_block_num_y;
    for (int i = 1; i < tree_level; i++) m_patches_each_level[i] = 4;

    m_patch_sizes = new int[tree_level];
    int temp = img_width / block_num_x;
    m_patch_sizes[0] = temp;
    for (int i = 1; i < tree_level; ++i) {
        temp /= 2;
        m_patch_sizes[i] = temp;
    }

    m_points_sum_pyramid.Initialize(m_patches_each_level, m_num_tree_layers, m_patch_sizes[m_num_tree_layers - 1]);
    m_patch_segment_result.Initialize(m_patches_each_level, m_num_tree_layers);
}


void HPS::Process() {
    OrganizePointCloud();
    auto t1 = cv::getTickCount();
    m_points_sum_pyramid.PreComputeSum(m_organized_pointcloud);
    RecursivePlaneSegment();
    PatchwiseRegionGrowing();
    auto t2 = cv::getTickCount();


    auto fps = cv::getTickFrequency() / (t2 - t1);
    std::cout << "FPS: " << fps << endl;
}

void HPS::GetHierarchicalIndexFromRowCol(int row, int col, int *hierarchical_index) {
//     判断row col 能否被block_size整除
    if (row % m_patch_sizes[m_num_tree_layers - 1] != 0 || col % m_patch_sizes[m_num_tree_layers - 1] != 0) {
        std::cout << "row or col can not be divided by block_size" << endl;
        return;
    }
    hierarchical_index[0] = row / m_patch_sizes[0] * 4 + col / m_patch_sizes[0];
    row = row % m_patch_sizes[0];
    col = col % m_patch_sizes[0];
    for (int i = 1; i < m_num_tree_layers; i++) {
        hierarchical_index[i] = row / m_patch_sizes[i] * 2 + col / m_patch_sizes[i];
        row = row % m_patch_sizes[i];
        col = col % m_patch_sizes[i];
    }
}

void HPS::GetHierarchicalIndexFromRowCol(int row, int col, vector<int> hierarchical_index) {
    // 判断row col 能否被block_size整除
    if (row % m_patch_sizes[m_num_tree_layers - 1] != 0 || col % m_patch_sizes[m_num_tree_layers - 1] != 0) {
        std::cout << "row or col can not be divided by block_size" << endl;
        return;
    }
    hierarchical_index[0] = row / m_patch_sizes[0] * 4 + col / m_patch_sizes[0];
    row = row % m_patch_sizes[0];
    col = col % m_patch_sizes[0];
    for (int i = 1; i < m_num_tree_layers; i++) {
        hierarchical_index[i] = row / m_patch_sizes[i] * 2 + col / m_patch_sizes[i];
        row = row % m_patch_sizes[i];
        col = col % m_patch_sizes[i];
    }
}

vector<int> HPS::GetHierarchicalIndexFromRowColLevel(int row, int col, int level) {
    vector<int> hirachical_index(level + 1);
    hirachical_index[0] = row / m_patch_sizes[0] * 4 + col / m_patch_sizes[0];
    row = row % m_patch_sizes[0];
    col = col % m_patch_sizes[0];
    for (int i = 1; i <= level; i++) {
        hirachical_index[i] = row / m_patch_sizes[i] * 2 + col / m_patch_sizes[i];
        row = row % m_patch_sizes[i];
        col = col % m_patch_sizes[i];
    }
    return hirachical_index;
}

vector<int> HPS::GetRowColLevelFromHierarchicalIndex(vector<int> hierarchical_index) {
    int row = 0, col = 0, level = hierarchical_index.size() - 1;
    row += (hierarchical_index[0] / 4) * m_patch_sizes[0];
    col += (hierarchical_index[0] % 4) * m_patch_sizes[0];
    for (int i = 1; i < hierarchical_index.size(); i++) {
        row += (hierarchical_index[i] / 2) * m_patch_sizes[i];
        col += (hierarchical_index[i] % 2) * m_patch_sizes[i];
    }
    return vector<int>{row, col, level};
}


void HPS::OrganizePointCloud() {
    m_organized_pointcloud.setZero(m_img_height * m_img_width, 3);
    // 遍历所有最小块的格点
    int min_block_size = m_patch_sizes[m_num_tree_layers - 1];
#pragma omp parallel for shared(min_block_size) default(none)
    for (int r = 0; r < m_img_height; r += min_block_size) {
        for (int c = 0; c < m_img_width; c += min_block_size) {
//            vector<int> hirachical_index(m_num_tree_layers);
            int hirachical_index[m_num_tree_layers];
            GetHierarchicalIndexFromRowCol(r, c, hirachical_index); // 获取当前格点的层次索引
            int dst_offset = GetOffsetOfHierarchicalIndex(hirachical_index);  // 层次索引对应在organized_pointcloud中的偏移量（起始位置）
            // 拷贝一个最小块的点云到organized_pointcloud
            for (int rr = 0; rr < min_block_size; rr++) {
                m_organized_pointcloud.block(dst_offset, 0, min_block_size, 3) =
                        m_cloud_array.block((r + rr) * m_img_width + c, 0, min_block_size, 3);
                dst_offset += min_block_size;
            }
        }
    }
}

void HPS::RecursivePlaneSegment() {
// 读取计算points_sum_pyramid的数据，判断，把结果写到patch_segment_result里
#ifdef RELEASE
omp_set_num_threads(8);
#pragma omp parallel for default(none)
#endif
    for (int i = 0; i < m_patches_each_level[0]; i++) {
        vector<vector<int>> wait_to_process;
        wait_to_process.reserve(4 * m_num_tree_layers);

        wait_to_process.emplace_back(vector<int>{i});
        while (!wait_to_process.empty()) {
            auto this_ind = wait_to_process.back();
            wait_to_process.pop_back();
            PointsSum point_sum = m_points_sum_pyramid.Index(this_ind);

            PlaneParams plane_params = point_sum.FitPlane();
#ifdef false
            if (plane_params.MSE<0){
                cv::Mat image_show = m_gray_img.clone();
                DrawPatchByHierarchicalIndex(this_ind, image_show, cv::Scalar(255,0,0), 2);
                cv::imshow("MSE<0", image_show);
                cv::waitKey();
                Eigen::MatrixXf patch_points = GetPatchPoints(this_ind);
                DrawPc(patch_points);
            }
#endif

            if (plane_params.IsPlane()) {
#ifdef false
                std::cout<<"MSE:  "<<plane_params.MSE<<"  Is plane"<<std::endl;
                cv::Mat image_show = m_gray_img.clone();
                DrawPatchByHierarchicalIndex(this_ind, image_show, cv::Scalar(255,0,0), 2);
                cv::imshow("Is / Isn't plane", image_show);
                cv::waitKey(100);
                Eigen::MatrixXf patch_points = GetPatchPoints(this_ind);
                DrawPc(patch_points);
#endif
                m_patch_segment_result.SetPatchSegmentResult(this_ind, this_ind.size(), plane_params);
//#ifdef DEBUG
                // DrawPatchByHierarchicalIndex(this_ind, m_gray_img);
//#endif
            } else {
#ifdef false
                std::cout<<"MSE:  "<<plane_params.MSE<<"  Is not plane"<<std::endl;
                cv::Mat image_show = m_gray_img.clone();
                DrawPatchByHierarchicalIndex(this_ind, image_show, cv::Scalar(255,0,0), 2);
                cv::imshow("Is / Isn't plane", image_show);
                cv::waitKey(100);
                Eigen::MatrixXf patch_points = GetPatchPoints(this_ind);
                DrawPc(patch_points);
#endif
                if (this_ind.size() < m_num_tree_layers) {
                    for (int j = 0; j < 4; j++) {
                        vector<int> new_ind = this_ind;
                        new_ind.emplace_back(j);
                        wait_to_process.emplace_back(new_ind);
                    }
                }
            }
        }
    }
#ifdef DEBUG
    DrawPatchSegmentResult(m_gray_img);
    cv::imshow("img_copy", m_gray_img);
    cv::waitKey();
#endif
}

#ifdef DEBUG
cv::Scalar color;
#endif
void HPS::PatchwiseRegionGrowing() {
    unsigned short cnt = 0;
    while (1) {
        vector<int> seed_index = m_patch_segment_result.GetRemainingLargestPatchIndex();
        if (seed_index.empty()) break;

        // 还有seed可选，则认新增一个新的连通区域
        ++cnt;
        m_patch_segment_result.ResetVisited();
#ifdef DEBUG
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        DrawPatchByHierarchicalIndex(seed_index, m_gray_img, color, -1);
#endif
        RecursiveRegionGrowing(seed_index, cnt);

    }

}

void HPS::RecursiveRegionGrowing(const vector<int>& seed_index, unsigned short label) {
    m_patch_segment_result.SetPatchLabel(seed_index, label);
    m_patch_segment_result.SetPatchVisited(seed_index);
    vector<vector<int>> neighbor_indexs = GetAllPossibleSmallerNeighborPatchIndexes(seed_index);

    for (auto neighbor_index : neighbor_indexs) {
//        if (!m_patch_segment_result.Has(neighbor_index)) continue;
        vector<int> most_similar_patch_index = m_patch_segment_result.GetMostSimilarPatchIndex(neighbor_index);
        if (most_similar_patch_index.empty()) continue;
        if (m_patch_segment_result.IsVisited(most_similar_patch_index)) continue;
        if (m_patch_segment_result.IsLabled(most_similar_patch_index)) continue;

        m_patch_segment_result.SetPatchVisited(most_similar_patch_index);

        PlaneParams seed_plane_params = m_patch_segment_result.GetPatchPlaneParameter(seed_index);
        PlaneParams neighbor_plane_params = m_patch_segment_result.GetPatchPlaneParameter(most_similar_patch_index);
#ifdef DEBUG
        DrawPatchByHierarchicalIndex(most_similar_patch_index, m_gray_img, color, 3);
        cv::imshow("neighbor", m_gray_img);
        cv::waitKey(1);
#endif
        if (PlaneParams::IsCoplanar(seed_plane_params, neighbor_plane_params)) {
            m_patch_segment_result.SetPatchLabel(most_similar_patch_index, label);
#ifdef DEBUG
            DrawPatchByHierarchicalIndex(most_similar_patch_index, m_gray_img, color, -1);
            cv::imshow("coplane", m_gray_img);
            cv::waitKey(1);
#endif
            RecursiveRegionGrowing(most_similar_patch_index, label);
        }
    }

}


vector<vector<int>> HPS::GetAllPossibleSmallerNeighborPatchIndexes(const vector<int>& ind) {
    vector<vector<int>> neighbors;
    for (int i = ind.size()-1; i < m_num_tree_layers; i++) {
        vector<vector<int>> neighbors_level_i = GetPossibleNeighborPatchIndexes(ind, i);
        for (const auto& neighbor:neighbors_level_i) {
            neighbors.emplace_back(neighbor);
        }
    }
    return neighbors;
}

vector<vector<int>> HPS::GetPossibleNeighborPatchIndexes(const vector<int> ind, unsigned short level_i) {
    vector<vector<int>> neighbors;
    vector<int> r_c_level_index= GetRowColLevelFromHierarchicalIndex(ind);
    int r(r_c_level_index[0]), c(r_c_level_index[1]), level(r_c_level_index[2]);
    int r_nb_u, r_nb_d, c_nb_l, c_nb_r;

    r_nb_u = r - m_patch_sizes[level_i];
    if (r_nb_u >= 0) {
        for (int c_nb_u = c; c_nb_u < c + m_patch_sizes[level]; c_nb_u+=m_patch_sizes[level_i]) {
            if (c_nb_u < m_img_width){
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_u, c_nb_u, level_i));
            }
        }
    }

    r_nb_d = r + m_patch_sizes[level];
    if (r_nb_d < m_img_height) {
        for (int c_nb_d = c; c_nb_d < c + m_patch_sizes[level]; c_nb_d+=m_patch_sizes[level_i]) {
            if (c_nb_d < m_img_width){
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_d, c_nb_d, level_i));
            }
        }
    }

    c_nb_l = c - m_patch_sizes[level_i];
    if (c_nb_l >= 0) {
        for (int r_nb_l = r; r_nb_l < r + m_patch_sizes[level]; r_nb_l+=m_patch_sizes[level_i]) {
            if (r_nb_l < m_img_height){
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_l, c_nb_l, level_i));
            }
        }
    }

    c_nb_r = c + m_patch_sizes[level];
    if (c_nb_r < m_img_width) {
        for (int r_nb_r = r; r_nb_r < r + m_patch_sizes[level]; r_nb_r+=m_patch_sizes[level_i]) {
            if (r_nb_r < m_img_height) {
                neighbors.emplace_back(GetHierarchicalIndexFromRowColLevel(r_nb_r, c_nb_r, level_i));
            }
        }
    }
    return neighbors;
}



void HPS::DrawPatchByHierarchicalIndex(const vector<int> &hierarchical_index, cv::Mat &img_copy, cv::Scalar color, int thickness) {
    vector<int> row_col_level = GetRowColLevelFromHierarchicalIndex(hierarchical_index);
    int patch_size = m_patch_sizes[row_col_level[2]];
    cv::rectangle(img_copy, cv::Rect(row_col_level[1], row_col_level[0], patch_size, patch_size), color, thickness);
}

void HPS::DrawPatchSegmentResult(cv::Mat &img_copy) {
    vector<vector<int>> wait_to_draw;
    wait_to_draw.reserve(4 * m_num_tree_layers);
    for (int i = 0; i < m_patches_each_level[0]; i++) {
        wait_to_draw.emplace_back(vector<int>{i});
        while (!wait_to_draw.empty()) {
            auto this_ind = wait_to_draw.back();
            wait_to_draw.pop_back();
            if (m_patch_segment_result.GetPatchSegmentLevel(this_ind) == this_ind.size()) {
                DrawPatchByHierarchicalIndex(this_ind, m_gray_img, cv::Scalar(0, 0, 255), 2);
            } else {
                if (this_ind.size() < m_num_tree_layers) {
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
    int patch_size = m_patch_sizes[ind.size()-1];
    return m_organized_pointcloud.block(offset, 0, patch_size * patch_size, 3);
}
