#include "HPS.h"
#include "utils.h"
#include "PointsInfoPyramid.h"
#include "PatchSegmentResult.h"
#include "PlaneSeg.h"

HPS::HPS(int img_hight, int img_width, int block_num_x, int block_num_y, int tree_level) :
        m_img_hight(img_hight), m_img_width(img_width),
        m_block_num_x(block_num_x), m_block_num_y(block_num_y),
        m_num_tree_layers(tree_level) {

    m_patchs_each_level = new int[tree_level];
    m_patchs_each_level[0] = 12;
    for (int i = 1; i < tree_level; i++) m_patchs_each_level[i] = 4;

    m_block_sizes = new int[tree_level];
    int temp = img_width / block_num_x;
    m_block_sizes[0] = temp;
    for (int i = 1; i < tree_level; ++i) {
        temp /= 2;
        m_block_sizes[i] = temp;
    }

    m_points_sum_pyramid.Initialize(m_patchs_each_level, m_num_tree_layers, m_block_sizes[m_num_tree_layers - 1]);
    m_patch_segment_result.Initialize(m_patchs_each_level, m_num_tree_layers);
}


void HPS::Process() {
    auto t1 = cv::getTickCount();

    OrganizePointCloud();
    m_points_sum_pyramid.PreComputeSum(m_organized_pointcloud);
    RecursivePlaneSegment();

    auto t2 = cv::getTickCount();

    auto fps = cv::getTickFrequency() / (t2 - t1);
    std::cout << "FPS: " << fps << endl;

    int ind[]{0};
    m_points_sum_pyramid.Index(ind, 1).sum_x = 1;

    vector<int> ind_test;
    ind_test.emplace_back(0);
    PointsSum out = m_points_sum_pyramid.Index(ind_test);


}

void HPS::GetHirachicalIndexFromRowCol(int row, int col, int *hirachical_index) {
//     判断row col 能否被block_size整除
    if (row % m_block_sizes[m_num_tree_layers - 1] != 0 || col % m_block_sizes[m_num_tree_layers - 1] != 0) {
        std::cout << "row or col can not be divided by block_size" << endl;
        return;
    }
    hirachical_index[0] = row / m_block_sizes[0] * 4 + col / m_block_sizes[0];
    row = row % m_block_sizes[0];
    col = col % m_block_sizes[0];
    for (int i = 1; i < m_num_tree_layers; i++) {
        hirachical_index[i] = row / m_block_sizes[i] * 2 + col / m_block_sizes[i];
        row = row % m_block_sizes[i];
        col = col % m_block_sizes[i];
    }
}

vector<int> HPS::GetRowColFromHirachicalIndex(vector<int> hirachical_index) {
    int row = 0, col = 0;
    row += (hirachical_index[0] / 4) * m_block_sizes[0];
    col += (hirachical_index[0] % 4) * m_block_sizes[0];
    for (int i = 1; i < hirachical_index.size(); i++) {
        row += (hirachical_index[i] / 2) * m_block_sizes[i];
        col += (hirachical_index[i] % 2) * m_block_sizes[i];
    }
    return vector<int>{row, col};
}


void HPS::OrganizePointCloud() {
    m_organized_pointcloud.setZero(m_img_hight * m_img_width, 3);
    // 遍历所有最小块的格点
    int min_block_size = m_block_sizes[m_num_tree_layers - 1];
#pragma omp parallel for shared(min_block_size) default(none)
    for (int r = 0; r < m_img_hight; r += min_block_size) {
        for (int c = 0; c < m_img_width; c += min_block_size) {
            int hirachical_index[m_num_tree_layers];
            GetHirachicalIndexFromRowCol(r, c, hirachical_index); // 获取当前格点的层次索引
            int dst_offset = GetOffsetOfHirachicalIndex(hirachical_index);  // 层次索引对应在organized_pointcloud中的偏移量（起始位置）
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
omp_set_num_threads(12);
#pragma omp parallel for default(none)
    for (int i = 0; i < 12; i++) {
        vector<vector<int>> wait_to_process;
        wait_to_process.reserve(4 * m_num_tree_layers);

        wait_to_process.emplace_back(vector<int>{i});
        while (!wait_to_process.empty()) {
            auto this_ind = wait_to_process.back();
            wait_to_process.pop_back();
            PointsSum points_sum = m_points_sum_pyramid.Index(this_ind);

            PlaneParams plane_params = points_sum.FitPlane();
            if(PointsSum::IsPlane(plane_params)){
                m_patch_segment_result.Set(this_ind, this_ind.size(), plane_params);
#ifdef DEBUG
                DrawPatchByHierachicalIndex(this_ind, m_gray_img);
#endif
            }
            else{
                if(this_ind.size() < m_num_tree_layers){
                    for(int j = 0; j < 4; j++){
                        vector<int> new_ind = this_ind;
                        new_ind.emplace_back(j);
                        wait_to_process.emplace_back(new_ind);
                    }
                }
            }
        }
    }
#ifdef DEBUG
    cv::imshow("img_copy", m_gray_img);
    cv::waitKey();
#endif
}

void HPS::DrawPatchByHierachicalIndex(const vector<int> &hierachical_index, cv::Mat &img_copy) {
    vector<int> row_col = GetRowColFromHirachicalIndex(hierachical_index);
    int patch_size = m_block_sizes[hierachical_index.size() - 1];
    cv::rectangle(img_copy, cv::Rect(row_col[1], row_col[0], patch_size, patch_size), cv::Scalar(0, 0, 255), 2);
}

