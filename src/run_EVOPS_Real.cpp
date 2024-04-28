#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "EVOPS_data_tools.hpp"
#include "HPS.h"

using std::vector, std::string, std::stringstream;
using std::cout, std::endl, std::setw, std::setfill;
using EVOPS_TUM::fx, EVOPS_TUM::fy, EVOPS_TUM::cx, EVOPS_TUM::cy, EVOPS_TUM::depth_scale;


int main() {
//    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Real(rename)/full_tum_desk/rgbd_dataset_freiburg1_desk/";
//    int data_frame_num = 595;

//    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Real(rename)/full_tum_long_office_val/rgbd_dataset_freiburg3_long_office_household_validation/";
//    int data_frame_num = 2600;

//    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Real(rename)/full_tum_pioner/rgbd_dataset_freiburg2_pioneer_slam/";
//    int data_frame_num = 2866;

    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Real(rename)/tum_freiburg3_cabinet/rgbd_dataset_freiburg3_cabinet/";
    int data_frame_num = 1119;


    //存储用时文件
#ifdef LOG_TIME
    std::string fn_out;
    std::ofstream log_time;
    fn_out = "HPS_time.txt";
    log_time.open(fn_out.c_str(), std::ios::out);
#endif
//    std::string fn_out;
//    std::ofstream log_fitplane_count;
//    fn_out = "FitPlaneCount.txt";
//    log_fitplane_count.open(fn_out.c_str(), std::ios::out);

    for (auto i=0; i<data_frame_num; i++){
        // Read frame i
        stringstream color_img_path, d_img_path;

        color_img_path << data_path << "rgb/" << setw(4) << setfill('0') << i << ".png";
        d_img_path << data_path << "depth/" << setw(4) << setfill('0') << i << ".png";

        cv::Mat color_image = cv::imread(color_img_path.str(), cv::IMREAD_COLOR);
        cv::Mat d_img = cv::imread(d_img_path.str(), cv::IMREAD_ANYDEPTH);
        d_img.convertTo(d_img, CV_32F);
        if (!color_image.data || !d_img.data) break;


        Eigen::MatrixXd cloud_array, color_array;
        DepthToPointCloud(d_img, fx, fy, cx, cy, depth_scale, 10, cloud_array);

// 用来转换存储点云
//        stringstream save_pc_path;
//        save_pc_path << data_path << "pointcloud/" << setw(4) << setfill('0') << i << ".ply";
//        SavePc(save_pc_path.str(), cloud_array);
//        continue;
//        DrawPc(_cloud_array);

        HPS hps(EVOPS_ICL::IMG_HGT, EVOPS_ICL::IMG_WID, 4, 3, 4);
        hps.SetImg(color_image, d_img, fx, fy, cx, cy, depth_scale);
        hps.SetPointCloud(cloud_array);

        hps.SetPlaneThresh(PLANE_PATCH_THRESHMODE_DISTANCE_RATIO, 0.00004);
        hps.SetMinAreaOfRG(8);
        hps.SetMergePlaneParams(5);
        hps.SetCoplanarParams(cos(M_PI / 15), PLANE_MERGE_THRESHMODE_DISTANCE_RATIO, 0.0001);



        // 分割平面
        auto t0 = std::chrono::high_resolution_clock::now();
        hps.Process();
        auto t1 = std::chrono::high_resolution_clock::now();
        auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
        cout << "Process: " << time_elapsed << "us" << endl;
        cout<<"FPS: "<<1000000/time_elapsed<<endl;

        stringstream save_img_path;
        save_img_path << data_path << "/my_output/" << setw(4) << setfill('0') << i << ".png";
        cv::Mat save_img;
        hps.GetResultReadable(save_img);
        cv::imshow("result", save_img);
        cv::waitKey(1);
//        cv::imwrite(save_img_path.str(), save_img);


#ifdef LOG_TIME
        log_time << time_elapsed << '\n';
#endif
//            log_fitplane_count<<g_program_counter<<'\n';

    }
#ifdef LOG_TIME
    log_time.close();
#endif
//    log_fitplane_count.close();
    return 0;
}