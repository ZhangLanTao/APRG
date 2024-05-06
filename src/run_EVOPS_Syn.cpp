#include <iostream>
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
using EVOPS_ICL::fx, EVOPS_ICL::fy, EVOPS_ICL::cx, EVOPS_ICL::cy;


int main() {
//    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Synthetic/icl-living-room_cc";
//    string image_prefix = "/living_room_traj0_loop/scene_00_";
//    int data_frame_num = 1510;
//    double depth_scale = 1;
//
    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Synthetic/icl-office";
    string image_prefix = "/office_room_traj0_loop/scene_00_";
    double depth_scale = 0.01;
//    int data_frame_num = 1510;
    int data_frame_num = 500;

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

        color_img_path << data_path << image_prefix << setw(4) << setfill('0') << i << ".png";
        d_img_path << data_path << image_prefix << setw(4) << setfill('0') << i << ".depth";

//        std::cout << "Frame: " << i << endl;

        cv::Mat color_image = cv::imread(color_img_path.str(), cv::IMREAD_COLOR);
        cv::Mat d_img = EVOPS_ICL::readDepthFile_ICL(d_img_path.str());
        d_img.convertTo(d_img, CV_32F);
        if (!color_image.data || !d_img.data) break;


        Eigen::MatrixXf cloud_array, color_array;
//            auto t0 = std::chrono::high_resolution_clock::now();
        DepthToPointCloud(d_img, fx, fy, cx, cy, depth_scale, 10, cloud_array);
//            auto t1 = std::chrono::high_resolution_clock::now();
//            auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
//            cout << "DepthToPointCloud: " << time_elapsed << "us" << endl;

        // p = p- average(p)
//        auto gaussian_noise = Eigen::MatrixXd::Random(_cloud_array.rows(), _cloud_array.cols())*0.1;

//        用来转换存储点云
//        stringstream save_pc_path;
//        save_pc_path << data_path << image_prefix << setw(4) << setfill('0') << i << ".ply";
//        SavePc(save_pc_path.str(), cloud_array);
//        continue;
//        DrawPc(_cloud_array);

        HPS hps(EVOPS_ICL::IMG_HGT, EVOPS_ICL::IMG_WID, 4, 3, 5);
        hps.SetImg(color_image, d_img, fx, fy, cx, cy, depth_scale);
        hps.SetPointCloud(cloud_array);

        hps.SetPlaneThresh(PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE, 0.5);
        hps.SetMinAreaOfRG(6);
        hps.SetCoplanarParams(cos(M_PI / 18), PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE, 1);
        hps.SetMergePlaneParams(3);
        hps.SetPointCoplaneThresh(1);


        // 分割平面
        auto t0 = std::chrono::high_resolution_clock::now();
        hps.Process();
        auto t1 = std::chrono::high_resolution_clock::now();
        auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
        cout << "Process: " << time_elapsed << "us" << endl;
        cout<<"FPS: "<<1000000/time_elapsed<<endl;

//        stringstream save_img_path;
//        save_img_path << data_path << "/my_output/" << setw(4) << setfill('0') << i << ".png";
        cv::Mat save_img;
        hps.GetResultReadable(save_img);
        cv::imshow("result", save_img);
//        cv::imwrite(save_img_path.str(), save_img);
        cv::waitKey(1);


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