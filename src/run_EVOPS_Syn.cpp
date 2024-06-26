#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "EVOPS_data_tools.hpp"
#include "Params.h"
#include "HPS.h"
#include "utils.h"

using std::vector, std::string, std::stringstream;
using std::cout, std::endl, std::setw, std::setfill;


int main() {
    g_total_time = 0;
    g_program_counter = 0;

//    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Synthetic/icl-living-room_cc";
//    string image_prefix = "/living_room_traj0_loop/scene_00_";
//    int data_frame_num = 1510;
//    int data_frame_num = 500;
//    Params params(data_path + "/params-icl-living-room.json");
//
    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/EVOPS/Synthetic/icl-office";
    string image_prefix = "/office_room_traj0_loop/scene_00_";
    double depth_scale = 0.01;
    int data_frame_num = 1510;
    Params params(data_path + "/params-icl-office.json");



//    //存储用时文件
//#ifdef LOG_TIME
//    std::string fn_out;
//    std::ofstream log_time;
//    fn_out = "HPS_time.txt";
//    log_time.open(fn_out.c_str(), std::ios::out);
//#endif
////    std::string fn_out;
////    std::ofstream log_fitplane_count;
////    fn_out = "FitPlaneCount.txt";
////    log_fitplane_count.open(fn_out.c_str(), std::ios::out);

    long total_process_time = 0;
    for (int i=0; i<data_frame_num; i++){
        stringstream color_img_path, d_img_path;

        color_img_path << data_path << image_prefix << setw(4) << setfill('0') << i << ".png";
        d_img_path << data_path << image_prefix << setw(4) << setfill('0') << i << ".depth";

        std::cout << "Frame: " << i << endl;

        cv::Mat color_image = cv::imread(color_img_path.str(), cv::IMREAD_COLOR);
        cv::Mat d_img = EVOPS_ICL::readDepthFile_ICL(d_img_path.str());
        d_img.convertTo(d_img, CV_32F);
        if (!color_image.data || !d_img.data) break;

        Eigen::MatrixXf cloud_array, X, Y, Z;
        DepthToPointCloud(d_img, params.fx, params.fy, params.cx, params.cy, params.depth_scale, 10, cloud_array);
        X = cloud_array.col(0);
        Y = cloud_array.col(1);
        Z = cloud_array.col(2);
        X.resize(params.img_width, params.img_height);
        Y.resize(params.img_width, params.img_height);
        Z.resize(params.img_width, params.img_height);
        X.transposeInPlace();
        Y.transposeInPlace();
        Z.transposeInPlace();

//        用来存储点云
//        stringstream save_pc_path;
//        save_pc_path << data_path << image_prefix << setw(4) << setfill('0') << i << ".ply";
//        SavePc(save_pc_path.str(), cloud_array);
//        continue;
//        DrawPc(_cloud_array);

        HPS hps(params);

        hps.SetData(color_image, X, Y,  Z);

        auto t1 = std::chrono::high_resolution_clock::now();
        hps.Process();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        total_process_time += time_elapsed;

        hps.DebugShowResult(1);

//        stringstream save_img_path;
//        save_img_path << data_path << "/my_output/" << setw(4) << setfill('0') << i << ".png";
        cv::Mat save_img;
        hps.GetResultReadable(save_img);
        cv::imshow("result", save_img);
//        cv::imwrite(save_img_path.str(), save_img);
        auto key = cv::waitKey(1);
        if (key == 's') DrawPc(cloud_array, color_image);
        if (key == 'r') cout<<"本帧重来"<<endl, --i;    // retry
//
//
//#ifdef LOG_TIME
//        log_time << time_elapsed << '\n';
//#endif
//            log_fitplane_count<<g_program_counter<<'\n';
//
        hps.Clear();
    }
    cout << "g_program_counter: " << float(g_program_counter)/data_frame_num << endl;
    cout << "g_total_time: " << float(g_total_time)/data_frame_num << endl;
    cout << "average FPS: " << 1e6*data_frame_num/ total_process_time<< endl;
//#ifdef LOG_TIME
//    log_time.close();
//#endif
////    log_fitplane_count.close();
    return 0;
}