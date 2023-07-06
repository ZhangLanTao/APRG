#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "itodd_data_tools.hpp"
#include "HPS.h"

using std::vector;


int main() {
    string data_path = "/home/zlt/CLionProjects/HierarchicalPlaneSegmentation/data/itodd/itodd_test_bop19/test/000001";
    // 读取json
    FILE *fp = fopen((data_path + "/scene_camera.json").c_str(), "r"); // 非 Windows 平台使用 "r"
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document d;
    d.ParseStream(is);
    vector<scene_info> scene_infos;
    LoadSceneInfos(d, scene_infos);

    // 遍历json里面写的所有场景
    //存储用时文件
    std::string fn_out;
    std::ofstream log_time;
    fn_out = "HPS_time.txt";
    log_time.open(fn_out.c_str(), std::ios::out);
    for (int repeat = 0; repeat < 1; ++repeat) {
        for (auto item: scene_infos) {
            int i = item.number;
            float fx = item.fx;
            float fy = item.fy;
            float cx = item.cx;
            float cy = item.cy;

             // Read frame i
            stringstream gray_img_path, d_img_path;

            gray_img_path << data_path << "/gray/" << setw(6) << setfill('0') << i << ".tif";
            d_img_path << data_path << "/depth/" << setw(6) << setfill('0') << i << ".tif";

    //        std::cout << "Frame: " << i << endl;

            cv::Mat gray_img = cv::imread(gray_img_path.str(), cv::IMREAD_COLOR);
            cv::Mat d_img = cv::imread(d_img_path.str(), cv::IMREAD_ANYDEPTH);
            d_img.convertTo(d_img, CV_32F);
            if (!gray_img.data || !d_img.data) break;


            Eigen::MatrixXf cloud_array;
            auto t0 = std::chrono::high_resolution_clock::now();
            DepthToPointCloud(d_img, fx, fy, cx, cy, 10, cloud_array);
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
            cout << "DepthToPointCloud: " << time_elapsed << "us" << endl;

            //        SavePc(cloud_array);

            HPS hps(ITODD::IMG_HGT, ITODD::IMG_WID, 4, 3, 6);
            hps.SetPointCloud(cloud_array);
            hps.SetGrayImg(gray_img);


            // 分割平面
            t0 = std::chrono::high_resolution_clock::now();
            hps.Process();
            t1 = std::chrono::high_resolution_clock::now();
            time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
            cout << "Process: " << time_elapsed << "us" << endl;
            log_time << time_elapsed << '\n';

        }
    }
    log_time.close();
    return 0;
}