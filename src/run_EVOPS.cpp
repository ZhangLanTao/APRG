#include "EVOPS_data_tools.hpp"
#include "HPS.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using std::cout, std::endl, std::setw, std::setfill;
using std::vector, std::string, std::stringstream;

int main(int argc, char **argv) {
    // load json file form command line
    if (argc != 2) {
        cout << "Usage: ./Run_EVOPS <config_path>.json" << endl;
        return 1;
    }
    string config_path = argv[1];
    FILE *fp           = fopen(config_path.c_str(), "r"); // use "r" if not Windows OS
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document d;
    d.ParseStream(is);
    string data_format     = d["data_format"].GetString();
    string data_path       = d["data_path"].GetString();
    string rgb_prefix      = d["rgb_prefix"].GetString();
    string depth_prefix    = d["depth_prefix"].GetString();
    string param_file_path = d["param_file_path"].GetString();
    int data_frame_num     = d["data_frame_num"].GetInt();

    Params params(param_file_path);

    // time log
    std::string fn_out;
    std::ofstream log_time;
    fn_out = data_path + "log_time.txt";
    log_time.open(fn_out.c_str(), std::ios::out);

    long total_process_time = 0;
    for (auto i = 0; i < data_frame_num; i++) {
        // Read frame i
        stringstream color_img_path, d_img_path;

        cv::Mat color_image, d_img;
        if (data_format == "icl") {
            color_img_path << data_path << rgb_prefix << setw(4) << setfill('0') << i << ".png";
            d_img_path << data_path << depth_prefix << setw(4) << setfill('0') << i << ".depth";
            cout << "Frame: " << i << endl;
            cout << "color image: " << color_img_path.str() << endl;
            cout << "depth : " << d_img_path.str() << endl;
            color_image = cv::imread(color_img_path.str(), cv::IMREAD_COLOR);
            d_img = readDepthFile_ICL(d_img_path.str(), params);
            d_img.convertTo(d_img, CV_32F);
        } else if (data_format == "tum") {
            color_img_path << data_path << rgb_prefix << setw(4) << setfill('0') << i << ".png";
            d_img_path << data_path << depth_prefix << setw(4) << setfill('0') << i << ".png";
            cout << "Frame: " << i << endl;
            cout << "color image: " << color_img_path.str() << endl;
            cout << "depth : " << d_img_path.str() << endl;
            color_image = cv::imread(color_img_path.str(), cv::IMREAD_COLOR);
            d_img       = cv::imread(d_img_path.str(), cv::IMREAD_ANYDEPTH);
            d_img.convertTo(d_img, CV_32F);
        }
        else {
            cout << "Unknown data format." << endl;
            break;
        }

        if (!color_image.data || !d_img.data) {
            cout << "No data." << endl;
            break;
        }

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

        // store point cloud for debug
        // stringstream save_pc_path;
        // save_pc_path << data_path << "pointcloud/" << setw(4) << setfill('0') << i << ".ply";
        // SavePc(save_pc_path.str(), cloud_array);
        // continue;
        // DrawPc(_cloud_array);

        HPS hps(params);

        hps.SetData(color_image, X, Y, Z);

        auto t1 = std::chrono::high_resolution_clock::now();
        hps.Process();
        auto t2 = std::chrono::high_resolution_clock::now();

        auto time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        log_time << time_elapsed << '\n';
        total_process_time += time_elapsed;

        // For debug
        // hps.DebugShowResult(1);

        stringstream save_img_path;
        save_img_path << data_path << "output/";
        // mkdir if not exist
        std::filesystem::create_directories(save_img_path.str());
        
        save_img_path << setw(4) << setfill('0') << i << ".png";
        cv::Mat save_img;
        hps.GetResultReadable(save_img);
        cv::imwrite(save_img_path.str(), save_img);
        hps.Clear();
    }
    cout << "g_total_time: " << float(g_total_time) / data_frame_num << endl;
    cout << "average FPS: " << 1e6 * data_frame_num / total_process_time << endl;
    log_time.close();

    return 0;
}