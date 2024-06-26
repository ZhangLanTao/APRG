#ifndef HIRACHICALPLANESEGMENTATION_PARAMS_H
#define HIRACHICALPLANESEGMENTATION_PARAMS_H

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"


enum PlanePatchThreshMode {
    PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE = 0,
    PLANE_PATCH_THRESHMODE_DISTANCE_RATIO = 1
};

enum PlaneMergeThreshMode {
    PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE = 0,
    PLANE_MERGE_THRESHMODE_DISTANCE_RATIO = 1
};


class Params{
public:
    // 图像信息
    double fx, fy, cx, cy, depth_scale, z_min;
    int img_height, img_width;

    // 递归分割参数
    int block_num_x, block_num_y;           // 首先将整个图像分成 _block_num_x * _block_num_y 个正方形块
    int num_pyramid_layers;                 // 递归分割的层数
    int PS_plane_thresh_mode;               // 递归分割的阈值模式
    double PS_plane_thresh;                 // 递归分割的阈值
    float PS_jump_max_distance2;            // 当相邻像素深度超过此阈值，jump_cnt+1
    int PS_max_allowed_jump_cnt;            // 当jump_cnt超过此阈值，认为不是平面

    // 区域生长参数
    int RG_min_area;                        // 区域生长的最小面积(最小patch的多少倍)
    int RG_plane_merge_thresh_mode;         // 区域生长的合并阈值模式
    double RG_plane_merge_min_cos_angle;    // 区域合并的角度阈值的 cos
    double RG_plane_merge_max_distance2;    // 区域合并的距离平方阈值

    // 像素细分参数
    double BR_point_coplane_max_distance2;  // 像素细分的点到平面距离平方阈值

    // 读取json
    Params(std::string param_file_path){
        FILE *fp = fopen(param_file_path.c_str(), "r"); // 非 Windows 平台使用 "r"
        char readBuffer[65536];
        rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
        rapidjson::Document d;
        d.ParseStream(is);
        fx                                  = d["fx"].GetDouble();
        fy                                  = d["fy"].GetDouble();
        cx                                  = d["cx"].GetDouble();
        cy                                  = d["cy"].GetDouble();
        depth_scale                         = d["depth_scale"].GetDouble();
        z_min                               = d["z_min"].GetDouble();
        img_height                          = d["img_height"].GetInt();
        img_width                           = d["img_width"].GetInt();
        block_num_x                         = d["block_num_x"].GetInt();
        block_num_y                         = d["block_num_y"].GetInt();
        num_pyramid_layers                  = d["num_pyramid_layers"].GetInt();
        PS_plane_thresh                     = d["PS_plane_thresh"].GetDouble();
        PS_jump_max_distance2               = d["PS_jump_max_distance2"].GetFloat();
        PS_max_allowed_jump_cnt             = d["PS_max_allowed_jump_cnt"].GetInt();
        RG_min_area                         = d["RG_min_area"].GetInt();
        RG_plane_merge_min_cos_angle        = d["RG_plane_merge_min_cos_angle"].GetDouble();
        RG_plane_merge_max_distance2        = d["RG_plane_merge_max_distance2"].GetDouble();
        BR_point_coplane_max_distance2      = d["BR_point_coplane_max_distance2"].GetDouble();

        PS_plane_thresh_mode                = strcmp(d["PS_plane_thresh_mode"].GetString(), "PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE") == 0 ? PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE : PLANE_PATCH_THRESHMODE_DISTANCE_RATIO;
        RG_plane_merge_thresh_mode          = strcmp(d["RG_plane_merge_thresh_mode"].GetString(), "PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE") == 0 ? PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE : PLANE_MERGE_THRESHMODE_DISTANCE_RATIO;

    }
};

#endif //HIRACHICALPLANESEGMENTATION_PARAMS_H