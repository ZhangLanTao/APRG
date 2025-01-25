#ifndef HIRACHICALPLANESEGMENTATION_PARAMS_H
#define HIRACHICALPLANESEGMENTATION_PARAMS_H

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

enum PlanePatchThreshMode { PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE = 0, PLANE_PATCH_THRESHMODE_DISTANCE_RATIO = 1 };

enum PlaneMergeThreshMode { PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE = 0, PLANE_MERGE_THRESHMODE_DISTANCE_RATIO = 1 };

class Params {
  public:
    // camera info
    double fx, fy, cx, cy, depth_scale, z_min;
    int img_height, img_width;

    // Parameters : recursive plane segmentation
    int block_num_x, block_num_y; // the input image is divided to _block_num_x * _block_num_y suqares
    int num_pyramid_layers;       // recursive segmentation layers
    int PS_plane_thresh_mode;     // plane threshold mode
    double PS_plane_thresh;       // plane threshold
    float PS_jump_max_distance2;  // max distance^2 between two adjacent points, if larger, jump_cnt++
    int PS_max_allowed_jump_cnt;  // max jump count, if larger, not a plane

    // Parameters : region growing
    int RG_min_area;                     // min area of a region (ratio to minimum patch size)
    int RG_plane_merge_thresh_mode;      // threshold mode of plane merge (absolute or distance-relative)
    double RG_plane_merge_min_cos_angle; // cos value of min angle between two planes
    double RG_plane_merge_max_distance2; // max distance^2 between two planes

    // Parameters : boundary refine
    double BR_point_coplane_max_distance2; // max distance^2 between a point and a plane

    // read param from json file
    Params(std::string param_file_path) {
        FILE *fp = fopen(param_file_path.c_str(), "r"); // use "r" if not Windows OS
        char readBuffer[65536];
        rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
        rapidjson::Document d;
        d.ParseStream(is);
        fx                             = d["fx"].GetDouble();
        fy                             = d["fy"].GetDouble();
        cx                             = d["cx"].GetDouble();
        cy                             = d["cy"].GetDouble();
        depth_scale                    = d["depth_scale"].GetDouble();
        z_min                          = d["z_min"].GetDouble();
        img_height                     = d["img_height"].GetInt();
        img_width                      = d["img_width"].GetInt();
        block_num_x                    = d["block_num_x"].GetInt();
        block_num_y                    = d["block_num_y"].GetInt();
        num_pyramid_layers             = d["num_pyramid_layers"].GetInt();
        PS_plane_thresh                = d["PS_plane_thresh"].GetDouble();
        PS_jump_max_distance2          = d["PS_jump_max_distance2"].GetFloat();
        PS_max_allowed_jump_cnt        = d["PS_max_allowed_jump_cnt"].GetInt();
        RG_min_area                    = d["RG_min_area"].GetInt();
        RG_plane_merge_min_cos_angle   = d["RG_plane_merge_min_cos_angle"].GetDouble();
        RG_plane_merge_max_distance2   = d["RG_plane_merge_max_distance2"].GetDouble();
        BR_point_coplane_max_distance2 = d["BR_point_coplane_max_distance2"].GetDouble();

        PS_plane_thresh_mode = strcmp(d["PS_plane_thresh_mode"].GetString(), "PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE") == 0
                                   ? PLANE_PATCH_THRESHMODE_ABSOLUTE_MSE
                                   : PLANE_PATCH_THRESHMODE_DISTANCE_RATIO;
        RG_plane_merge_thresh_mode =
            strcmp(d["RG_plane_merge_thresh_mode"].GetString(), "PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE") == 0
                ? PLANE_MERGE_THRESHMODE_ABSOLUTE_MSE
                : PLANE_MERGE_THRESHMODE_DISTANCE_RATIO;
    }
};

#endif // HIRACHICALPLANESEGMENTATION_PARAMS_H