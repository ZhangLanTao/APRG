#ifndef HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP
#define HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP

#include <vector>
#include "document.h"

namespace ITODD{
constexpr int IMG_WID=1280, IMG_HGT=960;
}

struct scene_info{
    int number;
    float fx, fy, cx, cy, depth_scale;
};


void LoadSceneInfos(rapidjson::Document & d, std::vector<scene_info> & scene_infos){
    scene_infos.reserve(777);
    for (int i = 0; i < 777; ++i) {
        const char* key = std::to_string(i).c_str();
        if (d.HasMember(key)){
            const rapidjson::Value & cam_K = d[key]["cam_K"];
            const float fx = cam_K[0].GetFloat();
            const float fy = cam_K[4].GetFloat();
            const float cx = cam_K[2].GetFloat();
            const float cy = cam_K[5].GetFloat();
            const float depth_scale = d[key]["depth_scale"].GetFloat();
            struct scene_info info = {i, fx, fy, cx, cy, depth_scale};
            scene_infos.push_back(info);
        }
    }
}




#endif //HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP
