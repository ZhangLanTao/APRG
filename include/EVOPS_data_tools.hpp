#ifndef HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP
#define HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP

#include <vector>
#include "document.h"
#include <sstream>
#include <opencv2/opencv.hpp>

namespace EVOPS_ICL{
    constexpr int IMG_WID=640, IMG_HGT=480;
    constexpr float fx = 481.20;
    constexpr float fy = 480.00;
    constexpr float cx = 319.50;
    constexpr float cy = 239.50;

    cv::Mat readDepthFile_ICL(const std::string& filename)
    {
        cv::Mat depth_img = cv::Mat::zeros(IMG_HGT, IMG_WID, CV_32FC1);

        std::ifstream depthfile;
        depthfile.open(filename, std::ios::in);

        for(int i = 0 ; i < IMG_HGT ; i++)
        {
            for (int j = 0 ; j < IMG_WID ; j++)
            {
                float val = 0;
                depthfile >> val;
                float z = 1000 * (fx+fy)/2 * sqrt((val*val)/(fx*fy+(i-cy)*(i-cy)+(j-cx)*(j-cx)));

                depth_img.at<float>(i,j) = z;
            }
        }
        depthfile.close();
        return depth_img;
    }



}


namespace EVOPS_TUM{
    constexpr int IMG_WID=640, IMG_HGT=480;
    constexpr float fx = 525.00;
    constexpr float fy = 525.00;
    constexpr float cx = 319.50;
    constexpr float cy = 239.50;

    constexpr float depth_scale = 1000/5000.0; // 查阅文档，此数据集的深度图数值需要乘以 0.2 才是真正深度值
}

#endif //HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP
