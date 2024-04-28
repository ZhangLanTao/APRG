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
// The depth images are scaled by a factor of 5000, i.e., a pixel value of 5000 in the depth image corresponds to a
// distance of 1 meter from the camera, 10000 to 2 meter distance, etc. A pixel value of 0 means missing value/no data.
    constexpr float depth_scale = 1000/5000.0;
}

#endif //HIRACHICALPLANESEGMENTATION_ITODD_DATA_TOOLS_HPP
