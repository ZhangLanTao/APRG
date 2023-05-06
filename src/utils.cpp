#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "open3d/Open3D.h"

using std::vector, std::cout, std::endl;

void DepthToPointCloud(cv::Mat & d_img, float fx, float fy, float cx, float cy, double z_min, Eigen::MatrixX3f & cloud_array){
    int height = d_img.rows;
    int width = d_img.cols;
    cloud_array.setZero(height*width,3);

    for(int r=0; r< height; r++){
        for(auto c=0; c< width; c++){
            float z = d_img.at<float>(r, c);
            if(z>z_min){
                cloud_array(r*width+c,0) = (c-cx)*z/fx;
                cloud_array(r*width+c,1) = (r-cy)*z/fy;
                cloud_array(r*width+c,2) = z;
            }
        }
    }
}


void DrawPc(Eigen::MatrixXf cloud_array){
    std::vector<Eigen::Vector3d> points;
    for (int i = 0 ; i<cloud_array.rows(); ++i) {
        Eigen::Vector3f p = cloud_array.row(i);
        if (p.isZero()) continue;
        points.push_back(p.cast<double>());
    }
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_ = points;

    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D", 1200, 900);
    visualizer.AddGeometry(cloud);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}

void PrintVector(vector<int>& v)
{
        for (auto e : v)
        {
            cout << e << " ";
        }
        cout << endl;
}

