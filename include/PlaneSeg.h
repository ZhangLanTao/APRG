//
// Created by zlt on 23-4-10.
//

#ifndef HIRACHICALPLANESEGMENTATION_PLANESEG_H
#define HIRACHICALPLANESEGMENTATION_PLANESEG_H

#include <iostream>
#include "Params.h"
#include <Eigen/Dense>
#include <ctime>


class PlaneSeg
{
public:
    int nr_pts, min_nr_pts;
    Eigen::MatrixXf X_matrix, Y_matrix, Z_matrix;
    double x_acc, y_acc, z_acc,
            xx_acc, yy_acc, zz_acc,
            xy_acc, xz_acc, yz_acc;
    float score;
    float MSE;
    bool planar;

    // Plane params
    double mean[3];
    double normal[3];
    double d;
    PlaneSeg(Eigen::MatrixXf & cloud_array, int cell_id, int nr_pts_per_cell, int cell_width);
    void fitPlane();
    void expandSegment(PlaneSeg * plane_seg);
    void clearPoints();
    ~PlaneSeg();
};



#endif //HIRACHICALPLANESEGMENTATION_PLANESEG_H
