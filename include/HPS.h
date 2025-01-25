#ifndef HIERARCHICALPLANESEGMENTATION_HPS_H
#define HIERARCHICALPLANESEGMENTATION_HPS_H

#include <opencv2/opencv.hpp>
// #define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>
#include <utility>

#include "Params.h"
#include "PointsSamplePyramid.h"
#include "SegStatePyramid.h"

#define TRUE

class HPS {
private:
  /*************************************************** General data ***************************************************/
  Params _params;     // all params
  cv::Mat _color_img; // input color image (mainly for debug)
  cv::Mat _layer0_label_img, _layer0_label_img_eroded, _layer0_label_img_eroded_scaleup; // label image of layer0
  cv::Mat _final_label_img;                                                              // final label image

  PointsSamplePyramid _points_sample_pyramid; // data sample pyramid
  SegStatePyramid _seg_state_pyramid;         // status pyramid

  /*********************************************** About Region Growing ***********************************************/
  int _RG_region_count = 1;        // this count is related to region label, 0 means invalid, so start from 1
  vector<int> _RG_region_areas{0}; // region area, {0} to align with mentioned _RG_region_count start from 1
  vector<PlaneParams> _RG_region_plane_params{{}}; // region plane params, also align with up mentioned _RG_region_count

  /********************************************** About Boundary Refine ***********************************************/
  // Record the minimum known distance when a pixel is assigned to a plane, for
  // speed up. Update if the new distance is smaller.
  float *_BR_pixel_refine_min_distance_to_plane;

  // lut for debug
  cv::Mat _Debug_label_color_lut = cv::Mat(1, 256, CV_8UC3);

  /*************************************************** Main Steps *****************************************************/
  void _BuildPyramid();
  void _RecursivePlaneSegment();
  void _PatchwiseRegionGrowing();
  void _BoundaryRefine();

  /************************************************ Other Functions ***************************************************/
  void _Init();
  bool _RG_GetNextSeed(vector<int> &seed_ind);                        // get next seed for region growing
  void _RG_RegionGrowingFrom(const vector<int> &seed_ind, int label); // RG from one seed
  vector<vector<int>> _RG_GetNeighbors(const vector<int> &ind);       // get neighbors of a patch

public:
  HPS(Params P) : _params(P) { _Init(); };

  void SetData(const cv::Mat &color_img, const Eigen::MatrixXf &X, const Eigen::MatrixXf &Y, const Eigen::MatrixXf &Z);
  void Process();
  void GetResultReadable(cv::Mat &output);
  void Clear();

  void DebugShowAPatch_HIndex(vector<int> ind);
  void DebugShowResult(int waitkey = 0);
};

#endif // HIERARCHICALPLANESEGMENTATION_HPS_H
