// #pragma once

#ifndef _UNDISTORTED_PHOTOMETRIC_HPP_
#define _UNDISTORTED_PHOTOMETRIC_HPP_

#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// #include "Eigen/Core"
// #include <opencv2/gapi/core.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <iterator>
/// #include <arm_neon.h>
#include <math.h>

using namespace cv;
using namespace std;


// wxliu added 2023-2-3
#ifdef __ARM__
#include <arm_neon.h>
#else

#define _WX_GRAIN_SIZE_ 1

#include <tbb/parallel_for.h>

struct BlockedRange {
  BlockedRange() = default;
  BlockedRange(int begin, int end, int gsize)
      : begin_{begin}, end_{end}, gsize_{gsize <= 0 ? end - begin : gsize} {} // gsize <= 0的情况下，块的大小就是end-begin 这种情况下就只能分一个块，也就是只能是单线程
  int begin_{};
  int end_{};
  int gsize_{};

  auto ToTbb() const noexcept {
    return tbb::blocked_range<int>(begin_, end_, gsize_);
  }
};

template <typename F>
void ParallelFor(const BlockedRange& range, const F& function) {
  tbb::parallel_for(range.ToTbb(), [&](const auto& block) { // lambda表达式，将range的值按线程并发数求平均后传给block
    for (int i = block.begin(); i < block.end(); ++i) {
      function(i);
    }
  });
}

#endif
// the end.

class undist_lite {
  // added by wxliu
 public:
  Mat img_undisted_32;  // 32 bit undistorted image
 private:
  string _gammaFile, _vignetteFile;    // dir of photometric calibration files
  string _gammaFile2, _vignetteFile2;  // dir of photometric calibration files
  float* G;                            // photometric result
  float* vignetteMapInv;
  float* _out_data_tmp;

  float* G2;  // photometric result
  float* vignetteMapInv2;
  float* _out_data_tmp2;
  void photometric_init();
  void photometric_init2();
  // void photometric_undistort_parallel(unsigned char* in_data, float*
  // out_data);
  void photometric_undistort_parallel(unsigned char* in_data,
                                      float* out_data,
                                      int cam_index);
  // the end.

 private:
  int w, h, wh;  // image width and height

 public:

  // 2023-2-3
  undist_lite(const int ImgWidth,
              const int ImgHeight,
              const string gammaFile1,
              const string vignetteFile1,
              const string gammaFile2,
              const string vignetteFile2);
  // const string gammaFile, const string vignetteFile, int undistort_type);
  // the end.

  ~undist_lite();
  void undist(cv::Mat& in_img,
              cv::Mat& out_img);  // include photometric_undistort and
                                  // geometric_undistort (both parallel)
  void geometric_undistort_parallel(float* in_data, float* out_data);
};


// the end.

#endif // _UNDISTORTED_PHOTOMETRIC_HPP_
