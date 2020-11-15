// Copyright (c) 2020 Joydeep Biswas joydeepb@cs.utexas.edu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <k4a/k4a.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "math/geometry.h"
#include "k4a_wrapper.h"
#include "util/helpers.h"
#include "util/timer.h"

#include "CImg.h"

using std::string;
using Eigen::Vector3f;
using k4a_wrapper::K4AWrapper;

DECLARE_int32(v);
DEFINE_string(serial, "", "Serial number of kinect to open");
DEFINE_string(save_dir, "", "Directory to save files to");

bool run_ = true;

class RGBDFileWriter : public K4AWrapper {
 public:

  RGBDFileWriter(
      const std::string& serial,
      const k4a_device_configuration_t& config)  :
      K4AWrapper(serial, config, true) { }

  void RegisteredRGBDCallback(k4a_image_t color_image, k4a_image_t depth_image)
      override {
    static uint32_t file_count = 0;
    if (FLAGS_v > 0) {
      printf("Received a registered frame %09u, t=%f\n",
          file_count, GetMonotonicTime());
    }
    const string color_file = StringPrintf("%s/%09d_color.pnm", 
        FLAGS_save_dir.c_str(), file_count);
    const string depth_file = StringPrintf("%s/%09d_depth.pnm",   
        FLAGS_save_dir.c_str(), file_count);
    const int w = calibration_.color_camera_calibration.resolution_width;
    const int h = calibration_.color_camera_calibration.resolution_height;
    const int n = w * h;
    cimg_library::CImg<uint8_t> color_cimg(w, h, 1, 3);
    cimg_library::CImg<uint16_t> depth_cimg(w, h, 1, 1);
    uint16_t* depth_data = 
        reinterpret_cast<uint16_t*>(k4a_image_get_buffer(depth_image));
    uint32_t* rgb_data = 
        reinterpret_cast<uint32_t*>(k4a_image_get_buffer(color_image));
    memcpy(depth_cimg.data(), depth_data, n * sizeof(uint16_t));
    for (int i = 0; i < n; ++i) {
      const int x = i % w;
      const int y = i / w;
      const uint32_t rgb = rgb_data[i];
      const uint32_t r = (rgb & 0xFF0000) >> 16;
      const uint32_t g = (rgb & 0xFF00) >> 8;
      const uint32_t b = (rgb & 0xFF);
      color_cimg(x, y, 0, 0) = r;
      color_cimg(x, y, 0, 1) = g;
      color_cimg(x, y, 0, 2) = b;
    }
    color_cimg.save_pnm(color_file.c_str());
    depth_cimg.save_pnm(depth_file.c_str());
    ++file_count;
  }
};

void SigIntHandler(int) {
  printf("Stopping...\n");
  run_ = false;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_save_dir.empty()) {
    fprintf(stderr, "ERROR: Must specify directory with --save_dir");
    return 1;
  }

  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  config.synchronized_images_only = true;
  RGBDFileWriter interface(FLAGS_serial, config);
  signal(SIGINT, SigIntHandler);
  while (run_) {
    interface.Capture();
  }
  return 0;
}



