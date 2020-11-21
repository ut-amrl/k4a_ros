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

DEFINE_string(serial, "", "Serial number of kinect to open");

void PrintExtrinsics(const k4a_calibration_extrinsics_t& k) {
  printf("rotation = [\n");
  for (int i = 0; i < 9; ++i) {
    printf("%9.6f ", k.rotation[i]);
    if (i == 2 || i == 5) printf(";\n");
  }
  printf("\n];\n");

  printf("translation = [\n");
  for (int i = 0; i < 3; ++i) {
    printf("%9.6f\n", k.translation[i]);
  }
  printf("\n];\n");
}

void PrintIntrinsics(const k4a_calibration_intrinsics_t& k) {
  printf("type = %d\n"
         "cx = %f\n"
         "cy = %f\n"
         "fx = %f\n"
         "fy = %f\n"
         "k1 = %f\n"
         "k2 = %f\n"
         "k3 = %f\n"
         "k4 = %f\n"
         "k5 = %f\n"
         "k6 = %f\n"
         "codx = %f\n"
         "cody = %f\n"
         "p2 = %f\n"
         "p1 = %f\n"
         "metric_radius = %f\n",
         static_cast<int>(k.type),
         k.parameters.param.cx,
         k.parameters.param.cy,
         k.parameters.param.fx,
         k.parameters.param.fy,
         k.parameters.param.k1,
         k.parameters.param.k2,
         k.parameters.param.k3,
         k.parameters.param.k4,
         k.parameters.param.k5,
         k.parameters.param.k6,
         k.parameters.param.codx,
         k.parameters.param.cody,
         k.parameters.param.p2,
         k.parameters.param.p1,
         k.parameters.param.metric_radius);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);


  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
  config.synchronized_images_only = true;
  K4AWrapper kinect(FLAGS_serial, config, false);
  printf("Color camera: \n");
  PrintIntrinsics(kinect.calibration_.color_camera_calibration.intrinsics);
  PrintExtrinsics(kinect.calibration_.color_camera_calibration.extrinsics);
  printf("\nDepth camera: \n");
  PrintIntrinsics(kinect.calibration_.depth_camera_calibration.intrinsics);
  PrintExtrinsics(kinect.calibration_.depth_camera_calibration.extrinsics);
  return 0;
}



