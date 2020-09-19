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

#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.hpp>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"

#include "math/geometry.h"
#include "k4a_image/K4ACaptureRecipient.h"
#include "k4a_image/KinectWrapper.h"
#include "util/timer.h"

class K4ARosInterface :  public K4ACaptureRecipient {
 public:
  void receiveFrame(k4a_capture_t capture) override {
    printf("Received a frame, t=%f\n", GetMonotonicTime());
  }
};

int main(int argc, char* argv[]) {
  K4ARosInterface interface;
  KinectWrapper wrapper(0, interface);
  ros::init(argc, argv, "joystick");
  ros::NodeHandle n;

  while (ros::ok()) {
    wrapper.capture();
    ros::spinOnce();
  }
  return 0;
}



