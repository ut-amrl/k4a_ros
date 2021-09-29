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

#include <vector>

#include "eigen3/Eigen/Dense"
#include "k4a/k4a.h"
#include "k4abt.hpp"

namespace k4a_wrapper {

class K4AWrapper {
 public:
  ~K4AWrapper();
  K4AWrapper() = delete;
  K4AWrapper(const std::string& serial,
             const k4a_device_configuration_t& config,
             bool enable_image_registration);

  void Capture();

  virtual void DepthCallback(k4a_image_t image) {}
  virtual void ColorCallback(k4a_image_t image) {}
  virtual void UnregisteredRGBDCallback(
      k4a_image_t image_rgb, k4a_image_t image_depth) {}
  virtual void RegisteredRGBDCallback(
      k4a_image_t image_rgb, k4a_image_t image_depth) {}

 private:
  void OpenDevice(const std::string& serial);
  void InitLookups();
  std::string GetKinectSerial();

 private:

  k4a_device_t device_;
  k4a_transformation_t transformation_;
  const bool register_images_;

 public:
  k4a_calibration_t calibration_;
  k4a_device_configuration_t config_;
};

}  // namespace k4a_wrapper
