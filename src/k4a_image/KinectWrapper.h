#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include "K4ACaptureRecipient.h"

#include "k4a/k4a.h"

class KinectWrapper {
 public:
  KinectWrapper(uint8_t deviceIndex, K4ACaptureRecipient &kfr);
  ~KinectWrapper();

  void capture();
  //void display();

 protected:
  k4a_device_t device_;
  K4ACaptureRecipient& recipient_;
  k4a_calibration_t calibration_;
  k4a_device_configuration_t config_;
};

#endif
