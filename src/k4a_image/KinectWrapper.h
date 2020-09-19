#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include "K4ACaptureRecipient.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

class KinectWrapper
{
public:
    KinectWrapper(uint8_t deviceIndex, K4ACaptureRecipient &kfr);
    ~KinectWrapper();

    void capture();
    //void display();

protected:
    k4a_device_t _device;
    k4a_calibration_t _calibration;
    k4a_device_configuration_t _config;
    K4ACaptureRecipient &_kfr;
};

#endif
