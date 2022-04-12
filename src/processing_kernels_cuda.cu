// Copyright (c) 2021 Joydeep Biswas joydeepb@cs.utexas.edu

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

#include <iostream>
#include <math.h>

#include "cuda.h"
#include "cuda_runtime_api.h"
#include "glog/logging.h"
// #include "eigen3/Eigen/Dense"
// #include "eigen3/Eigen/Geometry"
#include "shared/util/timer.h"
#include "processing_kernels.h"

#include <thrust/copy.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/generate.h>
#include <thrust/reduce.h>
#include <thrust/functional.h>
#include <thrust/tuple.h>
#include <algorithm>

#ifndef __global__
#define __global__
#endif

#ifndef __device__
#define __device__
#endif


// Anonymous namespace to prevent collisions with external use.
namespace {
struct SimpleVector3f {
  float x;
  float y;
  float z;

  __device__ __host__ 
  SimpleVector3f() {}

  __device__ __host__
  SimpleVector3f(float x, float y, float z) : x(x), y(y), z(z) {}

  // SimpleVector3f& operator=(SimpleVector3f&& other) {
  //   x = other.x;
  //   y = other.y;
  //   z = other.z;
  //   return *this;
  // }

  // SimpleVector3f& operator=(const SimpleVector3f& other) {
  //   x = other.x;
  //   y = other.y;
  //   z = other.z;
  //   return *this;
  // }

  // Vector binary operators.
  #define VECTORBINARYOP(op) \
    __device__ __host__ \
    SimpleVector3f operator op(const SimpleVector3f& other) const { \
      return SimpleVector3f( \
        x op other.x, \
        y op other.y, \
        z op other.z); \
    }
  VECTORBINARYOP(+);
  VECTORBINARYOP(-);
  VECTORBINARYOP(*);
  VECTORBINARYOP(/);

  // Scalar binary operators.
  #define SCALARBINARYOP(op) \
    __device__ __host__ \
    SimpleVector3f operator op(const float& c) const { \
      return SimpleVector3f( \
        x op c, \
        y op c, \
        z op c); \
    }
  // SCALARBINARYOP(+);
  // SCALARBINARYOP(-);
  SCALARBINARYOP(*);
  SCALARBINARYOP(/);

};

struct DepthTo3D {
  __device__ __host__
  SimpleVector3f operator() (const uint16_t& depth, const SimpleVector3f& ray) {
    return translation_ + (ray * depth);
  }
  SimpleVector3f translation_;
};

thrust::device_vector<uint16_t> depth_data_;
thrust::device_vector<SimpleVector3f> rgbd_ray_lookup_;
DepthTo3D depth_to_3d_;

}  // namespace

namespace processing_kernels {
void InitializeTransform(const float* ray_lookups,
                         const float* translation,
                         int N) {
  const SimpleVector3f* ray_lookups_vector = 
      reinterpret_cast<const SimpleVector3f*>(ray_lookups);
  rgbd_ray_lookup_.resize(N);
  thrust::copy(ray_lookups_vector, ray_lookups_vector + N, rgbd_ray_lookup_.begin());
  depth_to_3d_.translation_.x = translation[0];
  depth_to_3d_.translation_.y = translation[1];
  depth_to_3d_.translation_.z = translation[2];
}

void DepthImageToPointCloud(const uint16_t* depth_image, 
                            int N,
                            float* point_cloud) {
  static thrust::device_vector<uint16_t> depth_image_d;
  static thrust::device_vector<SimpleVector3f> point_cloud_d;
  depth_image_d.resize(N);
  point_cloud_d.resize(N);
  thrust::copy(depth_image, depth_image + N, depth_image_d.begin());
  thrust::transform(depth_image_d.begin(),
                    depth_image_d.end(),
                    rgbd_ray_lookup_.begin(),
                    point_cloud_d.begin(),
                    depth_to_3d_);
  thrust::copy(point_cloud_d.begin(),
               point_cloud_d.end(),
               reinterpret_cast<SimpleVector3f*>(point_cloud));
}

void TestCopy(int n, const float* src_h, float* dest_h) {
  // Allocate device memory.
  thrust::device_vector<float> tmp_d(n);
  // Host to cuda.
  thrust::copy(src_h, src_h + n, tmp_d.begin());
  // Cuda to host.
  thrust::copy(tmp_d.begin(), tmp_d.end(), dest_h);
}

void GetCudaCapabilities() {
  int nDevices;
  cudaGetDeviceCount(&nDevices);
  for (int i = 0; i < nDevices; i++) {
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, i);
    printf("Device Number: %d\n", i);
    printf("  Device name: %s\n", prop.name);
    printf("  Compute Capability: %d.%d\n", prop.major, prop.minor);
    printf("  Threads per block: %d\n", prop.maxThreadsPerBlock);
    //printf("  Blocks per multiprocessor: %d\n", 
    //    prop.maxBlocksPerMultiProcessor);
    printf("  Multiprocessor count: %d\n", prop.multiProcessorCount);
    
    printf("  Memory Clock Rate (KHz): %d\n",
           prop.memoryClockRate);
    printf("  Memory Bus Width (bits): %d\n",
           prop.memoryBusWidth);
    printf("  Peak Memory Bandwidth (GB/s): %f\n\n",
           2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
  }
}
}  // namespace processing_kernels
