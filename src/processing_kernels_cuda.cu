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

void TestCopy(int n, const float* src, float* dest) {
  const SimpleVector3f* src_vector = 
      reinterpret_cast<const SimpleVector3f*>(src);
  SimpleVector3f* dest_vector = 
      reinterpret_cast<SimpleVector3f*>(dest);
  rgbd_ray_lookup_.resize(n);
  thrust::copy(src_vector, src_vector + n, rgbd_ray_lookup_.begin());
  thrust::copy(rgbd_ray_lookup_.begin(), rgbd_ray_lookup_.end(), dest_vector);
}

// using Eigen::Vector3f;
// using Eigen::Affine3f;

// Kernel function to add the elements of two arrays
__global__
void Add(int n, float *x, float *y) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < n; i += stride)
    y[i] = sinf(x[i]) + cosf(y[i]);
}

__global__
void Add2(int n, float *x, float *y) {
  for (int i = 0; i < n; i++)
    y[i] = sin(x[i]) + cos(y[i]);
}

__global__
void Add3(int n, float *x, float *y) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  y[i] = sinf(x[i]) + cosf(y[i]);
}

__device__
void MatMult(const float* m, float* v1, float* v2) {
  v2[0] = m[0] * v1[0] + m[4] * v1[1] + m[8] * v1[2] + m[12];
  v2[1] = m[1] * v1[0] + m[5] * v1[1] + m[9] * v1[2] + m[13];
  v2[2] = m[2] * v1[0] + m[6] * v1[1] + m[10] * v1[2] + m[14];
}

__global__
void Transform2(int n, const float* m, float* v1, float* v2) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  int di = 3 * stride;
  int imax = 3 * n;
  for (int i = 3 * index; i < imax; i += di) {
    MatMult(m, v1 + i, v2 + i);
  }
}

void GetCapabilities() {
  int nDevices;
  cudaGetDeviceCount(&nDevices);
  for (int i = 0; i < nDevices; i++) {
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, i);
    printf("Device Number: %d\n", i);
    printf("  Device name: %s\n", prop.name);
    printf("  Compute Capability: %d.%d\n", prop.major, prop.minor);
    printf("  Threads per block: %d\n", prop.maxThreadsPerBlock);
    printf("  Blocks per multiprocessor: %d\n", 
        prop.maxBlocksPerMultiProcessor);
    printf("  Multiprocessor count: %d\n", prop.multiProcessorCount);
    
    printf("  Memory Clock Rate (KHz): %d\n",
           prop.memoryClockRate);
    printf("  Memory Bus Width (bits): %d\n",
           prop.memoryBusWidth);
    printf("  Peak Memory Bandwidth (GB/s): %f\n\n",
           2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
  }
}


// __global__
// void Transform(int n, int stride, const Affine3f tf, Vector3f* v1, Vector3f* v2) {
//   int start = (blockIdx.x * blockDim.x + threadIdx.x) * stride;
//   int end = min(n, start + stride);
//   for (int i = start; i < end; ++i) {
//     v2[i] = tf * v1[i];
//   }
// }


uint16_t* cuda_depth_image = nullptr;
float* cuda_tf = nullptr;
float* cuda_points = nullptr;
float* cuda_point_lookups = nullptr;
float* cuda_costmap = nullptr;
uint32_t* cuda_point_indices = nullptr;
int cuda_device = -1;
// Depth image size, hence point cloud size.
int N = 0;
// const float costmap_resolution = 0.05;
const int costmap_size = 100;

void InitCuda(int depth_image_size, float* point_lookups) {
  cudaGetDevice(&cuda_device);
  N = depth_image_size;

  CHECK_EQ(cudaMalloc(&cuda_tf, 12 * sizeof(float)), cudaSuccess);
  CHECK_EQ(cudaMalloc(&cuda_depth_image, N * sizeof(uint16_t)), cudaSuccess);
  CHECK_EQ(cudaMalloc(&cuda_points, 3 * N * sizeof(float)), cudaSuccess);
  CHECK_EQ(cudaMalloc(&cuda_point_lookups, 3 * N * sizeof(float)),
      cudaSuccess);
  CHECK_EQ(cudaMalloc(&cuda_costmap, costmap_size * costmap_size * 
      sizeof(float)), cudaSuccess);
  CHECK_EQ(cudaMalloc(&cuda_point_indices, N * sizeof(uint32_t)), cudaSuccess);

  // Copy the lookups to GPU memory.
  CHECK_EQ(cudaMemcpy(cuda_point_lookups, point_lookups, 3 * N * sizeof(float), cudaMemcpyHostToDevice), cudaSuccess);

}

void DepthToCostmap(float* depth_image, float* tf, float* costmap) {
  
  // Inputs: depth image, point lookups, tf
  // Output: costmap
  
  // Copy depth image to CUDA
  CHECK_EQ(cudaMemcpy(cuda_depth_image, depth_image, N * sizeof(float), 
      cudaMemcpyHostToDevice), cudaSuccess);
  // Copy tf to CUDA
  CHECK_EQ(cudaMemcpy(cuda_tf, tf, 12 * sizeof(float), cudaMemcpyHostToDevice), 
      cudaSuccess);

  // Run depth to point cloud & indexer kernel.
  // Run costmap projection kernel.
  // Run costmap propagation kernel.

  // Copy costmap to CPU.
  CHECK_EQ(cudaMemcpy(costmap, cuda_costmap, costmap_size * costmap_size * 
      sizeof(float), cudaMemcpyDeviceToHost), cudaSuccess);
  // Return costmap.
}

void TestCuda() {
}

// void TestCuda() {
//   GetCapabilities();
//   int N = 1920*1080;
//   Vector3f* v1 = nullptr;
//   Vector3f* v2 = nullptr;
//   Vector3f* v3 = new Vector3f[N];
//   cudaMallocManaged(&v1, N*sizeof(Vector3f));
//   cudaMallocManaged(&v2, N*sizeof(Vector3f));
//   cudaMemPrefetchAsync(v1, N*sizeof(Vector3f), cudaCpuDeviceId, NULL);
//   cudaMemPrefetchAsync(v2, N*sizeof(Vector3f), cudaCpuDeviceId, NULL);

//   Affine3f tf = Eigen::Translation3f(Vector3f(1, 0, 0)) * 
//       Eigen::AngleAxisf(1.0, Vector3f(0, 1, 0));

//   for (int i = 0; i < N; ++i) {
//     v1[i] = Vector3f(1, 2, 3);
//   }

//   {
//     FunctionTimer ft("CPU");
//     for (int i = 0; i < N; ++i) {
//       v3[i] = tf * v1[i];
//     }
//   }

//   int device = -1;
//   cudaGetDevice(&device);
//   cudaMemPrefetchAsync(v1, N*sizeof(Vector3f), device, NULL);
//   cudaMemPrefetchAsync(v2, N*sizeof(Vector3f), device, NULL);
//   {
//     FunctionTimer ft("GPU");
//     int stride = 128;
//     int blockSize = 1024;
//     int numBlocks = ((N + stride - 1) / stride + blockSize - 1) / blockSize;
//     printf("%d blocks, %d threads\n", numBlocks, blockSize);
//     if (true) {
//       Transform<<<numBlocks, blockSize>>>(N, stride, tf, v1, v2);
//     } else {
//       Eigen::Matrix4f m = tf.matrix();
//       Transform2<<<numBlocks, blockSize>>>(
//           N, 
//           m.data(), 
//           reinterpret_cast<float*>(v1), 
//           reinterpret_cast<float*>(v2));
//     }
//     // Wait for GPU to finish before accessing on host
//     cudaDeviceSynchronize();
//   }
//   cudaMemPrefetchAsync(v1, N*sizeof(Vector3f), cudaCpuDeviceId, NULL);
//   cudaMemPrefetchAsync(v2, N*sizeof(Vector3f), cudaCpuDeviceId, NULL);
//   float max_error = 0;
//   for (int i = 0; i < N; ++i) {
//     max_error = max(max_error, (v3[i] - v2[i]).norm());
//   }
//   printf("Max error: %f\n", max_error);

//   cudaFree(v1);
//   cudaFree(v2);
//   delete[] v3;
// }

// void TestCuda() {
//   GetCapabilities();
//   int N = 1<<26;
//   float *x, *y;
//   float *z = new float[N];

//   // Allocate Unified Memory – accessible from CPU or GPU
//   cudaMallocManaged(&x, N*sizeof(float));
//   cudaMallocManaged(&y, N*sizeof(float));

//   // initialize x and y arrays on the host
//   for (int i = 0; i < N; i++) {
//     x[i] = 1.0f;
//     y[i] = 2.0f;
//     z[i] = sin(x[i]) + cos(y[i]);
//   }

//   {
//     FunctionTimer ft("CPU");
//     for (int i = 0; i < N; i++) {
//       z[i] = sin(x[i]) + cos(y[i]);
//     }
//   }
//   // Run kernel on 1M elements on the GPU
//   if (true) {
//     FunctionTimer ft("GPU");
//     int blockSize = 1024;
//     int numBlocks = (N + 1) / blockSize;
//     printf("%d blocks, %d threads\n", numBlocks, blockSize);
//     Add3<<<numBlocks, blockSize>>>(N, x, y);
//     // Add2<<<1, 256>>>(N, x, y);

//     // Wait for GPU to finish before accessing on host
//     cudaDeviceSynchronize();
//   } else {
//     FunctionTimer ft("GPU");
//     int blockSize = 1024;
//     int numBlocks = (N + blockSize - 1) / blockSize;
//     printf("%d blocks, %d threads\n", numBlocks, blockSize);
//     Add<<<numBlocks, blockSize>>>(N, x, y);
//     // Add2<<<1, 256>>>(N, x, y);

//     // Wait for GPU to finish before accessing on host
//     cudaDeviceSynchronize();
//   }

//   // Check for errors (all values should be 3.0f)
//   float maxError = 0.0f;
//   for (int i = 0; i < N; i++) {
//     maxError = fmax(maxError, fabs(y[i]-z[i]));
//   }
//   std::cout << "Max error: " << maxError << std::endl;

//   // Free memory
//   cudaFree(x);
//   cudaFree(y);
//   delete z;
// }
