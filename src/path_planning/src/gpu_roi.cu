#include "path_planning/gpu_roi.hpp"
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/scan.h>
#include <cstdio>
#include <cmath>
#include <limits>

// (선택) CUB 사용 시 더 빠름: #include <cub/cub.cuh>

namespace path_planning {

static inline void cudaCheck(cudaError_t e, const char* where){
  if (e != cudaSuccess) {
    std::fprintf(stderr, "[CUDA] %s: %s\n", where, cudaGetErrorString(e));
  }
}

__global__ void transform_kernel(const float* __restrict__ x_in,
                                 const float* __restrict__ y_in,
                                 float* __restrict__ x_out,
                                 float* __restrict__ y_out,
                                 int N, float c, float s, float tx, float ty)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < N) {
    float x = x_in[i], y = y_in[i];
    float xr = c * x - s * y + tx;
    float yr = s * x + c * y + ty;
    x_out[i] = xr; y_out[i] = yr;
  }
}

__global__ void distance_kernel(const float* __restrict__ x,
                                const float* __restrict__ y,
                                float* __restrict__ d, // d[0]=0
                                int N)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i == 0 && N > 0) d[0] = 0.0f;
  if (i > 0 && i < N) {
    float dx = x[i] - x[i-1];
    float dy = y[i] - y[i-1];
    d[i] = sqrtf(dx*dx + dy*dy);
  }
}

// 윈도우 내 x<=0은 큰 값(INF)로 취급하여 argmin에서 자동 제외
__global__ void squared_dist_window_xpos_kernel(const float* __restrict__ x,
                                                const float* __restrict__ y,
                                                float* __restrict__ d2,
                                                int N, int i0, int i1)
{
  int t = blockIdx.x * blockDim.x + threadIdx.x;
  int i = i0 + t;
  if (i < i1) {
    float xi = x[i];
    if (xi <= 0.0f) {
      d2[t] = INFINITY;
    } else {
      float yi = y[i];
      d2[t] = xi*xi + yi*yi;
    }
  }
}

__global__ void block_argmin_kernel(const float* __restrict__ vals,
                                    int M, // window length
                                    int* __restrict__ blk_idx,
                                    float* __restrict__ blk_val)
{
  extern __shared__ unsigned char smem[];
  float* s_val = reinterpret_cast<float*>(smem);
  int*   s_idx = reinterpret_cast<int*>(s_val + blockDim.x);

  int g = blockIdx.x * blockDim.x + threadIdx.x;

  float v = INFINITY; int idx = -1;
  if (g < M) { v = vals[g]; idx = g; }

  s_val[threadIdx.x] = v;
  s_idx[threadIdx.x] = idx;
  __syncthreads();

  for (int ofs = blockDim.x/2; ofs>0; ofs>>=1){
    if (threadIdx.x < ofs){
      if (s_val[threadIdx.x + ofs] < s_val[threadIdx.x]){
        s_val[threadIdx.x] = s_val[threadIdx.x + ofs];
        s_idx[threadIdx.x] = s_idx[threadIdx.x + ofs];
      }
    }
    __syncthreads();
  }
  if (threadIdx.x==0){
    blk_idx[blockIdx.x] = s_idx[0];
    blk_val[blockIdx.x] = s_val[0];
  }
}

struct GPURoiHelper::Impl {
  // 변환/아크 계산용 임시 버퍼(콜마다 할당/해제)
  // — transform_and_scan에서만 사용
  // 주기용(지속):
  thrust::device_vector<float> d_x;   // 변환된 x
  thrust::device_vector<float> d_y;   // 변환된 y

  // 윈도우 argmin용 버퍼(재사용)
  thrust::device_vector<float> d_win_d2;
  thrust::device_vector<int>   d_blk_idx;
  thrust::device_vector<float> d_blk_val;

  int N{0};

  // 비동기 결과 수신을 위한 스트림/이벤트/호스트버퍼
  cudaStream_t stream{nullptr};
  cudaEvent_t  evt_done{nullptr};
  int          last_i0{0};
  int          result_index{-1};
  bool         pending{false}; // 현재 진행 중인지

  Impl(){
    cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
    cudaEventCreateWithFlags(&evt_done, cudaEventDisableTiming);
  }
  ~Impl(){
    if (evt_done) cudaEventDestroy(evt_done);
    if (stream)   cudaStreamDestroy(stream);
  }

  void clear_all(){
    d_x.clear(); d_y.clear();
    d_win_d2.clear(); d_blk_idx.clear(); d_blk_val.clear();
    N=0; pending=false; result_index=-1;
    last_i0=0;
  }
};

GPURoiHelper::GPURoiHelper() : pimpl_(new Impl) {}
GPURoiHelper::~GPURoiHelper(){ reset(); delete pimpl_; }

void GPURoiHelper::reset(){ pimpl_->clear_all(); }

bool GPURoiHelper::transform_and_scan(const std::vector<float>& x_in,
                                      const std::vector<float>& y_in,
                                      float c, float s, float tx, float ty,
                                      std::vector<float>& x_out,
                                      std::vector<float>& y_out,
                                      std::vector<double>& arc_out)
{
  const int N = static_cast<int>(x_in.size());
  if (N<=0 || y_in.size()!=static_cast<size_t>(N)) return false;

  thrust::device_vector<float> dx(N), dy(N), dxt(N), dyt(N), dseg(N);
  cudaCheck(cudaMemcpy(thrust::raw_pointer_cast(dx.data()), x_in.data(), sizeof(float)*N, cudaMemcpyHostToDevice), "Memcpy x_in");
  cudaCheck(cudaMemcpy(thrust::raw_pointer_cast(dy.data()), y_in.data(), sizeof(float)*N, cudaMemcpyHostToDevice), "Memcpy y_in");

  const int threads=256, blocks=(N+threads-1)/threads;
  transform_kernel<<<blocks,threads>>>(thrust::raw_pointer_cast(dx.data()),
                                       thrust::raw_pointer_cast(dy.data()),
                                       thrust::raw_pointer_cast(dxt.data()),
                                       thrust::raw_pointer_cast(dyt.data()),
                                       N, c, s, tx, ty);
  cudaError_t e = cudaDeviceSynchronize();
  if (e != cudaSuccess) { std::fprintf(stderr,"[CUDA] transform failed: %s\n", cudaGetErrorString(e)); return false; }

  distance_kernel<<<blocks,threads>>>(thrust::raw_pointer_cast(dxt.data()),
                                      thrust::raw_pointer_cast(dyt.data()),
                                      thrust::raw_pointer_cast(dseg.data()), N);
  e = cudaDeviceSynchronize();
  if (e != cudaSuccess) { std::fprintf(stderr,"[CUDA] distance failed: %s\n", cudaGetErrorString(e)); return false; }

  thrust::inclusive_scan(dseg.begin(), dseg.end(), dseg.begin());

  x_out.resize(N); y_out.resize(N);
  std::vector<float> tmp_arc(N);
  cudaCheck(cudaMemcpy(x_out.data(), thrust::raw_pointer_cast(dxt.data()), sizeof(float)*N, cudaMemcpyDeviceToHost), "Memcpy x_out");
  cudaCheck(cudaMemcpy(y_out.data(), thrust::raw_pointer_cast(dyt.data()), sizeof(float)*N, cudaMemcpyDeviceToHost), "Memcpy y_out");
  cudaCheck(cudaMemcpy(tmp_arc.data(), thrust::raw_pointer_cast(dseg.data()), sizeof(float)*N, cudaMemcpyDeviceToHost), "Memcpy arc");

  arc_out.resize(N);
  for (int i=0;i<N;++i) arc_out[i] = static_cast<double>(tmp_arc[i]);
  return true;
}

void GPURoiHelper::upload_transformed_xy(const std::vector<float>& x_out,
                                         const std::vector<float>& y_out)
{
  const int N = static_cast<int>(x_out.size());
  pimpl_->d_x.assign(x_out.begin(), x_out.end());
  pimpl_->d_y.assign(y_out.begin(), y_out.end());
  pimpl_->N = N;
  pimpl_->d_win_d2.resize(N); // 최대 윈도우 길이를 N으로 할당(한 번만)
  pimpl_->pending = false;
  pimpl_->result_index = -1;
}

void GPURoiHelper::argmin_window_xpos_async(int last_idx, int W){
  if (pimpl_->N<=0) return;
  int N = pimpl_->N;
  int i0 = last_idx - W; if (i0<0) i0 = 0;
  int i1 = last_idx + W; if (i1>N) i1 = N;
  int M = i1 - i0;
  if (M<=0) { // 윈도우 비었으면 전체로
    i0 = 0; i1 = N; M = N;
  }
  pimpl_->last_i0 = i0;

  // 1) 거리계산(x>0 필터 포함)
  const int threads=256, blocks=(M+threads-1)/threads;
  squared_dist_window_xpos_kernel<<<blocks,threads,0,pimpl_->stream>>>(
      thrust::raw_pointer_cast(pimpl_->d_x.data()),
      thrust::raw_pointer_cast(pimpl_->d_y.data()),
      thrust::raw_pointer_cast(pimpl_->d_win_d2.data()),
      N, i0, i1);

  // 2) 블록 argmin
  int B = blocks;
  pimpl_->d_blk_idx.resize(B);
  pimpl_->d_blk_val.resize(B);
  size_t shmem = threads*(sizeof(float)+sizeof(int));

  block_argmin_kernel<<<B,threads,shmem,pimpl_->stream>>>(
      thrust::raw_pointer_cast(pimpl_->d_win_d2.data()), M,
      thrust::raw_pointer_cast(pimpl_->d_blk_idx.data()),
      thrust::raw_pointer_cast(pimpl_->d_blk_val.data()));

  // 3) 마지막 축소(작으니 호스트로 복사)
  //    결과 복사를 기다리기 위한 이벤트 등록
  cudaEventRecord(pimpl_->evt_done, pimpl_->stream);
  pimpl_->pending = true;
}

bool GPURoiHelper::try_fetch_argmin_result(int& out_index){
  if (!pimpl_->pending) return false;
  cudaError_t q = cudaEventQuery(pimpl_->evt_done);
  if (q == cudaSuccess){
    // 이벤트 완료: 블록 결과를 호스트로 가져와 최종 argmin
    int B = static_cast<int>(pimpl_->d_blk_idx.size());
    std::vector<int>   h_idx(B);
    std::vector<float> h_val(B);
    cudaMemcpy(h_idx.data(), thrust::raw_pointer_cast(pimpl_->d_blk_idx.data()), sizeof(int)*B, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_val.data(), thrust::raw_pointer_cast(pimpl_->d_blk_val.data()), sizeof(float)*B, cudaMemcpyDeviceToHost);

    float best = INFINITY; int besti = -1;
    for (int b=0;b<B;++b){
      if (h_idx[b] >= 0 && h_val[b] < best){ best = h_val[b]; besti = h_idx[b]; }
    }
    if (besti >= 0){
      out_index = pimpl_->last_i0 + besti; // 윈도우 오프셋 보정
    } else {
      out_index = -1;
    }
    pimpl_->pending = false;
    pimpl_->result_index = out_index;
    return true;
  }
  // 아직 준비 안됨
  return false;
}

int GPURoiHelper::size() const { return pimpl_->N; }

} // namespace path_planning
