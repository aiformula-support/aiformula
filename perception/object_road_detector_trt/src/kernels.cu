#include "object_road_detector_trt/kernels.hpp"

#include "object_road_detector_trt/cuda_util.hpp"

namespace aiformula {

namespace {

constexpr int BLOCK_DIM = 16;

struct BilinearWeights {
    int x0;
    int y0;
    int x1;
    int y1;
    float weight_x;
    float weight_y;
};

/// Half-pixel centered sampling, matching `cv::resize` and `torch.nn.functional.interpolate`
/// with `align_corners=False`.
__device__ inline BilinearWeights bilinearWeights(const float sample_x, const float sample_y, const int width,
                                                  const int height) {
    BilinearWeights weights;
    const float floor_x = floorf(sample_x);
    const float floor_y = floorf(sample_y);
    weights.weight_x = sample_x - floor_x;
    weights.weight_y = sample_y - floor_y;
    weights.x0 = min(max(static_cast<int>(floor_x), 0), width - 1);
    weights.y0 = min(max(static_cast<int>(floor_y), 0), height - 1);
    weights.x1 = min(weights.x0 + 1, width - 1);
    weights.y1 = min(weights.y0 + 1, height - 1);
    return weights;
}

__device__ inline float blend(const float top_left, const float top_right, const float bottom_left,
                              const float bottom_right, const float weight_x, const float weight_y) {
    const float top = top_left + (top_right - top_left) * weight_x;
    const float bottom = bottom_left + (bottom_right - bottom_left) * weight_x;
    return top + (bottom - top) * weight_y;
}

__global__ void preprocessKernel(const std::uint8_t* __restrict__ source, const int source_pitch,
                                 const int source_channels, const int source_width, const int source_height,
                                 float* __restrict__ destination, const int destination_width,
                                 const int destination_height, const float inverse_scale, const float pad_x,
                                 const float pad_y, const float mean_0, const float mean_1, const float mean_2,
                                 const float inverse_stdev_0, const float inverse_stdev_1, const float inverse_stdev_2,
                                 const float pad_value, const bool swap_red_blue) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= destination_width || y >= destination_height) return;

    const float sample_x = (static_cast<float>(x) - pad_x + 0.5f) * inverse_scale - 0.5f;
    const float sample_y = (static_cast<float>(y) - pad_y + 0.5f) * inverse_scale - 0.5f;

    float channels[3] = {pad_value, pad_value, pad_value};
    const bool inside = sample_x > -1.0f && sample_y > -1.0f && sample_x < static_cast<float>(source_width) &&
                        sample_y < static_cast<float>(source_height);
    if (inside) {
        const BilinearWeights weights = bilinearWeights(sample_x, sample_y, source_width, source_height);
        const std::uint8_t* const row_0 = source + weights.y0 * source_pitch;
        const std::uint8_t* const row_1 = source + weights.y1 * source_pitch;
        for (int channel = 0; channel < 3; ++channel) {
            channels[channel] =
                blend(row_0[weights.x0 * source_channels + channel], row_0[weights.x1 * source_channels + channel],
                      row_1[weights.x0 * source_channels + channel], row_1[weights.x1 * source_channels + channel],
                      weights.weight_x, weights.weight_y) /
                255.0f;
        }
    }

    if (swap_red_blue) {
        const float red = channels[0];
        channels[0] = channels[2];
        channels[2] = red;
    }

    const int plane_stride = destination_width * destination_height;
    const int pixel_index = y * destination_width + x;
    destination[pixel_index] = (channels[0] - mean_0) * inverse_stdev_0;
    destination[plane_stride + pixel_index] = (channels[1] - mean_1) * inverse_stdev_1;
    destination[2 * plane_stride + pixel_index] = (channels[2] - mean_2) * inverse_stdev_2;
}

__global__ void laneMaskKernel(const float* __restrict__ score, const int score_width, const int score_height,
                               std::uint8_t* __restrict__ destination, const int destination_pitch,
                               const int destination_width, const int destination_height, const float scale,
                               const float pad_x, const float pad_y) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= destination_width || y >= destination_height) return;

    const float sample_x = (static_cast<float>(x) + 0.5f) * scale - 0.5f + pad_x;
    const float sample_y = (static_cast<float>(y) + 0.5f) * scale - 0.5f + pad_y;
    const BilinearWeights weights = bilinearWeights(sample_x, sample_y, score_width, score_height);
    const float* const row_0 = score + weights.y0 * score_width;
    const float* const row_1 = score + weights.y1 * score_width;
    const float value = blend(row_0[weights.x0], row_0[weights.x1], row_1[weights.x0], row_1[weights.x1],
                              weights.weight_x, weights.weight_y);

    // 255 rather than 1: RViz and other mono8 viewers treat the image as a full-range
    // intensity map, so a value of 1 is indistinguishable from black. Downstream nodes
    // (lane_line_publisher) only need non-zero, so either encoding works for them.
    destination[y * destination_pitch + x] = value > 0.0f ? 255 : 0;
}

dim3 gridFor(const int width, const int height) {
    return dim3((width + BLOCK_DIM - 1) / BLOCK_DIM, (height + BLOCK_DIM - 1) / BLOCK_DIM);
}

}  // namespace

void launchPreprocess(const std::uint8_t* const source_bgr, const int source_pitch, const int source_channels,
                      const int source_width, const int source_height, float* const destination_nchw,
                      const int destination_width, const int destination_height, const InputTransform& transform,
                      const float mean[3], const float stdev[3], const float pad_value, const bool swap_red_blue,
                      cudaStream_t stream) {
    preprocessKernel<<<gridFor(destination_width, destination_height), dim3(BLOCK_DIM, BLOCK_DIM), 0, stream>>>(
        source_bgr, source_pitch, source_channels, source_width, source_height, destination_nchw, destination_width,
        destination_height, 1.0f / transform.scale, transform.pad_x, transform.pad_y, mean[0], mean[1], mean[2],
        1.0f / stdev[0], 1.0f / stdev[1], 1.0f / stdev[2], pad_value, swap_red_blue);
    CUDA_CHECK(cudaGetLastError());
}

void launchLaneMask(const float* const lane_score, const int score_width, const int score_height,
                    std::uint8_t* const destination_mask, const int destination_pitch, const int destination_width,
                    const int destination_height, const InputTransform& transform, cudaStream_t stream) {
    laneMaskKernel<<<gridFor(destination_width, destination_height), dim3(BLOCK_DIM, BLOCK_DIM), 0, stream>>>(
        lane_score, score_width, score_height, destination_mask, destination_pitch, destination_width,
        destination_height, transform.scale, transform.pad_x, transform.pad_y);
    CUDA_CHECK(cudaGetLastError());
}

}  // namespace aiformula
