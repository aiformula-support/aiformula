#ifndef OBJECT_ROAD_DETECTOR_TRT_KERNELS_HPP
#define OBJECT_ROAD_DETECTOR_TRT_KERNELS_HPP

#include <cuda_runtime_api.h>

#include <cstdint>

namespace aiformula {

/**
 * @brief Affine map from the uploaded source rectangle into the engine input.
 *
 * `model = source * scale + pad`, where `source` is relative to the origin of the rectangle that
 * was uploaded to the device. Native-scale cropping leaves `scale` at 1 with no padding, which
 * makes both resampling steps exact; whole-frame letterboxing sets `scale < 1` and pads one axis.
 */
struct InputTransform {
    float scale;
    float pad_x;
    float pad_y;
};

/**
 * @brief Converts an interleaved 8-bit BGR(A) rectangle into the planar normalized float tensor the
 * engine expects, in a single pass over the output.
 *
 * Reading the camera's native BGRA directly is what removes the `cv_bridge` colour conversion from
 * the pipeline, so `source_channels` is 4 for the ZED and 3 for an already-converted image. The
 * channel order is otherwise preserved rather than swapped to RGB, because the Python
 * implementation fed the network the BGR buffer straight from `cv_bridge`; `swap_red_blue` exists
 * to undo that if a future model is trained the other way round.
 */
void launchPreprocess(const std::uint8_t* source_bgr, int source_pitch, int source_channels, int source_width,
                      int source_height, float* destination_nchw, int destination_width, int destination_height,
                      const InputTransform& transform, const float mean[3], const float stdev[3], float pad_value,
                      bool swap_red_blue, cudaStream_t stream);

/**
 * @brief Resamples the single-channel lane score to the source rectangle and thresholds it.
 *
 * The engine emits `lane_probability[1] - lane_probability[0]`, so thresholding the interpolated
 * difference at zero reproduces the arg-max of the interpolated per-class probabilities exactly.
 */
void launchLaneMask(const float* lane_score, int score_width, int score_height, std::uint8_t* destination_mask,
                    int destination_pitch, int destination_width, int destination_height,
                    const InputTransform& transform, cudaStream_t stream);

}  // namespace aiformula

#endif  // OBJECT_ROAD_DETECTOR_TRT_KERNELS_HPP
