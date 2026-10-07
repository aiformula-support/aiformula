#ifndef OBJECT_ROAD_DETECTOR_TRT_TRT_ENGINE_HPP
#define OBJECT_ROAD_DETECTOR_TRT_TRT_ENGINE_HPP

#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <rclcpp/rclcpp.hpp>

namespace aiformula {

/**
 * @brief Minimal owner of a serialized TensorRT engine and its execution context.
 *
 * Engines are built by `object_road_detector/tools/export_engine.py` for one GPU, one TensorRT
 * version and one fixed input size, so this class only has to deserialize and run them.
 */
class TrtEngine {
public:
    TrtEngine(const std::string& engine_path, const rclcpp::Logger& logger);
    ~TrtEngine();

    TrtEngine(const TrtEngine&) = delete;
    TrtEngine& operator=(const TrtEngine&) = delete;

    bool hasTensor(const std::string& name) const;
    std::vector<int64_t> tensorShape(const std::string& name) const;
    std::size_t tensorByteSize(const std::string& name) const;

    void bind(const std::string& name, void* device_address);
    void enqueue(cudaStream_t stream);

    std::string describe() const;

private:
    class TrtLogger;

    std::unique_ptr<TrtLogger> trt_logger_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
};

}  // namespace aiformula

#endif  // OBJECT_ROAD_DETECTOR_TRT_TRT_ENGINE_HPP
