#include "object_road_detector_trt/trt_engine.hpp"

#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <NvInferPlugin.h>

namespace aiformula {

namespace {

std::size_t elementByteSize(const nvinfer1::DataType type) {
    switch (type) {
        case nvinfer1::DataType::kFLOAT:
            return 4;
        case nvinfer1::DataType::kHALF:
            return 2;
        case nvinfer1::DataType::kINT32:
            return 4;
        case nvinfer1::DataType::kINT64:
            return 8;
        case nvinfer1::DataType::kINT8:
        case nvinfer1::DataType::kUINT8:
        case nvinfer1::DataType::kBOOL:
            return 1;
        default:
            throw std::runtime_error("unsupported TensorRT data type");
    }
}

std::vector<char> readFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("cannot open engine file '" + path + "'");
    const auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> contents(static_cast<std::size_t>(size));
    if (!file.read(contents.data(), size)) throw std::runtime_error("cannot read engine file '" + path + "'");
    return contents;
}

}  // namespace

class TrtEngine::TrtLogger : public nvinfer1::ILogger {
public:
    explicit TrtLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    void log(const Severity severity, const char* const message) noexcept override {
        switch (severity) {
            case Severity::kINTERNAL_ERROR:
            case Severity::kERROR:
                RCLCPP_ERROR(logger_, "[TensorRT] %s", message);
                break;
            case Severity::kWARNING:
                RCLCPP_WARN(logger_, "[TensorRT] %s", message);
                break;
            case Severity::kINFO:
                RCLCPP_INFO(logger_, "[TensorRT] %s", message);
                break;
            case Severity::kVERBOSE:
                RCLCPP_DEBUG(logger_, "[TensorRT] %s", message);
                break;
        }
    }

private:
    rclcpp::Logger logger_;
};

TrtEngine::TrtEngine(const std::string& engine_path, const rclcpp::Logger& logger)
    : trt_logger_(std::make_unique<TrtLogger>(logger)) {
    // The engine contains an EfficientNMS_TRT layer, whose implementation lives in the plugin
    // library rather than the core runtime.
    if (!initLibNvInferPlugins(trt_logger_.get(), "")) {
        throw std::runtime_error("failed to register the TensorRT plugins");
    }

    const auto serialized = readFile(engine_path);
    runtime_.reset(nvinfer1::createInferRuntime(*trt_logger_));
    if (!runtime_) throw std::runtime_error("failed to create the TensorRT runtime");

    engine_.reset(runtime_->deserializeCudaEngine(serialized.data(), serialized.size()));
    if (!engine_) throw std::runtime_error("failed to deserialize '" + engine_path + "'");

    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("failed to create an execution context");
}

TrtEngine::~TrtEngine() = default;

bool TrtEngine::hasTensor(const std::string& name) const {
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
        if (name == engine_->getIOTensorName(index)) return true;
    }
    return false;
}

std::vector<int64_t> TrtEngine::tensorShape(const std::string& name) const {
    const auto dims = engine_->getTensorShape(name.c_str());
    if (dims.nbDims < 0) throw std::runtime_error("engine has no tensor named '" + name + "'");
    return std::vector<int64_t>(dims.d, dims.d + dims.nbDims);
}

std::size_t TrtEngine::tensorByteSize(const std::string& name) const {
    const auto shape = tensorShape(name);
    const auto elements = std::accumulate(shape.begin(), shape.end(), static_cast<std::size_t>(1),
                                          [](const std::size_t total, const int64_t extent) {
                                              return total * static_cast<std::size_t>(extent);
                                          });
    return elements * elementByteSize(engine_->getTensorDataType(name.c_str()));
}

void TrtEngine::bind(const std::string& name, void* const device_address) {
    if (!context_->setTensorAddress(name.c_str(), device_address)) {
        throw std::runtime_error("failed to bind the device address of '" + name + "'");
    }
}

void TrtEngine::enqueue(cudaStream_t stream) {
    if (!context_->enqueueV3(stream)) throw std::runtime_error("failed to enqueue the inference");
}

std::string TrtEngine::describe() const {
    std::ostringstream description;
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
        const auto* const name = engine_->getIOTensorName(index);
        const auto is_input = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
        description << "\n  " << (is_input ? "in  " : "out ") << name << " [";
        const auto shape = tensorShape(name);
        for (std::size_t axis = 0; axis < shape.size(); ++axis) {
            description << (axis > 0 ? "x" : "") << shape[axis];
        }
        description << "]";
    }
    return description.str();
}

}  // namespace aiformula
