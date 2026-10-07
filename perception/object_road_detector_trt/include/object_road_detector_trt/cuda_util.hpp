#ifndef OBJECT_ROAD_DETECTOR_TRT_CUDA_UTIL_HPP
#define OBJECT_ROAD_DETECTOR_TRT_CUDA_UTIL_HPP

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace aiformula {

class CudaException : public std::runtime_error {
public:
    explicit CudaException(const std::string& message) : std::runtime_error(message) {}
};

inline void cudaCheck(const cudaError_t status, const char* const expression, const char* const file, const int line) {
    if (status == cudaSuccess) return;
    throw CudaException(std::string(file) + ":" + std::to_string(line) + " '" + expression + "' failed: " +
                        cudaGetErrorString(status));
}

#define CUDA_CHECK(expression) ::aiformula::cudaCheck((expression), #expression, __FILE__, __LINE__)

/**
 * @brief Device allocation that only ever grows, so a steady stream of same-sized frames never
 * reallocates.
 */
class DeviceBuffer {
public:
    DeviceBuffer() = default;
    ~DeviceBuffer() { release(); }

    DeviceBuffer(const DeviceBuffer&) = delete;
    DeviceBuffer& operator=(const DeviceBuffer&) = delete;

    DeviceBuffer(DeviceBuffer&& other) noexcept : data_(other.data_), bytes_(other.bytes_) {
        other.data_ = nullptr;
        other.bytes_ = 0;
    }
    DeviceBuffer& operator=(DeviceBuffer&& other) noexcept {
        if (this != &other) {
            release();
            std::swap(data_, other.data_);
            std::swap(bytes_, other.bytes_);
        }
        return *this;
    }

    void reserve(const std::size_t bytes) {
        if (bytes <= bytes_) return;
        release();
        CUDA_CHECK(cudaMalloc(&data_, bytes));
        bytes_ = bytes;
    }

    template <typename T = void>
    T* data() const {
        return static_cast<T*>(data_);
    }
    std::size_t bytes() const { return bytes_; }

private:
    void release() {
        if (data_) cudaFree(data_);
        data_ = nullptr;
        bytes_ = 0;
    }

    void* data_{nullptr};
    std::size_t bytes_{0};
};

/**
 * @brief Page-locked host allocation, so the small result transfers can overlap the stream instead
 * of forcing the driver through a staging copy.
 */
class HostPinnedBuffer {
public:
    HostPinnedBuffer() = default;
    ~HostPinnedBuffer() { release(); }

    HostPinnedBuffer(const HostPinnedBuffer&) = delete;
    HostPinnedBuffer& operator=(const HostPinnedBuffer&) = delete;

    void reserve(const std::size_t bytes) {
        if (bytes <= bytes_) return;
        release();
        CUDA_CHECK(cudaHostAlloc(&data_, bytes, cudaHostAllocDefault));
        bytes_ = bytes;
    }

    template <typename T = void>
    T* data() const {
        return static_cast<T*>(data_);
    }
    std::size_t bytes() const { return bytes_; }

private:
    void release() {
        if (data_) cudaFreeHost(data_);
        data_ = nullptr;
        bytes_ = 0;
    }

    void* data_{nullptr};
    std::size_t bytes_{0};
};

}  // namespace aiformula

#endif  // OBJECT_ROAD_DETECTOR_TRT_CUDA_UTIL_HPP
