// C entry points onto depthai-core, which is C++17 and ships no C API of its
// own. Everything here is mechanical: build the pipeline the settings ask for,
// hand frames back one at a time, and expose the factory calibration. No
// decision about what a frame means is taken on this side of the boundary.
//
// Frame data is not copied. Each stream keeps the message it last returned
// alive until the next call for that same stream, and every stream is polled
// from its own thread, so no lock is needed and no pointer outlives its owner.

#include <depthai/depthai.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace {

// Mirrors the Rust side of the boundary; see oakd_ffi.rs.
constexpr int32_t STREAM_DEPTH = 0;
constexpr int32_t STREAM_COLOR = 1;
constexpr int32_t STREAM_INFRA_LEFT = 2;
constexpr int32_t STREAM_INFRA_RIGHT = 3;
constexpr int32_t STREAM_IMU = 4;
constexpr int32_t STREAM_COUNT = 5;

constexpr int32_t PIXELS_MONO8 = 0;
constexpr int32_t PIXELS_MONO16 = 1;
constexpr int32_t PIXELS_BGR8 = 2;

struct StreamSlot {
    std::shared_ptr<dai::MessageQueue> queue;
    // Keeps the last returned message alive for as long as its pointer is out.
    std::shared_ptr<dai::ADatatype> held;
    std::deque<dai::IMUPacket> pending;
    int32_t pixels = PIXELS_MONO8;
};

void writeError(char* error, size_t capacity, const std::string& text) {
    if(error == nullptr || capacity == 0) {
        return;
    }
    const size_t length = text.size() < capacity - 1 ? text.size() : capacity - 1;
    std::memcpy(error, text.data(), length);
    error[length] = '\0';
}

uint64_t steadyNanos(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> point) {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(point.time_since_epoch()).count());
}

}  // namespace

struct LrOakDevice {
    std::shared_ptr<dai::Device> device;
    std::unique_ptr<dai::Pipeline> pipeline;
    StreamSlot slots[STREAM_COUNT];
    dai::CameraBoardSocket depthSocket = dai::CameraBoardSocket::CAM_B;
};

extern "C" {

struct LrOakConfig {
    int32_t color;
    int32_t depth;
    int32_t infrared;
    int32_t imu;
    int32_t width;
    int32_t height;
    int32_t frame_rate;
    int32_t imu_rate;
    int32_t emitter;
    int32_t align_depth_to_color;
    const char* serial;
};

struct LrOakSample {
    int32_t stream;
    int32_t pixels;
    int32_t width;
    int32_t height;
    const uint8_t* data;
    uint64_t length;
    uint64_t device_stamp_nanos;
    double accelerometer[3];
    double gyroscope[3];
};

struct LrOakCalibration {
    int32_t model;
    int32_t width;
    int32_t height;
    double intrinsics[9];
    double coefficients[14];
    int32_t coefficient_count;
    double baseline_centimetres;
};

/// Nanoseconds on the same steady clock depthai stamps its messages with. The
/// Rust side pairs one of these with a wall-clock reading to place the whole
/// stream on the epoch.
uint64_t lr_oak_steady_now_nanos(void) {
    return steadyNanos(std::chrono::steady_clock::now());
}

LrOakDevice* lr_oak_open(const LrOakConfig* config, char* error, size_t error_capacity) {
    if(config == nullptr) {
        writeError(error, error_capacity, "no configuration given");
        return nullptr;
    }
    try {
        auto handle = std::make_unique<LrOakDevice>();

        if(config->serial != nullptr && config->serial[0] != '\0') {
            handle->device = std::make_shared<dai::Device>(dai::DeviceInfo(std::string(config->serial)));
        } else {
            handle->device = std::make_shared<dai::Device>();
        }

        // The dot projector is the OAK-D Pro's own addition and the reason
        // stereo works on a blank wall. Off means plain passive stereo, which is
        // what feature tracking wants.
        handle->device->setIrLaserDotProjectorIntensity(config->emitter != 0 ? 1.0f : 0.0f);

        handle->pipeline = std::make_unique<dai::Pipeline>(handle->device);
        auto& pipeline = *handle->pipeline;

        const std::pair<uint32_t, uint32_t> size{static_cast<uint32_t>(config->width), static_cast<uint32_t>(config->height)};
        const float fps = static_cast<float>(config->frame_rate);

        const bool needsMono = config->depth != 0 || config->infrared != 0;
        dai::Node::Output* leftOutput = nullptr;
        dai::Node::Output* rightOutput = nullptr;
        if(needsMono) {
            auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
            auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
            leftOutput = left->requestOutput(size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps);
            rightOutput = right->requestOutput(size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps);
        }

        if(config->color != 0) {
            auto colour = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
            auto* output = colour->requestOutput(size, dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::CROP, fps);
            handle->slots[STREAM_COLOR].queue = output->createOutputQueue(4, false);
            handle->slots[STREAM_COLOR].pixels = PIXELS_BGR8;
        }

        if(config->infrared != 0) {
            handle->slots[STREAM_INFRA_LEFT].queue = leftOutput->createOutputQueue(4, false);
            handle->slots[STREAM_INFRA_LEFT].pixels = PIXELS_MONO8;
            handle->slots[STREAM_INFRA_RIGHT].queue = rightOutput->createOutputQueue(4, false);
            handle->slots[STREAM_INFRA_RIGHT].pixels = PIXELS_MONO8;
        }

        if(config->depth != 0) {
            auto stereo = pipeline.create<dai::node::StereoDepth>();
            stereo->build(*leftOutput, *rightOutput, dai::node::StereoDepth::PresetMode::DEFAULT);
            stereo->setLeftRightCheck(true);
            if(config->align_depth_to_color != 0) {
                stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
                handle->depthSocket = dai::CameraBoardSocket::CAM_A;
            }
            handle->slots[STREAM_DEPTH].queue = stereo->depth.createOutputQueue(4, false);
            handle->slots[STREAM_DEPTH].pixels = PIXELS_MONO16;
        }

        if(config->imu != 0) {
            auto imu = pipeline.create<dai::node::IMU>();
            imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, config->imu_rate);
            imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, config->imu_rate);
            // One report per message keeps the batching latency at a single
            // sample, which is what a recorder wants; the queue absorbs bursts.
            imu->setBatchReportThreshold(1);
            imu->setMaxBatchReports(20);
            handle->slots[STREAM_IMU].queue = imu->out.createOutputQueue(100, false);
        }

        pipeline.start();
        return handle.release();
    } catch(const std::exception& failure) {
        writeError(error, error_capacity, failure.what());
        return nullptr;
    }
}

void lr_oak_close(LrOakDevice* handle) {
    if(handle == nullptr) {
        return;
    }
    try {
        if(handle->pipeline) {
            handle->pipeline->stop();
            handle->pipeline->wait();
        }
    } catch(const std::exception&) {
        // Closing is best effort: the device is being given up either way.
    }
    delete handle;
}

int32_t lr_oak_has_stream(const LrOakDevice* handle, int32_t stream) {
    if(handle == nullptr || stream < 0 || stream >= STREAM_COUNT) {
        return 0;
    }
    return handle->slots[stream].queue ? 1 : 0;
}

/// 1 when a sample was written, 0 on timeout, -1 on failure.
int32_t lr_oak_wait(LrOakDevice* handle, int32_t stream, int32_t timeout_ms, LrOakSample* out, char* error, size_t error_capacity) {
    if(handle == nullptr || out == nullptr || stream < 0 || stream >= STREAM_COUNT) {
        writeError(error, error_capacity, "no such stream");
        return -1;
    }
    StreamSlot& slot = handle->slots[stream];
    if(!slot.queue) {
        writeError(error, error_capacity, "stream is not enabled");
        return -1;
    }

    std::memset(out, 0, sizeof(*out));
    out->stream = stream;

    try {
        if(stream == STREAM_IMU) {
            if(slot.pending.empty()) {
                bool timedOut = false;
                auto data = slot.queue->get<dai::IMUData>(std::chrono::milliseconds(timeout_ms), timedOut);
                if(timedOut || data == nullptr) {
                    return 0;
                }
                for(const auto& packet : data->packets) {
                    slot.pending.push_back(packet);
                }
                if(slot.pending.empty()) {
                    return 0;
                }
            }
            const dai::IMUPacket packet = slot.pending.front();
            slot.pending.pop_front();
            out->accelerometer[0] = packet.acceleroMeter.x;
            out->accelerometer[1] = packet.acceleroMeter.y;
            out->accelerometer[2] = packet.acceleroMeter.z;
            out->gyroscope[0] = packet.gyroscope.x;
            out->gyroscope[1] = packet.gyroscope.y;
            out->gyroscope[2] = packet.gyroscope.z;
            // The gyroscope's stamp, not the accelerometer's: the two parts are
            // sampled independently and a ROS Imu carries one time for both, so
            // one has to be picked and named. Orientation integrates the
            // gyroscope, so its stamp is the one that matters.
            out->device_stamp_nanos = steadyNanos(packet.gyroscope.getTimestamp());
            return 1;
        }

        bool timedOut = false;
        auto frame = slot.queue->get<dai::ImgFrame>(std::chrono::milliseconds(timeout_ms), timedOut);
        if(timedOut || frame == nullptr) {
            return 0;
        }
        auto data = frame->getData();
        out->pixels = slot.pixels;
        out->width = static_cast<int32_t>(frame->getWidth());
        out->height = static_cast<int32_t>(frame->getHeight());
        out->data = data.data();
        out->length = data.size();
        out->device_stamp_nanos = steadyNanos(frame->getTimestamp());
        slot.held = frame;
        return 1;
    } catch(const std::exception& failure) {
        writeError(error, error_capacity, failure.what());
        return -1;
    }
}

/// The factory intrinsics for the imager behind a stream, already scaled to the
/// resolution being streamed. Returns 0 on success.
int32_t lr_oak_calibration(LrOakDevice* handle, int32_t socket, int32_t width, int32_t height, LrOakCalibration* out) {
    if(handle == nullptr || out == nullptr) {
        return -1;
    }
    try {
        std::memset(out, 0, sizeof(*out));
        auto calibration = handle->device->readCalibration();
        const auto board = static_cast<dai::CameraBoardSocket>(socket);

        const auto intrinsics = calibration.getCameraIntrinsics(board, width, height);
        for(size_t row = 0; row < 3 && row < intrinsics.size(); ++row) {
            for(size_t column = 0; column < 3 && column < intrinsics[row].size(); ++column) {
                out->intrinsics[row * 3 + column] = static_cast<double>(intrinsics[row][column]);
            }
        }

        const auto coefficients = calibration.getDistortionCoefficients(board);
        const size_t count = coefficients.size() < 14 ? coefficients.size() : 14;
        for(size_t index = 0; index < count; ++index) {
            out->coefficients[index] = static_cast<double>(coefficients[index]);
        }
        out->coefficient_count = static_cast<int32_t>(count);
        out->model = static_cast<int32_t>(calibration.getDistortionModel(board));
        out->width = width;
        out->height = height;

        // Only the right imager carries a baseline, exactly as on a RealSense:
        // it is the term that turns a disparity into a depth.
        if(board == dai::CameraBoardSocket::CAM_C) {
            out->baseline_centimetres = static_cast<double>(calibration.getBaselineDistance());
        }
        return 0;
    } catch(const std::exception&) {
        return -1;
    }
}

namespace {

int32_t copyTransform(const std::vector<std::vector<float>>& matrix, double* rotation, double* translation) {
    if(matrix.size() < 3) {
        return -1;
    }
    for(size_t row = 0; row < 3; ++row) {
        if(matrix[row].size() < 4) {
            return -1;
        }
        for(size_t column = 0; column < 3; ++column) {
            rotation[row * 3 + column] = static_cast<double>(matrix[row][column]);
        }
        translation[row] = static_cast<double>(matrix[row][3]);
    }
    return 0;
}

}  // namespace

/// The rigid transform from the left imager to another socket, as the factory
/// measured it. Rotation is row-major 3x3; translation is centimetres, which is
/// the unit depthai reports and the Rust side converts.
int32_t lr_oak_extrinsics(LrOakDevice* handle, int32_t socket, double* rotation, double* translation) {
    if(handle == nullptr || rotation == nullptr || translation == nullptr) {
        return -1;
    }
    try {
        auto calibration = handle->device->readCalibration();
        return copyTransform(calibration.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, static_cast<dai::CameraBoardSocket>(socket)),
                             rotation,
                             translation);
    } catch(const std::exception&) {
        return -1;
    }
}

/// The same edge for the inertial part, which is not addressed by a socket and
/// so needs its own call. Taken in the camera-to-IMU direction so it hangs off
/// the left imager like every other frame.
int32_t lr_oak_imu_extrinsics(LrOakDevice* handle, double* rotation, double* translation) {
    if(handle == nullptr || rotation == nullptr || translation == nullptr) {
        return -1;
    }
    try {
        auto calibration = handle->device->readCalibration();
        return copyTransform(calibration.getCameraToImuExtrinsics(dai::CameraBoardSocket::CAM_B), rotation, translation);
    } catch(const std::exception&) {
        return -1;
    }
}

int32_t lr_oak_set_emitter(LrOakDevice* handle, int32_t on) {
    if(handle == nullptr) {
        return -1;
    }
    try {
        handle->device->setIrLaserDotProjectorIntensity(on != 0 ? 1.0f : 0.0f);
        return 0;
    } catch(const std::exception&) {
        return -1;
    }
}

}  // extern "C"
