#include "camera_realsense.hpp"

#include <arpa/inet.h>
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <pthread.h>
#include <signal.h>
#include <turbojpeg.h>

#include <algorithm>
#include <condition_variable>
#include <future>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

#include "third_party/fpng.h"
#include "third_party/lodepng.h"

#ifndef htonll
#define htonll(x) \
    ((1 == htonl(1)) ? (x) : ((uint64_t)htonl((x)&0xFFFFFFFF) << 32) | htonl((x) >> 32))
#endif

namespace {
bool debug_enabled = false;
const uint32_t rgbaMagicNumber =
    htonl(1380401729);  // the utf-8 binary encoding for "RGBA", big-endian
const size_t rgbaMagicByteCount =
    sizeof(uint32_t);  // number of bytes used to represent the rgba magic number
const size_t rgbaWidthByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image width
const size_t rgbaHeightByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image height

const uint64_t depthMagicNumber =
    htonll(4919426490892632400);  // the utf-8 binary encoding for "DEPTHMAP", big-endian
const size_t depthMagicByteCount =
    sizeof(uint64_t);  // number of bytes used to represent the depth magic number
const size_t depthWidthByteCount =
    sizeof(uint64_t);  // number of bytes used to represent depth image width
const size_t depthHeightByteCount =
    sizeof(uint64_t);  // number of bytes used to represent depth image height

// COLOR responses
struct color_response {
    std::vector<uint8_t> color_bytes;
};

color_response encodeColorPNG(const void* data, const uint width, const uint height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    std::vector<uint8_t> encoded;
    if (!fpng::fpng_encode_image_to_memory(data, width, height, 3, encoded)) {
        throw std::runtime_error("failed to encode color PNG");
    }

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  PNG color encode:      " << duration.count() << "ms\n";
    }
    return {std::move(encoded)};
}

std::unique_ptr<viam::sdk::Camera::raw_image> encodeColorPNGToResponse(const void* data,
                                                                       const uint width,
                                                                       const uint height) {
    color_response encoded = encodeColorPNG(data, width, height);
    auto response = std::make_unique<viam::sdk::Camera::raw_image>();
    response->source_name = "color";
    response->mime_type = "image/png";
    response->bytes = std::move(encoded.color_bytes);
    return response;
}

struct jpeg_image {
    std::unique_ptr<unsigned char, decltype(&tjFree)> data;
    long unsigned int size;
    // constructor
    jpeg_image(unsigned char* imageData, long unsigned int imageSize)
        : data(imageData, &tjFree), size(imageSize) {}
};

jpeg_image encodeJPEG(const unsigned char* data, const uint width, const uint height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    unsigned char* encoded = nullptr;
    long unsigned int encodedSize = 0;
    tjhandle handle = tjInitCompress();
    if (handle == nullptr) {
        throw std::runtime_error("[GetImage] failed to init JPEG compressor");
    }
    int success;
    try {
        success = tjCompress2(handle, data, width, 0, height, TJPF_RGB, &encoded, &encodedSize,
                              TJSAMP_420, 75, TJFLAG_FASTDCT);
        tjDestroy(handle);
    } catch (const std::exception& e) {
        tjDestroy(handle);
        throw std::runtime_error("[GetImage] JPEG compressor failed to compress: " +
                                 std::string(e.what()));
    }
    if (success != 0) {
        throw std::runtime_error("[GetImage] JPEG compressor failed to compress image");
    }
    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  JPEG color encode:     " << duration.count() << "ms\n";
    }

    jpeg_image output(encoded, encodedSize);
    return output;
}

std::unique_ptr<viam::sdk::Camera::raw_image> encodeJPEGToResponse(const unsigned char* data,
                                                                   const uint width,
                                                                   const uint height) {
    jpeg_image encoded = encodeJPEG(data, width, height);
    auto response = std::make_unique<viam::sdk::Camera::raw_image>();
    response->source_name = "color";
    response->mime_type = "image/jpeg";
    response->bytes.assign(encoded.data.get(), encoded.data.get() + encoded.size);
    return response;
}

struct raw_camera_image {
    using deleter_type = void (*)(unsigned char*);
    using uniq = std::unique_ptr<unsigned char[], deleter_type>;

    static constexpr deleter_type free_deleter = [](unsigned char* ptr) { free(ptr); };

    static constexpr deleter_type array_delete_deleter = [](unsigned char* ptr) { delete[] ptr; };

    uniq bytes;
    size_t size;
};

raw_camera_image encodeColorRAW(const unsigned char* data, const uint32_t width,
                                const uint32_t height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }
    // set size of raw file
    size_t pixelByteCount = 4 * width * height;
    uint32_t widthToEncode = htonl(width);    // make sure everything is big-endian
    uint32_t heightToEncode = htonl(height);  // make sure everything is big-endian
    size_t totalByteCount =
        rgbaMagicByteCount + rgbaWidthByteCount + rgbaHeightByteCount + pixelByteCount;
    // memcpy data into buffer
    raw_camera_image::uniq rawBuf(new unsigned char[totalByteCount],
                                  raw_camera_image::array_delete_deleter);
    int offset = 0;
    std::memcpy(rawBuf.get() + offset, &rgbaMagicNumber, rgbaMagicByteCount);
    offset += rgbaMagicByteCount;
    std::memcpy(rawBuf.get() + offset, &widthToEncode, rgbaWidthByteCount);
    offset += rgbaWidthByteCount;
    std::memcpy(rawBuf.get() + offset, &heightToEncode, rgbaHeightByteCount);
    offset += rgbaHeightByteCount;
    int pixelOffset = 0;
    uint8_t alphaValue = 255;  // alpha  channel is always 255 for color images
    for (int i = 0; i < width * height; i++) {
        std::memcpy(rawBuf.get() + offset, data + pixelOffset, 3);  // 3 bytes for RGB
        std::memcpy(rawBuf.get() + offset + 3, &alphaValue, 1);     // 1 byte for A
        pixelOffset += 3;
        offset += 4;
    }
    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  RAW color encode:      " << duration.count() << "ms\n";
    }
    return {std::move(rawBuf), totalByteCount};
}

std::unique_ptr<viam::sdk::Camera::raw_image> encodeColorRAWToResponse(const unsigned char* data,
                                                                       const uint width,
                                                                       const uint height) {
    raw_camera_image encoded = encodeColorRAW(data, width, height);
    auto response = std::make_unique<viam::sdk::Camera::raw_image>();
    response->source_name = "color";
    response->mime_type = "image/vnd.viam.rgba";
    response->bytes.assign(encoded.bytes.get(), encoded.bytes.get() + encoded.size);
    return response;
}

// DEPTH responses
raw_camera_image encodeDepthPNG(const unsigned char* data, const uint width, const uint height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    // convert data to guarantee big-endian
    size_t pixelByteCount = 2 * width * height;
    std::unique_ptr<unsigned char[]> rawBuf(new unsigned char[pixelByteCount]);
    int offset = 0;
    int pixelOffset = 0;
    for (int i = 0; i < width * height; i++) {
        uint16_t pix;
        std::memcpy(&pix, data + pixelOffset, 2);
        uint16_t pixEncode = htons(pix);  // PNG expects pixel values to be big-endian
        std::memcpy(rawBuf.get() + offset, &pixEncode, 2);
        pixelOffset += 2;
        offset += 2;
    }
    unsigned char* encoded = 0;
    size_t encoded_size = 0;
    unsigned result =
        lodepng_encode_memory(&encoded, &encoded_size, rawBuf.get(), width, height, LCT_GREY, 16);
    raw_camera_image::uniq uniqueEncoded(encoded, raw_camera_image::free_deleter);
    if (result != 0) {
        throw std::runtime_error("[GetImage]  failed to encode depth PNG");
    }

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  PNG depth encode:      " << duration.count() << "ms\n";
    }

    return {std::move(uniqueEncoded), encoded_size};
}

std::unique_ptr<viam::sdk::Camera::raw_image> encodeDepthPNGToResponse(const unsigned char* data,
                                                                       const uint width,
                                                                       const uint height) {
    raw_camera_image encoded = encodeDepthPNG(data, width, height);
    auto response = std::make_unique<viam::sdk::Camera::raw_image>();
    response->source_name = "depth";
    response->mime_type = "image/png";
    response->bytes.assign(encoded.bytes.get(), encoded.bytes.get() + encoded.size);
    return response;
}

raw_camera_image encodeDepthRAW(const unsigned char* data, const uint64_t width,
                                const uint64_t height, const bool littleEndian) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }
    // Depth header contains 8 bytes worth of magic number, followed by 8 bytes for width and
    // another 8 bytes for height each pixel has 2 bytes.
    size_t pixelByteCount = 2 * width * height;
    uint64_t widthToEncode = htonll(width);    // make sure everything is big-endian
    uint64_t heightToEncode = htonll(height);  // make sure everything is big-endian
    size_t totalByteCount =
        depthMagicByteCount + depthWidthByteCount + depthHeightByteCount + pixelByteCount;
    // memcpy data into buffer
    raw_camera_image::uniq rawBuf(new unsigned char[totalByteCount],
                                  raw_camera_image::array_delete_deleter);
    int offset = 0;
    std::memcpy(rawBuf.get() + offset, &depthMagicNumber, depthMagicByteCount);
    offset += depthMagicByteCount;
    std::memcpy(rawBuf.get() + offset, &widthToEncode, depthWidthByteCount);
    offset += depthWidthByteCount;
    std::memcpy(rawBuf.get() + offset, &heightToEncode, depthHeightByteCount);
    offset += depthHeightByteCount;
    if (littleEndian) {
        std::memcpy(rawBuf.get() + offset, data, pixelByteCount);
    } else {
        int pixelOffset = 0;
        for (int i = 0; i < width * height; i++) {
            uint16_t pix;
            std::memcpy(&pix, data + pixelOffset, 2);
            uint16_t pixEncode = htons(pix);  // make sure the pixel values are big-endian
            std::memcpy(rawBuf.get() + offset, &pixEncode, 2);
            pixelOffset += 2;
            offset += 2;
        }
    }

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  RAW depth encode:      " << duration.count() << "ms\n";
    }

    return {std::move(rawBuf), std::move(totalByteCount)};
}

std::unique_ptr<viam::sdk::Camera::raw_image> encodeDepthRAWToResponse(const unsigned char* data,
                                                                       const uint width,
                                                                       const uint height,
                                                                       const bool littleEndian) {
    raw_camera_image encoded = encodeDepthRAW(data, width, height, littleEndian);
    auto response = std::make_unique<viam::sdk::Camera::raw_image>();
    response->source_name = "depth";
    response->mime_type = "image/vnd.viam.dep";
    response->bytes.assign(encoded.bytes.get(), encoded.bytes.get() + encoded.size);
    return response;
}
}  // namespace

namespace viam {
namespace realsense {

// Global AtomicFrameSet
AtomicFrameSet GLOBAL_LATEST_FRAMES;
// align to the color camera's origin when color and depth enabled
const rs2::align FRAME_ALIGNMENT = RS2_STREAM_COLOR;

// CAMERA module methods

// initialize will use the ResourceConfigs to begin the realsense pipeline.
std::tuple<RealSenseProperties, bool, bool> CameraRealSense::initialize(sdk::ResourceConfig cfg) {
    if (device_ != nullptr) {
        std::cout << "reinitializing, restarting pipeline" << std::endl;
        {
            // wait until frameLoop is stopped
            std::unique_lock<std::mutex> lock(device_->mutex);
            device_->shouldRun = false;
            device_->cv.wait(lock, [this] { return !(device_->isRunning); });
        }
    }
    std::cout << "initializing the Intel RealSense Camera Module" << std::endl;
    // set variables from config
    uint width = 0;
    uint height = 0;
    auto attrs = cfg.attributes();
    if (attrs->count("width_px") == 1) {
        std::shared_ptr<sdk::ProtoType> width_proto = attrs->at("width_px");
        auto width_value = width_proto->proto_value();
        if (width_value.has_number_value()) {
            uint width_num = static_cast<uint>(width_value.number_value());
            width = width_num;
        }
    }
    if (attrs->count("height_px") == 1) {
        std::shared_ptr<sdk::ProtoType> height_proto = attrs->at("height_px");
        auto height_value = height_proto->proto_value();
        if (height_value.has_number_value()) {
            uint height_num = static_cast<uint>(height_value.number_value());
            height = height_num;
        }
    }
    if (width == 0 || height == 0) {
        std::cout << "note: will pick any suitable width and height" << std::endl;
    }
    if (attrs->count("debug") == 1) {
        std::shared_ptr<sdk::ProtoType> debug_proto = attrs->at("debug");
        auto debug_value = debug_proto->proto_value();
        if (debug_value.has_bool_value()) {
            bool debug_bool = static_cast<bool>(debug_value.bool_value());
            debug_enabled = debug_bool;
        }
    }
    bool littleEndianDepth = false;
    if (attrs->count("little_endian_depth") == 1) {
        std::shared_ptr<sdk::ProtoType> endian_proto = attrs->at("little_endian_depth");
        auto endian_value = endian_proto->proto_value();
        if (endian_value.has_bool_value()) {
            bool endian_bool = static_cast<bool>(endian_value.bool_value());
            littleEndianDepth = endian_bool;
        }
    }
    bool enablePointClouds = false;
    if (attrs->count("enable_point_clouds") == 1) {
        std::shared_ptr<sdk::ProtoType> pointclouds_proto = attrs->at("enable_point_clouds");
        auto pointclouds_value = pointclouds_proto->proto_value();
        if (pointclouds_value.has_bool_value()) {
            bool pointclouds_bool = static_cast<bool>(pointclouds_value.bool_value());
            enablePointClouds = pointclouds_bool;
        }
    }
    bool disableDepth = true;
    bool disableColor = true;
    std::vector<std::string> sensors;
    if (attrs->count("sensors") == 1) {
        std::shared_ptr<sdk::ProtoType> sensor_proto = attrs->at("sensors");
        auto sensor_value = sensor_proto->proto_value();
        if (sensor_value.has_list_value()) {
            auto sensor_list = sensor_value.list_value();
            for (const auto element : sensor_list.values()) {
                if (element.has_string_value()) {
                    std::string sensor_name = static_cast<std::string>(element.string_value());
                    if (sensor_name == "color") {
                        disableColor = false;
                        sensors.push_back("color");
                    }
                    if (sensor_name == "depth") {
                        disableDepth = false;
                        sensors.push_back("depth");
                    }
                }
            }
        }
    }
    if (disableColor && disableDepth) {
        throw std::runtime_error("cannot disable both color and depth");
    }

    // DeviceProperties context also holds a bool that can stop the thread if device gets
    // disconnected
    std::shared_ptr<DeviceProperties> newDevice = std::make_shared<DeviceProperties>(
        width, height, disableColor, width, height, disableDepth);
    device_ = std::move(newDevice);

    // First start of Pipeline
    rs2::pipeline pipe;
    RealSenseProperties props;
    std::tie(pipe, props) = startPipeline(disableDepth, width, height, disableColor, width, height);
    // First start of camera thread
    props.sensors = sensors;
    props.mainSensor = sensors[0];
    std::cout << "main sensor will be " << sensors[0] << std::endl;
    props.littleEndianDepth = littleEndianDepth;
    if (props.mainSensor == "depth") {
        std::cout << std::boolalpha << "depth little endian encoded: " << littleEndianDepth
                  << std::endl;
    }
    props.enablePointClouds = enablePointClouds;
    std::string pointcloudString = (enablePointClouds) ? "true" : "false";
    std::cout << "point clouds enabled: " << pointcloudString << std::endl;
    std::promise<void> ready;
    std::thread cameraThread(frameLoop, pipe, ref(ready), device_, props.depthScaleMm);
    std::cout << "waiting for camera frame loop thread to be ready..." << std::endl;
    ready.get_future().wait();
    std::cout << "camera frame loop ready!" << std::endl;
    cameraThread.detach();
    return std::make_tuple(props, disableColor, disableDepth);
}

CameraRealSense::CameraRealSense(sdk::Dependencies deps, sdk::ResourceConfig cfg)
    : Camera(cfg.name()) {
    RealSenseProperties props;
    bool disableColor;
    bool disableDepth;
    try {
        std::tie(props, disableColor, disableDepth) = initialize(cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to initialize realsense: " + std::string(e.what()));
    }
    this->props_ = props;
    this->disableColor_ = disableColor;
    this->disableDepth_ = disableDepth;
}

CameraRealSense::~CameraRealSense() {
    // stop and wait for the frameLoop thread to exit
    if (!this->device_) return;
    // wait until frameLoop is stopped
    std::unique_lock<std::mutex> lock(this->device_->mutex);
    this->device_->shouldRun = false;
    this->device_->cv.wait(lock, [this] { return !(device_->isRunning); });
}

void CameraRealSense::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    RealSenseProperties props;
    bool disableColor;
    bool disableDepth;
    try {
        std::tie(props, disableColor, disableDepth) = initialize(cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to reconfigure realsense: " + std::string(e.what()));
    }
    this->props_ = props;
    this->disableColor_ = disableColor;
    this->disableDepth_ = disableDepth;
}

sdk::Camera::raw_image CameraRealSense::get_image(std::string mime_type,
                                                  const sdk::AttributeMap& extra) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    rs2::frame latestColorFrame;
    std::shared_ptr<std::vector<uint16_t>> latestDepthFrame;
    {
        std::lock_guard<std::mutex> lock(GLOBAL_LATEST_FRAMES.mutex);
        latestColorFrame = GLOBAL_LATEST_FRAMES.colorFrame;
        latestDepthFrame = GLOBAL_LATEST_FRAMES.depthFrame;
    }
    std::unique_ptr<sdk::Camera::raw_image> response;
    if (this->props_.mainSensor.compare("color") == 0) {
        if (this->disableColor_) {
            throw std::invalid_argument("color disabled");
        }
        if (mime_type.compare("image/png") == 0 || mime_type.compare("image/png+lazy") == 0) {
            response =
                encodeColorPNGToResponse((const void*)latestColorFrame.get_data(),
                                         this->props_.color.width, this->props_.color.height);
        } else if (mime_type.compare("image/vnd.viam.rgba") == 0) {
            response =
                encodeColorRAWToResponse((const unsigned char*)latestColorFrame.get_data(),
                                         this->props_.color.width, this->props_.color.height);
        } else {
            response = encodeJPEGToResponse((const unsigned char*)latestColorFrame.get_data(),
                                            this->props_.color.width, this->props_.color.height);
        }
    } else if (this->props_.mainSensor.compare("depth") == 0) {
        if (this->disableDepth_) {
            throw std::invalid_argument("depth disabled");
        }
        if (mime_type.compare("image/vnd.viam.dep") == 0) {
            response = encodeDepthRAWToResponse((const unsigned char*)latestDepthFrame->data(),
                                                this->props_.depth.width, this->props_.depth.height,
                                                this->props_.littleEndianDepth);
        } else {
            response =
                encodeDepthPNGToResponse((const unsigned char*)latestDepthFrame->data(),
                                         this->props_.depth.width, this->props_.depth.height);
        }
    }

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[get_image]  total:           " << duration.count() << "ms\n";
    }

    return std::move(*response);
}

sdk::Camera::properties CameraRealSense::get_properties() {
    auto fillResp = [](sdk::Camera::properties* p, CameraProperties props, bool supportsPCD) {
        p->supports_pcd = supportsPCD;
        p->intrinsic_parameters.width_px = props.width;
        p->intrinsic_parameters.height_px = props.height;
        p->intrinsic_parameters.focal_x_px = props.fx;
        p->intrinsic_parameters.focal_y_px = props.fy;
        p->intrinsic_parameters.center_x_px = props.ppx;
        p->intrinsic_parameters.center_y_px = props.ppy;
        p->distortion_parameters.model = props.distortionModel;
        for (int i = 0; i < std::size(props.distortionParameters); i++) {
            p->distortion_parameters.parameters.push_back(props.distortionParameters[i]);
        }
    };

    sdk::Camera::properties response{};
    // pcd enabling will be a config parameter, for now, just put false
    bool pcdEnabled = false;
    if (this->props_.mainSensor.compare("color") == 0) {
        fillResp(&response, this->props_.color, pcdEnabled);
    } else if (props_.mainSensor.compare("depth") == 0) {
        fillResp(&response, this->props_.depth, pcdEnabled);
    }

    return response;
}

sdk::Camera::image_collection CameraRealSense::get_images() {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }
    sdk::Camera::image_collection response;

    rs2::frame latestColorFrame;
    std::shared_ptr<std::vector<uint16_t>> latestDepthFrame;
    std::chrono::milliseconds latestTimestamp;
    {
        std::lock_guard<std::mutex> lock(GLOBAL_LATEST_FRAMES.mutex);
        latestColorFrame = GLOBAL_LATEST_FRAMES.colorFrame;
        latestDepthFrame = GLOBAL_LATEST_FRAMES.depthFrame;
        latestTimestamp = GLOBAL_LATEST_FRAMES.timestamp;
    }

    for (const auto& sensor : this->props_.sensors) {
        if (sensor == "color") {
            std::unique_ptr<sdk::Camera::raw_image> color_response;
            color_response =
                encodeJPEGToResponse((const unsigned char*)latestColorFrame.get_data(),
                                     this->props_.color.width, this->props_.color.height);
            response.images.emplace_back(std::move(*color_response));
        } else if (sensor == "depth") {
            std::unique_ptr<sdk::Camera::raw_image> depth_response;
            depth_response = encodeDepthRAWToResponse(
                (const unsigned char*)latestDepthFrame->data(), this->props_.depth.width,
                this->props_.depth.height, this->props_.littleEndianDepth);
            response.images.emplace_back(std::move(*depth_response));
        }
    }
    response.metadata.captured_at = std::chrono::time_point<long long, std::chrono::nanoseconds>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp));
    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[get_images]  total:           " << duration.count() << "ms\n";
    }
    return response;
}

sdk::AttributeMap CameraRealSense::do_command(sdk::AttributeMap command) {
    std::cerr << "do_command not implemented" << std::endl;
    return sdk::AttributeMap{};
}

sdk::Camera::point_cloud CameraRealSense::get_point_cloud(std::string mime_type,
                                                          const sdk::AttributeMap& extra) {
    std::cerr << "get_point_cloud not implemented" << std::endl;
    return sdk::Camera::point_cloud{};
}
std::vector<sdk::GeometryConfig> CameraRealSense::get_geometries(const sdk::AttributeMap& extra) {
    std::cerr << "get_geometries not implemented" << std::endl;
    return std::vector<sdk::GeometryConfig>{};
}

// Loop functions
void frameLoop(rs2::pipeline pipeline, std::promise<void>& ready,
               std::shared_ptr<DeviceProperties> deviceProps, float depthScaleMm) {
    bool readyOnce = false;
    {
        std::lock_guard<std::mutex> lock(deviceProps->mutex);
        deviceProps->shouldRun = true;
        deviceProps->isRunning = true;
    }
    // start the callback function that will look for camera disconnects and reconnects.
    // on reconnects, it will close and restart the pipeline and thread.
    rs2::context ctx;
    ctx.set_devices_changed_callback(
        [&](rs2::event_information& info) { on_device_reconnect(info, pipeline, deviceProps); });
    std::cout << "[frameLoop] frame loop is starting" << std::endl;
    while (true) {
        {
            std::lock_guard<std::mutex> lock(deviceProps->mutex);
            if (!deviceProps->shouldRun) {
                ctx = rs2::context();  // deregisters callback
                pipeline.stop();
                std::cout << "[frameLoop] pipeline stopped, exiting frame loop" << std::endl;
                break;
            }
        }
        auto failureWait = std::chrono::milliseconds(5);

        std::chrono::time_point<std::chrono::high_resolution_clock> start;
        if (debug_enabled) {
            start = std::chrono::high_resolution_clock::now();
        }

        rs2::frameset frames;
        const uint timeoutMillis = 2000;
        /*
            D435 1920x1080 RGB + Depth ~20ms on a Raspberry Pi 4 Model B
        */
        bool succ = pipeline.try_wait_for_frames(&frames, timeoutMillis);
        if (!succ) {
            if (debug_enabled) {
                std::cerr << "[frameLoop] could not get frames from realsense after "
                          << timeoutMillis << "ms" << std::endl;
            }
            std::this_thread::sleep_for(failureWait);
            continue;
        }
        if (debug_enabled) {
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            std::cout << "[frameLoop] wait for frames: " << duration.count() << "ms\n";
        }

        if (!deviceProps->disableColor && !deviceProps->disableDepth) {
            std::chrono::time_point<std::chrono::high_resolution_clock> start;
            if (debug_enabled) {
                start = std::chrono::high_resolution_clock::now();
            }

            try {
                frames = FRAME_ALIGNMENT.process(frames);
            } catch (const std::exception& e) {
                std::cerr << "[frameLoop] exception while aligning images: " << e.what()
                          << std::endl;
                std::this_thread::sleep_for(failureWait);
                continue;
            }

            if (debug_enabled) {
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                std::cout << "[frameLoop] frame alignment: " << duration.count() << "ms\n";
            }
        }
        // scale every pixel value to be depth in units of mm
        std::unique_ptr<std::vector<uint16_t>> depthFrameScaled;
        if (!deviceProps->disableDepth) {
            auto depthFrame = frames.get_depth_frame();
            auto depthWidth = depthFrame.get_width();
            auto depthHeight = depthFrame.get_height();
            const uint16_t* depthFrameData = (const uint16_t*)depthFrame.get_data();
            depthFrameScaled = std::make_unique<std::vector<uint16_t>>(depthWidth * depthHeight);
            for (int y = 0; y < depthHeight; y++) {
                for (int x = 0; x < depthWidth; x++) {
                    auto px = (y * depthWidth) + x;
                    uint16_t depthScaled = depthScaleMm * depthFrameData[px];
                    (*depthFrameScaled)[px] = depthScaled;
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock(GLOBAL_LATEST_FRAMES.mutex);
            GLOBAL_LATEST_FRAMES.colorFrame = frames.get_color_frame();
            GLOBAL_LATEST_FRAMES.depthFrame = std::move(depthFrameScaled);
            GLOBAL_LATEST_FRAMES.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double, std::milli>(frames.get_timestamp()));
        }

        if (debug_enabled) {
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            std::cout << "[frameLoop] total:           " << duration.count() << "ms\n";
        }

        if (!readyOnce) {
            readyOnce = true;
            ready.set_value();
        }
    }
    {
        std::lock_guard<std::mutex> lock(deviceProps->mutex);
        deviceProps->isRunning = false;
    }
    deviceProps->cv.notify_all();
};

// gives the pixel to mm conversion for the depth sensor
float getDepthScale(rs2::device dev) {
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors()) {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            return dpt.get_depth_scale() * 1000.0;  // rs2 gives pix2meters
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

std::tuple<rs2::pipeline, RealSenseProperties> startPipeline(bool disableDepth, int depthWidth,
                                                             int depthHeight, bool disableColor,
                                                             int colorWidth, int colorHeight) {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw std::runtime_error("no device connected; please connect an Intel RealSense device");
    }
    rs2::device selected_device = devices.front();

    auto serial = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "found device:\n";
    std::cout << "name:      " << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\n";
    std::cout << "serial:    " << serial << "\n";
    std::cout << "firmware:  " << selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
              << "\n";
    std::cout << "port:      " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << "\n";
    std::cout << "usb type:  " << selected_device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
              << "\n";

    float depthScaleMm = 0.0;
    if (!disableDepth) {
        depthScaleMm = getDepthScale(selected_device);
    }

    rs2::config cfg;
    cfg.enable_device(serial);

    if (!disableColor) {
        std::cout << "color width and height from config: (" << colorWidth << ", " << colorHeight
                  << ")\n";
        cfg.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight, RS2_FORMAT_RGB8);
    }

    if (!disableDepth) {
        std::cout << "depth width and height from config: (" << depthWidth << ", " << depthHeight
                  << ")\n";
        cfg.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16);
    }

    rs2::pipeline pipeline(ctx);
    pipeline.start(cfg);

    auto fillProps = [](auto intrinsics, std::string distortionModel) -> CameraProperties {
        CameraProperties camProps;
        camProps.width = intrinsics.width;
        camProps.height = intrinsics.height;
        camProps.fx = intrinsics.fx;
        camProps.fy = intrinsics.fy;
        camProps.ppx = intrinsics.ppx;
        camProps.ppy = intrinsics.ppy;
        if (distortionModel != "") {
            camProps.distortionModel = std::move(distortionModel);
            int distortionSize = std::min((int)std::size(intrinsics.coeffs), 5);
            for (int i = 0; i < distortionSize; i++) {
                camProps.distortionParameters[i] = double(intrinsics.coeffs[i]);
            }
        }
        return camProps;
    };

    RealSenseProperties props;
    props.depthScaleMm = depthScaleMm;
    if (!disableColor) {
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.color = fillProps(intrinsics, "brown_conrady");
    }
    if (!disableDepth) {
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_DEPTH)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.depth = fillProps(intrinsics, "");
        if (!disableColor) {
            props.depth.width = props.color.width;
            props.depth.height = props.color.height;
        }
    }

    std::cout << "pipeline started with:\n";
    std::cout << "color_enabled:  " << std::boolalpha << !disableColor << "\n";
    if (!disableColor) {
        std::cout << "color_width:    " << props.color.width << "\n";
        std::cout << "color_height:   " << props.color.height << "\n";
    }
    std::cout << "depth_enabled:  " << !disableDepth << std::endl;
    if (!disableDepth) {
        auto alignedText = "";
        if (!disableColor) {
            alignedText = " (aligned to color)";
        }
        std::cout << "depth_width:    " << props.depth.width << alignedText << "\n";
        std::cout << "depth_height:   " << props.depth.height << alignedText << std::endl;
    }

    return std::make_tuple(pipeline, props);
};

void on_device_reconnect(rs2::event_information& info, rs2::pipeline pipeline,
                         std::shared_ptr<DeviceProperties> device) {
    if (device == nullptr) {
        throw std::runtime_error(
            "no device info to reconnect to. RealSense device was never initialized.");
    }
    if (info.was_added(info.get_new_devices().front())) {
        std::cout << "Device was reconnected, restarting pipeline" << std::endl;
        {
            // wait until frameLoop is stopped
            std::unique_lock<std::mutex> lock(device->mutex);
            device->shouldRun = false;
            device->cv.wait(lock, [device] { return !(device->isRunning); });
        }
        // Find and start the first available device
        RealSenseProperties props;
        try {
            std::tie(pipeline, props) =
                startPipeline(device->disableDepth, device->depthWidth, device->depthHeight,
                              device->disableColor, device->colorWidth, device->colorHeight);
        } catch (const std::exception& e) {
            std::cout << "caught exception: \"" << e.what() << "\"" << std::endl;
            return;
        }
        // Start the camera std::thread
        std::promise<void> ready;
        std::thread cameraThread(frameLoop, pipeline, ref(ready), device, props.depthScaleMm);
        std::cout << "waiting for camera frame loop thread to be ready..." << std::endl;
        ready.get_future().wait();
        std::cout << "camera frame loop ready!" << std::endl;
        cameraThread.detach();
    } else {
        std::cout << "Device disconnected, stopping frame pipeline" << std::endl;
        {
            std::lock_guard<std::mutex> lock(device->mutex);
            device->shouldRun = false;
        }
    }
};

// validate will validate the ResourceConfig. If there is an error, it will throw an exception.
std::vector<std::string> validate(sdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();
    if (attrs->count("width_px") == 1) {
        std::shared_ptr<sdk::ProtoType> width_proto = attrs->at("width_px");
        auto width_value = width_proto->proto_value();
        if (width_value.has_number_value()) {
            int width_num = static_cast<int>(width_value.number_value());
            if (width_num < 0) {
                throw std::invalid_argument("width_px cannot be negative");
            }
        }
    }
    if (attrs->count("height_px") == 1) {
        std::shared_ptr<sdk::ProtoType> height_proto = attrs->at("height_px");
        auto height_value = height_proto->proto_value();
        if (height_value.has_number_value()) {
            int height_num = static_cast<int>(height_value.number_value());
            if (height_num < 0) {
                throw std::invalid_argument("height_px cannot be negative");
            }
        }
    }
    if (attrs->count("sensors") >= 1) {
        std::shared_ptr<sdk::ProtoType> sensors_proto = attrs->at("sensors");
        auto sensors_value = sensors_proto->proto_value();
        if (sensors_value.has_list_value()) {
            auto sensors_list = sensors_value.list_value();
            if (sensors_list.values().size() == 0) {
                throw std::invalid_argument(
                    "sensors field cannot be empty, must list color and/or depth sensor");
            }
        }
    } else {
        throw std::invalid_argument("could not find required 'sensors' attribute in the config");
    }
    return {};
}

int serve(int argc, char** argv) {
    std::shared_ptr<sdk::ModelRegistration> mr = std::make_shared<sdk::ModelRegistration>(
        sdk::API::get<sdk::Camera>(),
        sdk::Model{kAPINamespace, kAPIType, kAPISubtype},
        [](sdk::Dependencies deps, sdk::ResourceConfig cfg) -> std::shared_ptr<sdk::Resource> {
            return std::make_unique<CameraRealSense>(deps, cfg);
        },
        validate
    );

    std::vector<std::shared_ptr<sdk::ModelRegistration>> mrs = {mr};
    auto module_service = std::make_shared<sdk::ModuleService>(argc, argv, mrs);
    module_service->serve();

    return EXIT_SUCCESS;
}

}  // namespace realsense
}  // namespace viam
