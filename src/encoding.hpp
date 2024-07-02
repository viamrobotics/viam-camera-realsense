#ifndef ENCODING_HPP
#define ENCODING_HPP

#include "camera_realsense.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <turbojpeg.h>
#include <vector>

#include "third_party/fpng.h"
#include "third_party/lodepng.h"

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

    viam::sdk::Camera::depth_map m = xt::xarray<uint16_t>::from_shape({height, width});
    std::copy(reinterpret_cast<const uint16_t*>(data),
              reinterpret_cast<const uint16_t*>(data) + height * width, m.begin());

    std::vector<unsigned char> encodedData = viam::sdk::Camera::encode_depth_map(m);

    unsigned char* rawBuf = new unsigned char[encodedData.size()];
    std::memcpy(rawBuf, encodedData.data(), encodedData.size());

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  RAW depth encode:      " << duration.count() << "ms\n";
    }

    return raw_camera_image{raw_camera_image::uniq(rawBuf, raw_camera_image::array_delete_deleter),
                            encodedData.size()};
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

std::vector<unsigned char> rsPointsToPCDBytes(const rs2::points& points, const rs2::frame& colorFrame) {
    std::stringstream header;
    header << "VERSION .7\n";

    bool hasColor = points.get_texture_coordinates() != nullptr && colorFrame;
    if (hasColor) {
        header << "FIELDS x y z rgb\n";
        header << "SIZE 4 4 4 4\n";
        header << "TYPE F F F U\n";
        header << "COUNT 1 1 1 1\n";
    } else {
        header << "FIELDS x y z\n";
        header << "SIZE 4 4 4\n";
        header << "TYPE F F F\n";
        header << "COUNT 1 1 1\n";
    }

    size_t fieldsCount = hasColor ? 4 : 3;
    size_t pointSize = sizeof(float) * fieldsCount;

    const rs2::vertex* vertices = points.get_vertices();
    const rs2::texture_coordinate* texCoords = hasColor ? points.get_texture_coordinates() : nullptr;
    size_t vertexCount = points.size();
    std::vector<unsigned char> dataBytes;
    dataBytes.reserve(vertexCount * pointSize);

    const uint8_t* colorData = nullptr;
    int width = 0, height = 0;

    if (hasColor && colorFrame) {
        colorData = static_cast<const uint8_t*>(colorFrame.get_data());
        if (colorFrame.is<rs2::video_frame>()) {
            auto video_frame = colorFrame.as<rs2::video_frame>();
            width = video_frame.get_width();
            height = video_frame.get_height();
            if (width <= 0 || height <= 0) {
                throw std::runtime_error("Error processing point cloud: color frame dimensions must be positive non-zero values.");
            }
        } else {
            throw std::runtime_error("Error processing point cloud: provided frame is not a video frame.");
        }
    }

    for (size_t i = 0; i < vertexCount; ++i) {
        const unsigned char* vertexBytes = reinterpret_cast<const unsigned char*>(&vertices[i]);
        dataBytes.insert(dataBytes.end(), vertexBytes, vertexBytes + sizeof(rs2::vertex));
        if (hasColor && colorData) {
            uint32_t rgb = 0;
            if (texCoords[i].u != 0 && texCoords[i].v != 0) {
                int u = std::min(std::max(int(texCoords[i].u * width), 0), width - 1);
                int v = std::min(std::max(int(texCoords[i].v * height), 0), height - 1);
                int color_index = (v * width + u) * 3;

                // librealsense uses BGR order
                rgb = ((uint32_t)colorData[color_index] << 16) | 
                      ((uint32_t)colorData[color_index + 1] << 8) | 
                      ((uint32_t)colorData[color_index + 2]);
            }
            const unsigned char* colorBytes = reinterpret_cast<const unsigned char*>(&rgb);
            dataBytes.insert(dataBytes.end(), colorBytes, colorBytes + sizeof(uint32_t));
        }
    }

    // Post-process subsampling if the data size exceeds 4MB (gRPC message size limit)
    // TODO RSDK-7976: Change string to new gRPC size limit or remove totally
    if (dataBytes.size() > 4194304) {
        std::vector<unsigned char> subsampledData;
        int subsampleRatio = dataBytes.size() / 4194304 + 1;
        for (size_t i = 0; i < dataBytes.size(); i += subsampleRatio * pointSize) {
            subsampledData.insert(subsampledData.end(), dataBytes.begin() + i, dataBytes.begin() + i + pointSize);
        }
        dataBytes = std::move(subsampledData);
    }

    size_t numPoints = dataBytes.size() / pointSize;
    header << "WIDTH " << numPoints << "\n";
    header << "HEIGHT 1\n";
    header << "VIEWPOINT 0 0 0 1 0 0 0\n";
    header << "POINTS " << numPoints << "\n";
    header << "DATA binary\n";

    std::vector<unsigned char> pcdBytes;
    std::string headerStr = header.str();
    pcdBytes.insert(pcdBytes.end(), headerStr.begin(), headerStr.end());
    pcdBytes.insert(pcdBytes.end(), dataBytes.begin(), dataBytes.end());

    return pcdBytes;
}
}  // namespace

#endif // ENCODING_HPP
