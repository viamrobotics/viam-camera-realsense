// End to end tests: the production Realsense resource driven with a
// librealsense software device in place of the camera. Everything below the
// resource is real: createDevice/startDevice/stopDevice/destroyDevice, an
// rs2::pipeline, frameCallback, rs2::align, rs2::pointcloud and the JPEG,
// depth map and PCD encoders. The one substitution is the pipeline's
// context: the test's instead of a private one, because that is the only
// context that knows the software device. The context is software-only, so
// no USB is touched and this runs wherever the unit tests do.
//
// librealsense 2.57.7 quirks handled here:
//   * software_device::add_to leaves the device unowned; the context keeps
//     only a weak_ptr. The synchronous devices-changed callback hands out an
//     owning rs2::device_list, and keeping that alive is what makes
//     query_devices() and the pipeline see the device.
//   * A software sensor is an rs2::depth_sensor only once it has an
//     RS2_OPTION_DEPTH_UNITS option, and is never an rs2::color_sensor.
//   * Software frames report get_data_size() == 0 (caller-owned pixels), so
//     get_point_cloud only null-checks frame data.
//
// Stop feeding frames before destroying a resource: the software sensor does
// not serialize on_video_frame against stop().
//
// On macOS each case takes about 2 s longer than its assertions: every
// rs2::software_device owns a private default context, and the last such
// context in a process stops librealsense's USB polling watcher on the way
// out, which waits out its 2 s interval. Linux tears down quickly. ctest runs
// each case as its own process, so run the label with ctest -j to overlap it.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "realsense.hpp"
#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>

#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/rs.hpp>
#include <turbojpeg.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace realsense;
using namespace viam::sdk;
using ::testing::HasSubstr;

using SyncContext = boost::synchronized_value<rs2::context>;
using ModuleContext = RealsenseContext<SyncContext>;
using SwRealsense = Realsense<SyncContext>;

constexpr int kWidth = 320;
constexpr int kHeight = 240;
constexpr int kFps = 30;
constexpr float kDepthUnits = 0.001f; // 1 mm, the D435 default
constexpr float kFocal = 300.f;
constexpr std::uint16_t kUniformDepthMm = 500;

// Context settings: software devices only (no USB backend, no hotplug thread)
// and do not inherit the machine's realsense-config.json.
std::string contextSettings() {
  return std::string("{\"device-mask\": ") +
         std::to_string(RS2_PRODUCT_LINE_SW_ONLY) +
         ", \"dds\": false, \"inherit\": false}";
}

rs2_intrinsics pinholeIntrinsics() {
  rs2_intrinsics i{};
  i.width = kWidth;
  i.height = kHeight;
  i.ppx = kWidth / 2.f;
  i.ppy = kHeight / 2.f;
  i.fx = kFocal;
  i.fy = kFocal;
  i.model = RS2_DISTORTION_BROWN_CONRADY;
  return i; // zero coefficients: projection is a plain pinhole
}

rs2_extrinsics identityExtrinsics() {
  return {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};
}

enum class DepthPattern { ramp, uniform };

struct SoftwareCameraSpec {
  std::string serial{"SW0000001"};
  std::string name{"Intel RealSense D435"};
  std::string usb_descriptor{"3.2"};
  DepthPattern depth_pattern{DepthPattern::ramp};
  std::uint16_t depth_base_mm{400};
  rs2_intrinsics depth_intrinsics{pinholeIntrinsics()};
  rs2_intrinsics color_intrinsics{pinholeIntrinsics()};
  rs2_extrinsics depth_to_color{identityExtrinsics()};
};

// A software D4xx: two sensors, one Z16 depth stream and one RGB8 color
// stream at the same resolution and frame rate, plus the depth options the
// module reads and writes. Frames are injected from a feeder thread with
// wall clock timestamps so the module's staleness checks behave as they do
// with a real camera.
class SoftwareCamera {
public:
  explicit SoftwareCamera(SoftwareCameraSpec spec)
      : spec_(std::move(spec)), depth_sensor_(dev_.add_sensor("Stereo Module")),
        color_sensor_(dev_.add_sensor("RGB Camera")), depth_(kWidth * kHeight),
        color_(kWidth * kHeight * 3) {
    dev_.register_info(RS2_CAMERA_INFO_NAME, spec_.name);
    dev_.register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, spec_.serial);
    dev_.register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, "5.16.0.1");
    dev_.register_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION,
                       "5.16.0.1");
    dev_.register_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR,
                       spec_.usb_descriptor);
    dev_.register_info(RS2_CAMERA_INFO_PRODUCT_LINE, "D400");

    depth_profile_ = depth_sensor_.add_video_stream(
        {RS2_STREAM_DEPTH, 0, 0, kWidth, kHeight, kFps, 2, RS2_FORMAT_Z16,
         spec_.depth_intrinsics});
    color_profile_ = color_sensor_.add_video_stream(
        {RS2_STREAM_COLOR, 0, 1, kWidth, kHeight, kFps, 3, RS2_FORMAT_RGB8,
         spec_.color_intrinsics});

    // DEPTH_UNITS is also what makes the sensor cast to rs2::depth_sensor.
    depth_sensor_.add_read_only_option(RS2_OPTION_DEPTH_UNITS, kDepthUnits);
    addOption(RS2_OPTION_LASER_POWER, 0, 360, 150, 30);
    addOption(RS2_OPTION_EMITTER_ENABLED, 0, 1, 1, 1);
    addOption(RS2_OPTION_VISUAL_PRESET, 0, RS2_RS400_VISUAL_PRESET_COUNT - 1,
              RS2_RS400_VISUAL_PRESET_DEFAULT, 1);
    addOption(RS2_OPTION_EXPOSURE, 1, 200000, 8500, 1);
    addOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0, 1, 1, 1);
    addOption(RS2_OPTION_GAIN, 16, 248, 16, 1);

    depth_profile_.register_extrinsics_to(color_profile_, spec_.depth_to_color);
    dev_.create_matcher(RS2_MATCHER_DEFAULT);

    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        depth_[y * kWidth + x] = depthAt(x, y);
        auto rgb = colorAt(x, y);
        color_[(y * kWidth + x) * 3 + 0] = rgb[0];
        color_[(y * kWidth + x) * 3 + 1] = rgb[1];
        color_[(y * kWidth + x) * 3 + 2] = rgb[2];
      }
    }
  }

  ~SoftwareCamera() { stopFeeding(); }
  SoftwareCamera(SoftwareCamera const &) = delete;
  SoftwareCamera &operator=(SoftwareCamera const &) = delete;

  // Registers the device with the context. The devices-changed callback runs
  // inline inside add_to and is the only place an owning handle to the new
  // device_info is available; keep it for the camera's lifetime.
  void addTo(rs2::context &ctx) {
    ctx.set_devices_changed_callback([this](rs2::event_information &info) {
      keepalive_.push_back(info.get_new_devices());
    });
    dev_.add_to(ctx);
    ASSERT_EQ(keepalive_.size(), 1u) << "add_to did not fire the callback";
  }

  void startFeeding() {
    if (feeder_.joinable()) {
      return;
    }
    feeding_ = true;
    feeder_ = std::thread([this] {
      int frame_number = 0;
      while (feeding_) {
        double const ts = time::getNowMs();
        depth_sensor_.on_video_frame({depth_.data(), [](void *) {}, kWidth * 2,
                                      2, ts, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
                                      frame_number, depth_profile_.get(),
                                      kDepthUnits});
        color_sensor_.on_video_frame({color_.data(), [](void *) {}, kWidth * 3,
                                      3, ts, RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
                                      frame_number, color_profile_.get(), 0.f});
        ++frame_number;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kFps));
      }
    });
  }

  void stopFeeding() {
    feeding_ = false;
    if (feeder_.joinable()) {
      feeder_.join();
    }
  }

  std::uint16_t depthAt(int x, int y) const {
    if (spec_.depth_pattern == DepthPattern::uniform) {
      return kUniformDepthMm;
    }
    return static_cast<std::uint16_t>(spec_.depth_base_mm + x + 2 * y);
  }

  static std::array<std::uint8_t, 3> colorAt(int x, int y) {
    return {static_cast<std::uint8_t>(x & 0xff),
            static_cast<std::uint8_t>(y & 0xff), 128};
  }

  SoftwareCameraSpec const &spec() const { return spec_; }

private:
  void addOption(rs2_option opt, float min, float max, float def, float step) {
    rs2::option_range range{};
    range.min = min;
    range.max = max;
    range.def = def;
    range.step = step;
    depth_sensor_.add_option(opt, range);
  }

  SoftwareCameraSpec spec_;
  rs2::software_device dev_; // declared before the sensors it creates
  rs2::software_sensor depth_sensor_;
  rs2::software_sensor color_sensor_;
  rs2::stream_profile depth_profile_;
  rs2::stream_profile color_profile_;
  std::vector<std::uint16_t> depth_;
  std::vector<std::uint8_t> color_;
  std::vector<rs2::device_list> keepalive_;
  std::atomic<bool> feeding_{false};
  std::thread feeder_;
};

// ---- response decoders ----

struct DepthMap {
  std::uint64_t width{};
  std::uint64_t height{};
  std::vector<std::uint8_t> bytes; // owned: callers often pass temporaries

  static std::uint64_t be64(std::uint8_t const *p) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
      v = (v << 8) | p[i];
    }
    return v;
  }

  static DepthMap parse(std::vector<std::uint8_t> const &bytes) {
    EXPECT_GE(bytes.size(), 24u);
    EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 8), "DEPTHMAP");
    DepthMap m;
    m.width = be64(bytes.data() + 8);
    m.height = be64(bytes.data() + 16);
    m.bytes = bytes;
    EXPECT_EQ(bytes.size(), 24 + m.width * m.height * 2);
    return m;
  }

  std::uint16_t at(std::uint64_t x, std::uint64_t y) const {
    auto const *p = bytes.data() + 24 + (y * width + x) * 2;
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
  }
};

struct JpegHeader {
  int width{};
  int height{};

  static JpegHeader parse(std::vector<std::uint8_t> const &bytes) {
    JpegHeader h;
    int subsamp = 0, colorspace = 0;
    tjhandle tj = tjInitDecompress();
    EXPECT_NE(tj, nullptr);
    EXPECT_EQ(tjDecompressHeader3(tj, bytes.data(), bytes.size(), &h.width,
                                  &h.height, &subsamp, &colorspace),
              0)
        << tjGetErrorStr();
    tjDestroy(tj);
    return h;
  }
};

struct PcdPoint {
  float x, y, z;
  std::uint32_t rgb;
};

struct Pcd {
  std::string header;
  std::size_t points{};
  std::vector<std::uint8_t> bytes; // owned

  static Pcd parse(std::vector<std::uint8_t> const &bytes) {
    Pcd pcd;
    std::string const marker = "DATA binary\n";
    std::string head(bytes.begin(),
                     bytes.begin() + std::min<std::size_t>(512, bytes.size()));
    auto data_pos = head.find(marker);
    EXPECT_NE(data_pos, std::string::npos) << "no binary PCD data section";
    pcd.header = head.substr(0, data_pos + marker.size());
    auto points_pos = pcd.header.find("POINTS ");
    EXPECT_NE(points_pos, std::string::npos);
    pcd.points = std::stoul(pcd.header.substr(points_pos + 7));
    pcd.bytes = bytes;
    EXPECT_EQ(bytes.size(), pcd.header.size() + pcd.points * sizeof(PcdPoint));
    return pcd;
  }

  PcdPoint at(std::size_t i) const {
    PcdPoint p;
    std::memcpy(&p, bytes.data() + header.size() + i * sizeof(PcdPoint),
                sizeof(PcdPoint));
    return p;
  }
};

viam::sdk::Camera::raw_image const &
findImage(viam::sdk::Camera::image_collection const &images,
          std::string const &source) {
  for (auto const &img : images.images) {
    if (img.source_name == source) {
      return img;
    }
  }
  throw std::runtime_error("no image with source " + source);
}

// ---- fixture ----

class SoftwareDeviceTest : public ::testing::Test {
protected:
  void SetUp() override {
    ctx_ =
        std::make_shared<SyncContext>(rs2::context(contextSettings().c_str()));
    assigned_serials_ = std::make_shared<
        boost::synchronized_value<std::unordered_set<std::string>>>();
  }

  void TearDown() override {
    for (auto &cam : cameras_) {
      cam->stopFeeding();
    }
    resources_.clear();
    module_ctx_.reset();
    cameras_.clear();
  }

  // Add every camera before the first makeResource(): constructing the
  // module context installs the module's device-change callback.
  SoftwareCamera &addCamera(SoftwareCameraSpec spec = {}) {
    EXPECT_EQ(module_ctx_, nullptr) << "add cameras before resources";
    cameras_.push_back(std::make_unique<SoftwareCamera>(std::move(spec)));
    auto ctx = ctx_->synchronize();
    cameras_.back()->addTo(*ctx);
    return *cameras_.back();
  }

  std::shared_ptr<ModuleContext> moduleContext() {
    if (not module_ctx_) {
      module_ctx_ = std::make_shared<ModuleContext>(ctx_);
    }
    return module_ctx_;
  }

  static ResourceConfig makeConfig(ProtoStruct attributes,
                                   std::string const &name = "swcam") {
    return ResourceConfig("rdk:component:camera", "", name, attributes, "",
                          Model("viam", "camera", "realsense"), LinkConfig{},
                          log_level::info);
  }

  static ProtoStruct bothSensors(std::string const &serial) {
    ProtoStruct attrs;
    attrs["serial_number"] = serial;
    attrs["sensors"] = ProtoList{"color", "depth"};
    return attrs;
  }

  // Production DeviceFunctions, except that after createDevice the pipeline
  // is swapped for one on the test context: librealsense only lists a
  // software device in the context it was added to, and a default
  // rs2::pipeline lives on a private one.
  SwRealsense &makeResource(ResourceConfig const &cfg) {
    auto funcs = SwRealsense::createDefaultDeviceFunctions();
    auto create = funcs.createDevice;
    funcs.createDevice = [create, ctx = ctx_](auto const &serial, auto dev_ptr,
                                              auto const &supported_models,
                                              auto const &config,
                                              auto &logger) {
      auto device = create(serial, dev_ptr, supported_models, config, logger);
      if (device) {
        auto guard = device->synchronize();
        guard->pipe = std::make_shared<rs2::pipeline>(*ctx->synchronize());
      }
      return device;
    };
    resources_.push_back(std::make_unique<SwRealsense>(
        Dependencies{}, cfg, moduleContext(), funcs, assigned_serials_));
    return *resources_.back();
  }

  void destroyResources() { resources_.clear(); }

  // Streams start asynchronously; poll until the first frameset lands.
  static bool waitForFrames(
      SwRealsense &cam,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    auto const deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        cam.get_images({}, {});
        return true;
      } catch (std::exception const &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    return false;
  }

  // One camera, one resource with both sensors, frames flowing.
  SwRealsense &streamingResource(SoftwareCameraSpec spec = {},
                                 ProtoStruct extra_attrs = {}) {
    auto &cam = addCamera(spec);
    auto attrs = bothSensors(cam.spec().serial);
    for (auto const &kv : extra_attrs) {
      attrs[kv.first] = kv.second;
    }
    auto &rs = makeResource(makeConfig(attrs));
    cam.startFeeding();
    EXPECT_TRUE(waitForFrames(rs)) << "no frameset within timeout";
    return rs;
  }

  std::shared_ptr<SyncContext> ctx_;
  std::shared_ptr<ModuleContext> module_ctx_;
  std::shared_ptr<boost::synchronized_value<std::unordered_set<std::string>>>
      assigned_serials_;
  std::vector<std::unique_ptr<SoftwareCamera>> cameras_;
  std::vector<std::unique_ptr<SwRealsense>> resources_;
};

// ---- get_images ----

TEST_F(SoftwareDeviceTest, GetImages_ReturnsColorJpegAndDepthMap) {
  auto &rs = streamingResource();
  auto const &cam = *cameras_.front();

  auto images = rs.get_images({}, {});
  ASSERT_EQ(images.images.size(), 2u);

  auto const &color = findImage(images, "color");
  EXPECT_EQ(color.mime_type, "image/jpeg");
  auto jpeg = JpegHeader::parse(color.bytes);
  EXPECT_EQ(jpeg.width, kWidth);
  EXPECT_EQ(jpeg.height, kHeight);

  auto const &depth = findImage(images, "depth");
  EXPECT_EQ(depth.mime_type, "image/vnd.viam.dep");
  auto map = DepthMap::parse(depth.bytes);
  ASSERT_EQ(map.width, static_cast<std::uint64_t>(kWidth));
  ASSERT_EQ(map.height, static_cast<std::uint64_t>(kHeight));
  for (auto [x, y] : {std::pair{0, 0}, std::pair{100, 50},
                      std::pair{kWidth - 1, kHeight - 1}}) {
    EXPECT_EQ(map.at(x, y), cam.depthAt(x, y)) << "pixel " << x << "," << y;
  }
}

TEST_F(SoftwareDeviceTest, GetImages_FilterSourceNamesSelectsOneStream) {
  auto &rs = streamingResource();

  auto depth_only = rs.get_images({"depth"}, {});
  ASSERT_EQ(depth_only.images.size(), 1u);
  EXPECT_EQ(depth_only.images[0].source_name, "depth");

  auto color_only = rs.get_images({"color"}, {});
  ASSERT_EQ(color_only.images.size(), 1u);
  EXPECT_EQ(color_only.images[0].source_name, "color");
}

TEST_F(SoftwareDeviceTest, GetImages_CapturedAtTracksWallClock) {
  auto &rs = streamingResource();
  auto images = rs.get_images({}, {});
  auto captured_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         images.metadata.captured_at.time_since_epoch())
                         .count();
  auto now_ms = static_cast<long long>(time::getNowMs());
  EXPECT_NEAR(static_cast<double>(now_ms - captured_ms), 0, 1000)
      << "captured_at should be within a second of now";
}

// A uniform depth plane at 0.5 m with the color sensor 20 mm to the +x of the
// depth sensor: every depth pixel lands 12 columns to the right in the color
// frame (fx * t / z = 300 * 0.02 / 0.5), so the leftmost color columns have
// no depth behind them once aligned and keep their value when not aligned.
TEST_F(SoftwareDeviceTest,
       GetImages_AlignColorDepthResamplesDepthIntoColorFrame) {
  SoftwareCameraSpec spec;
  spec.depth_pattern = DepthPattern::uniform;
  spec.depth_to_color.translation[0] = 0.02f;
  ProtoStruct align;
  align["align_color_depth"] = true;
  auto &rs = streamingResource(spec, align);

  auto map = DepthMap::parse(findImage(rs.get_images({}, {}), "depth").bytes);
  ASSERT_EQ(map.width, static_cast<std::uint64_t>(kWidth));
  ASSERT_EQ(map.height, static_cast<std::uint64_t>(kHeight));
  EXPECT_EQ(map.at(2, kHeight / 2), 0) << "no depth source left of the shift";
  EXPECT_EQ(map.at(kWidth / 2, kHeight / 2), kUniformDepthMm);
}

TEST_F(SoftwareDeviceTest, GetImages_WithoutAlignReturnsRawDepth) {
  SoftwareCameraSpec spec;
  spec.depth_pattern = DepthPattern::uniform;
  spec.depth_to_color.translation[0] = 0.02f;
  auto &rs = streamingResource(spec);

  auto map = DepthMap::parse(findImage(rs.get_images({}, {}), "depth").bytes);
  EXPECT_EQ(map.at(2, kHeight / 2), kUniformDepthMm);
  EXPECT_EQ(map.at(kWidth / 2, kHeight / 2), kUniformDepthMm);
}

// ---- get_point_cloud ----

TEST_F(SoftwareDeviceTest, GetPointCloud_ReturnsPcdMatchingInjectedGeometry) {
  auto &rs = streamingResource();
  auto const &cam = *cameras_.front();
  auto const intr = cam.spec().color_intrinsics;

  auto cloud = rs.get_point_cloud("pointcloud/pcd", {});
  EXPECT_EQ(cloud.mime_type, "pointcloud/pcd");
  auto pcd = Pcd::parse(cloud.pc);
  EXPECT_THAT(pcd.header, HasSubstr("FIELDS x y z rgb\n"));
  EXPECT_THAT(pcd.header, HasSubstr("TYPE F F F U\n"));
  ASSERT_EQ(pcd.points, static_cast<std::size_t>(kWidth * kHeight));
  EXPECT_LT(cloud.pc.size(), MAX_GRPC_MESSAGE_SIZE);

  // Identity extrinsics and shared intrinsics: aligning depth to color is a
  // no-op, so point (u, v) is the pinhole deprojection of the injected depth
  // and carries the injected color.
  for (auto [u, v] : {std::pair{0, 0}, std::pair{kWidth / 2, kHeight / 2},
                      std::pair{37, 101}, std::pair{kWidth - 1, kHeight - 1}}) {
    auto p = pcd.at(static_cast<std::size_t>(v) * kWidth + u);
    float const z = cam.depthAt(u, v) * kDepthUnits;
    EXPECT_NEAR(p.z, z, 1e-5) << "pixel " << u << "," << v;
    EXPECT_NEAR(p.x, (u - intr.ppx) / intr.fx * z, 1e-4) << "pixel " << u;
    EXPECT_NEAR(p.y, (v - intr.ppy) / intr.fy * z, 1e-4) << "pixel " << v;
    auto rgb = SoftwareCamera::colorAt(u, v);
    std::uint32_t expected =
        (std::uint32_t{rgb[0]} << 16) | (std::uint32_t{rgb[1]} << 8) | rgb[2];
    EXPECT_EQ(p.rgb, expected) << "pixel " << u << "," << v;
  }
}

TEST_F(SoftwareDeviceTest, GetPointCloud_StaleFramesThrowUsbHint) {
  auto &rs = streamingResource();
  cameras_.front()->stopFeeding();
  std::this_thread::sleep_for(
      std::chrono::milliseconds(MAX_FRAME_AGE_MS + 300));

  try {
    rs.get_point_cloud("pointcloud/pcd", {});
    FAIL() << "expected a staleness error";
  } catch (std::runtime_error const &e) {
    EXPECT_THAT(e.what(), HasSubstr("no recent"));
    EXPECT_THAT(e.what(), HasSubstr("check USB connection"));
  }
  // get_images has no staleness gate: the last frameset is still served.
  EXPECT_EQ(rs.get_images({}, {}).images.size(), 2u);
}

// ---- get_properties / get_status / get_geometries ----

TEST_F(SoftwareDeviceTest,
       GetProperties_ReportsColorIntrinsicsAndReordersDistortion) {
  SoftwareCameraSpec spec;
  spec.color_intrinsics.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY;
  float const coeffs[5] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f}; // k1 k2 p1 p2 k3
  std::copy(coeffs, coeffs + 5, spec.color_intrinsics.coeffs);
  auto &rs = streamingResource(spec);

  auto props = rs.get_properties();
  EXPECT_TRUE(props.supports_pcd);
  EXPECT_EQ(props.intrinsic_parameters.width_px, kWidth);
  EXPECT_EQ(props.intrinsic_parameters.height_px, kHeight);
  EXPECT_FLOAT_EQ(props.intrinsic_parameters.focal_x_px, kFocal);
  EXPECT_FLOAT_EQ(props.intrinsic_parameters.focal_y_px, kFocal);
  EXPECT_FLOAT_EQ(props.intrinsic_parameters.center_x_px, kWidth / 2.f);
  EXPECT_FLOAT_EQ(props.intrinsic_parameters.center_y_px, kHeight / 2.f);
  EXPECT_EQ(props.distortion_parameters.model, "inverse_brown_conrady");
  // rdk order: k1 k2 k3 p1 p2
  EXPECT_THAT(props.distortion_parameters.parameters,
              ::testing::ElementsAre(::testing::DoubleNear(0.1, 1e-6),
                                     ::testing::DoubleNear(0.2, 1e-6),
                                     ::testing::DoubleNear(0.5, 1e-6),
                                     ::testing::DoubleNear(0.3, 1e-6),
                                     ::testing::DoubleNear(0.4, 1e-6)));
  // The cloud is emitted in the color frame the intrinsics describe.
  EXPECT_DOUBLE_EQ(props.extrinsic_parameters.translation.x(), 0.0);
  EXPECT_DOUBLE_EQ(props.extrinsic_parameters.translation.y(), 0.0);
  EXPECT_DOUBLE_EQ(props.extrinsic_parameters.translation.z(), 0.0);
  EXPECT_DOUBLE_EQ(props.extrinsic_parameters.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(props.extrinsic_parameters.orientation.theta, 0.0);
}

TEST_F(SoftwareDeviceTest, GetStatus_ReportsStreamingAndSerial) {
  auto &rs = streamingResource();
  auto status = rs.get_status();
  EXPECT_TRUE(*status["streaming"].get<bool>());
  EXPECT_TRUE(*status["physical_camera_assigned"].get<bool>());
  EXPECT_FALSE(*status["is_recovery_mode"].get<bool>());
  EXPECT_EQ(*status["serial_number"].get<std::string>(),
            cameras_.front()->spec().serial);
}

TEST_F(SoftwareDeviceTest, GetGeometries_FollowsCameraModel) {
  SoftwareCameraSpec d415;
  d415.name = "Intel RealSense D415";
  auto &rs = streamingResource(d415);
  EXPECT_EQ(rs.get_geometries({}),
            std::vector<GeometryConfig>{
                GeometryConfig(pose{35, 0, -8.9}, box({99, 23, 20}), "box")});
}

// ---- do_command depth knobs ----

TEST_F(SoftwareDeviceTest, DoCommand_DepthOptionsRoundTripOnTheSensor) {
  auto &rs = streamingResource();

  ProtoStruct set_laser;
  set_laser["set_laser_power"] = 90.0;
  auto r = rs.do_command(set_laser);
  ASSERT_TRUE(*r["success"].get<bool>()) << *r["error"].get<std::string>();
  EXPECT_EQ(*r["option"].get<std::string>(), "laser_power");

  ProtoStruct set_emitter;
  set_emitter["set_depth_emitter"] = false;
  r = rs.do_command(set_emitter);
  ASSERT_TRUE(*r["success"].get<bool>()) << *r["error"].get<std::string>();

  ProtoStruct set_preset;
  set_preset["set_depth_visual_preset"] = std::string("high_accuracy");
  r = rs.do_command(set_preset);
  ASSERT_TRUE(*r["success"].get<bool>()) << *r["error"].get<std::string>();

  ProtoStruct set_gain;
  set_gain["set_depth_gain"] = 32.0;
  r = rs.do_command(set_gain);
  ASSERT_TRUE(*r["success"].get<bool>()) << *r["error"].get<std::string>();

  ProtoStruct get;
  get["get_depth_options"] = true;
  auto opts = rs.do_command(get);
  EXPECT_TRUE(*opts["sensor_present"].get<bool>());
  EXPECT_DOUBLE_EQ(*opts["laser_power"].get<double>(), 90.0);
  EXPECT_DOUBLE_EQ(*opts["emitter_enabled"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(*opts["gain"].get<double>(), 32.0);
  EXPECT_EQ(*opts["visual_preset"].get<std::string>(), "high_accuracy");

  ProtoStruct bad_preset;
  bad_preset["set_depth_visual_preset"] = std::string("cinematic");
  r = rs.do_command(bad_preset);
  EXPECT_FALSE(*r["success"].get<bool>());
  EXPECT_THAT(*r["error"].get<std::string>(),
              HasSubstr("unknown visual_preset"));
}

TEST_F(SoftwareDeviceTest, DepthOptionsFromConfig_AppliedBeforeStreaming) {
  ProtoStruct knobs;
  knobs["laser_power"] = 60.0;
  knobs["depth_visual_preset"] = std::string("high_density");
  knobs["depth_emitter_enabled"] = false;
  auto &rs = streamingResource({}, knobs);

  ProtoStruct get;
  get["get_depth_options"] = true;
  auto opts = rs.do_command(get);
  EXPECT_DOUBLE_EQ(*opts["laser_power"].get<double>(), 60.0);
  EXPECT_DOUBLE_EQ(*opts["emitter_enabled"].get<double>(), 0.0);
  EXPECT_EQ(*opts["visual_preset"].get<std::string>(), "high_density");
}

// ---- single sensor configs ----

TEST_F(SoftwareDeviceTest, SensorsColorOnly_ServesColorAndRefusesPointCloud) {
  auto &cam = addCamera();
  ProtoStruct attrs;
  attrs["serial_number"] = cam.spec().serial;
  attrs["sensors"] = ProtoList{"color"};
  auto &rs = makeResource(makeConfig(attrs));
  cam.startFeeding();
  ASSERT_TRUE(waitForFrames(rs));

  auto images = rs.get_images({}, {});
  ASSERT_EQ(images.images.size(), 1u);
  EXPECT_EQ(images.images[0].source_name, "color");
  EXPECT_EQ(rs.get_properties().intrinsic_parameters.width_px, kWidth);

  try {
    rs.get_point_cloud("pointcloud/pcd", {});
    FAIL() << "point cloud needs a depth frame";
  } catch (std::runtime_error const &e) {
    EXPECT_THAT(e.what(), HasSubstr("no depth frame"));
  }
}

TEST_F(SoftwareDeviceTest, SensorsDepthOnly_ServesDepthAndRefusesPointCloud) {
  auto &cam = addCamera();
  ProtoStruct attrs;
  attrs["serial_number"] = cam.spec().serial;
  attrs["sensors"] = ProtoList{"depth"};
  auto &rs = makeResource(makeConfig(attrs));
  cam.startFeeding();
  ASSERT_TRUE(waitForFrames(rs));

  auto images = rs.get_images({}, {});
  ASSERT_EQ(images.images.size(), 1u);
  EXPECT_EQ(images.images[0].source_name, "depth");
  auto map = DepthMap::parse(images.images[0].bytes);
  EXPECT_EQ(map.at(10, 10), cam.depthAt(10, 10));
  // No color stream: intrinsics come from the depth stream.
  EXPECT_EQ(rs.get_properties().intrinsic_parameters.width_px, kWidth);

  try {
    rs.get_point_cloud("pointcloud/pcd", {});
    FAIL() << "point cloud needs a color frame";
  } catch (std::runtime_error const &e) {
    EXPECT_THAT(e.what(), HasSubstr("no color frame"));
  }
}

// ---- device assignment and lifecycle ----

TEST_F(SoftwareDeviceTest, Constructor_ThrowsWhenSerialIsNotConnected) {
  addCamera();
  EXPECT_THROW(makeResource(makeConfig(bothSensors("not-plugged-in"))),
               std::runtime_error);
  EXPECT_TRUE(assigned_serials_->get().empty());
}

TEST_F(SoftwareDeviceTest, TwoCameras_SerialPinningRoutesFramesToEachDevice) {
  SoftwareCameraSpec a;
  a.serial = "SW0000001";
  a.depth_base_mm = 400;
  SoftwareCameraSpec b;
  b.serial = "SW0000002";
  b.depth_base_mm = 700;
  auto &cam_a = addCamera(a);
  auto &cam_b = addCamera(b);
  auto &rs_b = makeResource(makeConfig(bothSensors(b.serial), "cam_b"));
  auto &rs_a = makeResource(makeConfig(bothSensors(a.serial), "cam_a"));
  cam_a.startFeeding();
  cam_b.startFeeding();
  ASSERT_TRUE(waitForFrames(rs_a));
  ASSERT_TRUE(waitForFrames(rs_b));

  auto depth_a =
      DepthMap::parse(findImage(rs_a.get_images({}, {}), "depth").bytes);
  auto depth_b =
      DepthMap::parse(findImage(rs_b.get_images({}, {}), "depth").bytes);
  EXPECT_EQ(depth_a.at(0, 0), 400);
  EXPECT_EQ(depth_b.at(0, 0), 700);
  EXPECT_EQ(*rs_a.get_status()["serial_number"].get<std::string>(), a.serial);
  EXPECT_EQ(*rs_b.get_status()["serial_number"].get<std::string>(), b.serial);
  EXPECT_EQ(assigned_serials_->get().size(), 2u);
}

TEST_F(SoftwareDeviceTest, TwoCameras_EmptySerialTakesAnUnassignedDevice) {
  SoftwareCameraSpec a;
  a.serial = "SW0000001";
  SoftwareCameraSpec b;
  b.serial = "SW0000002";
  addCamera(a);
  addCamera(b);
  auto &first = makeResource(makeConfig(bothSensors(""), "first"));
  auto &second = makeResource(makeConfig(bothSensors(""), "second"));

  auto s1 = *first.get_status()["serial_number"].get<std::string>();
  auto s2 = *second.get_status()["serial_number"].get<std::string>();
  EXPECT_NE(s1, s2);
  EXPECT_EQ(assigned_serials_->get().size(), 2u);
  // A third resource has nothing left to claim.
  EXPECT_THROW(makeResource(makeConfig(bothSensors(""), "third")),
               std::runtime_error);
}

TEST_F(SoftwareDeviceTest, Destructor_ReleasesSerialAndDeviceCanBeReopened) {
  auto &cam = addCamera();
  auto cfg = makeConfig(bothSensors(cam.spec().serial));
  for (int round = 0; round < 3; ++round) {
    auto &rs = makeResource(cfg);
    cam.startFeeding();
    ASSERT_TRUE(waitForFrames(rs)) << "round " << round;
    EXPECT_EQ(assigned_serials_->get().count(cam.spec().serial), 1u);
    cam.stopFeeding();
    destroyResources();
    EXPECT_TRUE(assigned_serials_->get().empty()) << "round " << round;
  }
}

class SoftwareDeviceEnvironment : public ::testing::Environment {
public:
  void SetUp() override { instance_ = std::make_unique<Instance>(); }
  void TearDown() override { instance_.reset(); }

private:
  std::unique_ptr<Instance> instance_;
};

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new SoftwareDeviceEnvironment);
  return RUN_ALL_TESTS();
}
