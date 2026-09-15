#pragma once

#include "sensors.hpp"
#include "time.hpp"
#include "usb.hpp"
#include "utils.hpp"

#include <array>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>
#include <librealsense2/rs.hpp>

namespace realsense {
namespace device {

// Helper macro to use resource logging with explicit logger
#define VIAM_DEVICE_LOG(logger, level) VIAM_SDK_LOG_IMPL(logger, level)

/********************** UTILITIES ************************/

template <typename SensorT>
void enableGlobalTimestamp(SensorT &sensor, viam::sdk::LogSource &logger) {
  if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
    try {
      auto sensor_type = sensors::get_sensor_type(sensor, logger);
      if (sensor_type == sensors::SensorType::unknown) {
        throw std::runtime_error("Unknown sensor type");
      }
      sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0);
      VIAM_DEVICE_LOG(logger, info)
          << "[enableGlobalTimestamp] Enabled Global Timestamp for sensor: "
          << sensors::sensor_type_to_string(sensor_type, logger);
    } catch (const std::exception &e) {
      VIAM_DEVICE_LOG(logger, warn)
          << "[enableGlobalTimestamp] Failed to enable Global Timestamp: "
          << e.what();
    }
  }
}

// Single source of truth for the visual_preset string <-> enum mapping used
// by both the config validate path and the runtime do_command handler.
inline constexpr std::array<
    std::pair<std::string_view, rs2_rs400_visual_preset>, 6>
    kVisualPresetNames{{
        {"default", RS2_RS400_VISUAL_PRESET_DEFAULT},
        {"hand", RS2_RS400_VISUAL_PRESET_HAND},
        {"high_accuracy", RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY},
        {"high_density", RS2_RS400_VISUAL_PRESET_HIGH_DENSITY},
        {"medium_density", RS2_RS400_VISUAL_PRESET_MEDIUM_DENSITY},
        {"remove_ir_pattern", RS2_RS400_VISUAL_PRESET_REMOVE_IR_PATTERN},
    }};

inline std::optional<rs2_rs400_visual_preset>
visualPresetFromString(std::string const &name) {
  for (auto const &[k, v] : kVisualPresetNames) {
    if (name == k) {
      return v;
    }
  }
  return std::nullopt;
}

inline std::optional<std::string_view>
visualPresetToString(rs2_rs400_visual_preset preset) {
  for (auto const &[k, v] : kVisualPresetNames) {
    if (v == preset) {
      return k;
    }
  }
  return std::nullopt;
}

// Apply user-supplied depth sensor knobs (visual preset, laser power, emitter,
// exposure, gain) before the pipeline starts. Each option is guarded by
// supports(); unsupported ones are logged and skipped rather than aborting.
//
// Ordering matters: visual_preset is applied first because librealsense
// presets reset the other RS2_OPTION_* values they cover.
template <typename SensorT, typename ViamConfigT>
void applyDepthSensorOptions(SensorT &sensor, ViamConfigT const &viamConfig,
                             viam::sdk::LogSource &logger) {
  auto safe_set = [&](rs2_option opt, double value, char const *name) {
    if (not sensor.supports(opt)) {
      VIAM_DEVICE_LOG(logger, warn)
          << "[applyDepthSensorOptions] sensor does not support " << name;
      return;
    }
    try {
      sensor.set_option(opt, static_cast<float>(value));
      VIAM_DEVICE_LOG(logger, info)
          << "[applyDepthSensorOptions] set " << name << "=" << value;
    } catch (std::exception const &e) {
      VIAM_DEVICE_LOG(logger, error)
          << "[applyDepthSensorOptions] failed to set " << name << ": "
          << e.what();
    }
  };

  if (viamConfig.depth_visual_preset) {
    auto preset = visualPresetFromString(*viamConfig.depth_visual_preset);
    if (not preset) {
      VIAM_DEVICE_LOG(logger, warn)
          << "[applyDepthSensorOptions] unknown visual_preset \""
          << *viamConfig.depth_visual_preset << "\"";
    } else {
      safe_set(RS2_OPTION_VISUAL_PRESET, static_cast<double>(*preset),
               "visual_preset");
    }
  }
  if (viamConfig.depth_auto_exposure and viamConfig.depth_exposure_us) {
    VIAM_DEVICE_LOG(logger, warn)
        << "[applyDepthSensorOptions] both depth_auto_exposure and "
           "depth_exposure_us are set; manual exposure will override "
           "auto-exposure";
  }
  if (viamConfig.depth_auto_exposure) {
    safe_set(RS2_OPTION_ENABLE_AUTO_EXPOSURE,
             *viamConfig.depth_auto_exposure ? 1.0 : 0.0,
             "enable_auto_exposure");
  }
  if (viamConfig.depth_exposure_us) {
    safe_set(RS2_OPTION_EXPOSURE, *viamConfig.depth_exposure_us, "exposure_us");
  }
  if (viamConfig.depth_gain) {
    safe_set(RS2_OPTION_GAIN, *viamConfig.depth_gain, "gain");
  }
  if (viamConfig.depth_emitter_enabled) {
    safe_set(RS2_OPTION_EMITTER_ENABLED,
             *viamConfig.depth_emitter_enabled ? 1.0 : 0.0, "emitter_enabled");
  }
  if (viamConfig.laser_power) {
    safe_set(RS2_OPTION_LASER_POWER, *viamConfig.laser_power, "laser_power");
  }
}

// Call this on the sensor that serves the color stream. On most D400
// cameras that is the dedicated RGB sensor; on the D405 the stereo sensor
// serves color itself, so callers gate on what the sensor serves rather
// than on its extension type.
template <typename SensorT>
void disableAutoExposurePriority(SensorT &sensor,
                                 viam::sdk::LogSource &logger) {
  // CRITICAL: Disable auto-exposure priority to maintain frame sync.
  // When enabled, RGB sensor drops frames to adjust exposure, breaking
  // temporal alignment with depth (causes 5-20ms timestamp drift).
  // Tradeoff: slightly worse exposure in changing light conditions,
  // but tight temporal sync (mostly <5ms) between color and depth.
  if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
    try {
      // Disable auto-exposure priority to ensure constant FPS
      sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0);
      VIAM_DEVICE_LOG(logger, info)
          << "[disableAutoExposurePriority] Disabled Auto-Exposure Priority "
             "(constant FPS) for color sensor";
    } catch (const std::exception &e) {
      VIAM_DEVICE_LOG(logger, warn) << "[disableAutoExposurePriority] Failed "
                                       "to disable Auto-Exposure Priority: "
                                    << e.what();
    }
  }
}

template <typename DeviceT>
std::optional<std::string> getCameraModel(std::shared_ptr<DeviceT> dev) {
  if (not dev->supports(RS2_CAMERA_INFO_NAME)) {
    return std::nullopt;
  }
  std::string camera_info = dev->get_info(RS2_CAMERA_INFO_NAME);
  // Model is the 3rd word in this string
  std::istringstream iss(camera_info);
  std::string word;
  int word_count = 0;
  while (iss >> word) {
    word_count++;
    if (word_count == 3) {
      return word;
    }
  }
  return std::nullopt;
}

template <typename DeviceT>
void printDeviceInfo(DeviceT const &dev, viam::sdk::LogSource &logger) {
  try {
    std::stringstream info;
    if (dev.supports(RS2_CAMERA_INFO_NAME)) {
      info << "DeviceInfo:\n"
           << "  Name:                           "
           << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
      info << "  Serial Number:                  "
           << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PRODUCT_LINE)) {
      info << "  Product Line:                   "
           << dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)) {
      info << "  Product ID:                      "
           << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      info << "  USB Type Descriptor:            "
           << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)) {
      info << "  Firmware Version:               "
           << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
      info << "  Recommended Firmware Version:   "
           << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)
           << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
      info << "  Firmware Update ID:   "
           << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
      info << "  Physical Port:   "
           << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_DEBUG_OP_CODE)) {
      info << "  Debug OP Code:   "
           << dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_ADVANCED_MODE)) {
      info << "  Advanced Mode:   "
           << dev.get_info(RS2_CAMERA_INFO_ADVANCED_MODE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
      info << "  Camera Locked:   "
           << dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)) {
      info << "  ASIC Serial Number:             "
           << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_DFU_DEVICE_PATH)) {
      info << "  DFU Device Path:            "
           << dev.get_info(RS2_CAMERA_INFO_DFU_DEVICE_PATH) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_IP_ADDRESS)) {
      info << "  IP Address:            "
           << dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS) << std::endl;
    }
    VIAM_DEVICE_LOG(logger, info) << info.str();

    // USB connection diagnostics — depth streaming requires USB 3.x. A USB 3
    // camera plugged into a USB 2 port or through a USB 2 cable enumerates as
    // "2.1", the most common cause of missing depth frames in the field.
    if (auto usb = getUsbConnectionType(dev)) {
      switch (usb->first) {
      case UsbConnectionType::usb2:
        VIAM_DEVICE_LOG(logger, warn)
            << "[printDeviceInfo] Device is connected via USB " << usb->second
            << ". USB 3.x is recommended for depth streaming. "
               "Depth frames may be unavailable or unreliable at USB 2.x "
               "speeds. Check that the camera is plugged into a USB 3 port "
               "using a USB 3 cable, with no USB 2 hub in between.";
        break;
      case UsbConnectionType::usb3:
        VIAM_DEVICE_LOG(logger, info)
            << "[printDeviceInfo] Device is connected via USB " << usb->second
            << ". Full resolution and frame rate options are available.";
        break;
      case UsbConnectionType::unknown:
        VIAM_DEVICE_LOG(logger, debug)
            << "[printDeviceInfo] USB type descriptor not recognized: \""
            << usb->second << "\". Cannot verify USB 3 connection.";
        break;
      }
    }

    // Warn about outdated firmware
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) &&
        dev.supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
      std::string fw = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
      std::string recommended_fw =
          dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
      if (fw != recommended_fw) {
        VIAM_DEVICE_LOG(logger, warn)
            << "[printDeviceInfo] Device firmware (" << fw
            << ") differs from recommended version (" << recommended_fw
            << "). Consider updating firmware to avoid stability issues.";
      }
    }

    // Note if camera is locked (advanced mode presets cannot be changed)
    if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
      std::string locked = dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED);
      if (locked == "YES") {
        VIAM_DEVICE_LOG(logger, info)
            << "[printDeviceInfo] Camera is locked. Advanced mode JSON "
               "presets (depth table tuning, etc.) cannot be modified. "
               "This does not affect normal operation.";
      }
    }
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[printDeviceInfo] Failed to retrieve device info with error: "
        << e.what()
        << ". Device may be in an invalid state or have firmware compatibility "
           "issues.";
  }
}

/********************** CALLBACKS ************************/

template <typename FrameT, typename FrameSetT, typename ViamConfigT>
void frameCallback(FrameT const &frame, std::uint64_t const maxFrameAgeMs,
                   boost::synchronized_value<FrameSetT> &frame_set_,
                   ViamConfigT const &viamConfig) {
  // With callbacks, all synchronized stream will arrive in a single
  // frameset
  int expected_frame_count = viamConfig.sensors.size();
  auto initial_frameset = frame.template as<FrameSetT>();
  if (not initial_frameset or initial_frameset.size() != expected_frame_count) {
    std::cerr << "[frame_callback] got count other than "
              << expected_frame_count << ": " << initial_frameset.size()
              << std::endl;
    return;
  }

  auto frameset = initial_frameset;

  double nowMs = time::getNowMs();
  auto color_frame = frameset.get_color_frame();
  if (not color_frame and
      utils::contains(sensors::SensorType::color, viamConfig.sensors)) {
    std::cerr << "[frame_callback] no color frame" << std::endl;
    return;
  }
  if (color_frame and
      not utils::contains(sensors::SensorType::color, viamConfig.sensors)) {
    std::cerr << "[frame_callback] received color frame when not expected"
              << std::endl;
    return;
  }

  if (color_frame) {
    double colorAge = nowMs - color_frame.get_timestamp();
    if (colorAge > maxFrameAgeMs) {
      std::cerr << "[frame_callback] received color frame is too stale, age: "
                << colorAge << "ms" << std::endl;
    }
  }

  auto depth_frame = frameset.get_depth_frame();
  if (not depth_frame and
      utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    std::cerr << "[frame_callback] no depth frame" << std::endl;
    return;
  }
  if (depth_frame and
      not utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    std::cerr << "[frame_callback] received depth frame when not expected"
              << std::endl;
    return;
  }
  if (depth_frame) {
    double depthAge = nowMs - depth_frame.get_timestamp();
    if (depthAge > maxFrameAgeMs) {
      std::cerr << "[frame_callback] received depth frame is too stale, age: "
                << depthAge << "ms" << std::endl;
    }
  }

  // Publish under the synchronized_value's lock; get_images/get_point_cloud/
  // watchdog read it concurrently through the same lock.
  frame_set_ = frameset;
}

/************************ STREAM PROFILES ************************/

template <typename VideoStreamProfileT>
bool checkIfMatchingColorDepthProfiles(const VideoStreamProfileT &color,
                                       const VideoStreamProfileT &depth,
                                       viam::sdk::LogSource &logger) noexcept {
  if (std::tuple(color.width(), color.height(), color.fps()) ==
      std::tuple(depth.width(), depth.height(), depth.fps())) {
    VIAM_DEVICE_LOG(logger, info)
        << "using width: " << color.width() << " height: " << color.height()
        << " fps: " << color.fps();
    return true;
  }
  return false;
}

template <typename SensorT> class SensorTypeTraits;

template <> class SensorTypeTraits<rs2::color_sensor> {
public:
  static constexpr rs2_stream stream_type = RS2_STREAM_COLOR;
  static constexpr rs2_format format_type = RS2_FORMAT_RGB8;
};

template <> class SensorTypeTraits<rs2::depth_sensor> {
public:
  static constexpr rs2_stream stream_type = RS2_STREAM_DEPTH;
  static constexpr rs2_format format_type = RS2_FORMAT_Z16;
};

template <typename DeviceT, typename ConfigT, typename SensorT,
          typename VideoStreamProfileT, typename ViamConfigT>
std::shared_ptr<ConfigT>
createSingleSensorConfig(std::shared_ptr<DeviceT> dev,
                         ViamConfigT const &viamConfig,
                         viam::sdk::LogSource &logger) {

  VIAM_DEVICE_LOG(logger, info)
      << "[createSingleSensorConfig] Creating config for single "
         "sensor: "
      << SensorTypeTraits<SensorT>::stream_type;
  auto cfg = std::make_shared<ConfigT>();

  VIAM_DEVICE_LOG(logger, info)
      << "[createSingleSensorConfig] Querying sensors";
  // Query all sensors for the device
  auto sensors = dev->query_sensors();

  // Select by stream format rather than by sensor extension type: on most
  // D400 cameras color and depth come from dedicated sensors, but on the
  // D405 the single stereo sensor serves both streams and does not cast to
  // rs2::color_sensor.
  for (auto &s : sensors) {
    auto profiles = s.get_stream_profiles();
    bool sensor_serves_stream = false;
    for (auto &cp : profiles) {
      // Skip non-video profiles (e.g. motion streams on IMU-equipped
      // models) — casting them to a video profile is invalid.
      if (not cp.template is<VideoStreamProfileT>()) {
        continue;
      }
      auto csp = cp.template as<VideoStreamProfileT>();
      if (csp.format() != SensorTypeTraits<SensorT>::format_type) {
        continue;
      }
      if (not sensor_serves_stream) {
        sensor_serves_stream = true;
        enableGlobalTimestamp(s, logger);
        // depth-sensor-only single-stream path
        if (SensorTypeTraits<SensorT>::stream_type == RS2_STREAM_DEPTH) {
          applyDepthSensorOptions(s, viamConfig, logger);
        }
        VIAM_DEVICE_LOG(logger, info)
            << "[createSingleSensorConfig] Got sensor with " << profiles.size()
            << " stream profiles, looking for matches";
      }

      if ((not viamConfig.width) or
          (viamConfig.width == csp.width()) and
              (not viamConfig.height or (viamConfig.height == csp.height()))) {
        VIAM_DEVICE_LOG(logger, info)
            << "[createSingleSensorConfig] Found matching "
               "stream profile, enabling";
        cfg->enable_stream(SensorTypeTraits<SensorT>::stream_type,
                           csp.stream_index(), csp.width(), csp.height(),
                           csp.format(), csp.fps());
        VIAM_DEVICE_LOG(logger, info)
            << "[createSingleSensorConfig] enabled stream";
        return cfg;
      }
    }
  }
  return nullptr;
}
// create a config for software depth-to-color alignment
template <typename DeviceT, typename ConfigT, typename ColorSensorT,
          typename DepthSensorT, typename VideoStreamProfileT,
          typename ViamConfigT>
std::shared_ptr<ConfigT> createSwD2CAlignConfig(std::shared_ptr<DeviceT> dev,
                                                ViamConfigT const &viamConfig,
                                                viam::sdk::LogSource &logger) {
  auto cfg = std::make_shared<ConfigT>();

  // Query all sensors for the device
  auto sensors = dev->query_sensors();

  // Collect color and depth profiles by stream format from every sensor
  // rather than looking for a dedicated color sensor and a dedicated depth
  // sensor: on the D405 the single stereo sensor serves both streams and
  // does not cast to rs2::color_sensor.
  using SensorVectorT = decltype(sensors);
  using ProfileVectorT =
      std::decay_t<decltype(std::declval<typename SensorVectorT::value_type>()
                                .get_stream_profiles())>;
  ProfileVectorT color_profiles;
  ProfileVectorT depth_profiles;
  for (auto &s : sensors) {
    enableGlobalTimestamp(s, logger);
    bool serves_color = false;
    bool serves_depth = false;
    for (auto &p : s.get_stream_profiles()) {
      // Skip non-video profiles (e.g. motion streams on IMU-equipped
      // models) — casting them to a video profile is invalid.
      if (not p.template is<VideoStreamProfileT>()) {
        continue;
      }
      auto vsp = p.template as<VideoStreamProfileT>();
      if (vsp.format() == SensorTypeTraits<ColorSensorT>::format_type) {
        color_profiles.push_back(p);
        serves_color = true;
      } else if (vsp.format() == SensorTypeTraits<DepthSensorT>::format_type) {
        depth_profiles.push_back(p);
        serves_depth = true;
      }
    }
    if (serves_color) {
      disableAutoExposurePriority(s, logger);
    }
    if (serves_depth) {
      applyDepthSensorOptions(s, viamConfig, logger);
    }
  }

  // Find matching profiles
  for (auto &cp : color_profiles) {
    auto csp = cp.template as<VideoStreamProfileT>();
    if (csp.format() != SensorTypeTraits<ColorSensorT>::format_type) {
      continue;
    }
    for (auto &dp : depth_profiles) {
      auto dsp = dp.template as<VideoStreamProfileT>();
      if (dsp.format() != SensorTypeTraits<DepthSensorT>::format_type) {
        continue;
      }
      if (checkIfMatchingColorDepthProfiles(csp, dsp, logger) and
          ((not viamConfig.width) or (viamConfig.width == csp.width())) and
          ((not viamConfig.height) or (viamConfig.height == csp.height()))) {
        VIAM_DEVICE_LOG(logger, info)
            << "[createSwD2CAlignConfig] Found matching color "
               "and depth stream profiles";
        cfg->enable_stream(SensorTypeTraits<ColorSensorT>::stream_type,
                           csp.stream_index(), csp.width(), csp.height(),
                           csp.format(), csp.fps());
        cfg->enable_stream(SensorTypeTraits<DepthSensorT>::stream_type,
                           dsp.stream_index(), dsp.width(), dsp.height(),
                           dsp.format(), dsp.fps());
        VIAM_DEVICE_LOG(logger, info)
            << "[createSwD2CAlignConfig] enabled color and depth streams";
        return cfg;
      }
    }
  }
  return nullptr;
}
template <typename DeviceT, typename ConfigT, typename ColorSensorT,
          typename DepthSensorT, typename VideoStreamProfileT,
          typename ViamConfigT>
std::shared_ptr<ConfigT> createConfig(std::shared_ptr<DeviceT> device,
                                      ViamConfigT const &viamConfig,
                                      viam::sdk::LogSource &logger) {
  std::shared_ptr<ConfigT> config = nullptr;

  if (utils::contains(sensors::SensorType::color, viamConfig.sensors) and
      utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    VIAM_DEVICE_LOG(logger, info)
        << "[createConfig] Creating config with color and "
           "depth sensors";
    config =
        createSwD2CAlignConfig<DeviceT, ConfigT, ColorSensorT, DepthSensorT,
                               VideoStreamProfileT, ViamConfigT>(
            device, viamConfig, logger);

  } else if (viamConfig.sensors.size() == 1 &&
             utils::contains(sensors::SensorType::color, viamConfig.sensors)) {
    VIAM_DEVICE_LOG(logger, info)
        << "[createConfig] Creating config with color sensor";
    config = createSingleSensorConfig<DeviceT, ConfigT, ColorSensorT,
                                      VideoStreamProfileT, ViamConfigT>(
        device, viamConfig, logger);
  } else if (viamConfig.sensors.size() == 1 &&
             utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    VIAM_DEVICE_LOG(logger, info)
        << "[createConfig] Creating config with depth sensor";
    config = createSingleSensorConfig<DeviceT, ConfigT, DepthSensorT,
                                      VideoStreamProfileT, ViamConfigT>(
        device, viamConfig, logger);
  } else {
    VIAM_DEVICE_LOG(logger, error)
        << "[createConfig] Unsupported sensor configuration requested, "
           "only "
           "'color', 'depth' or both are supported";
    throw std::runtime_error("Unsupported sensor configuration requested, "
                             "only 'color', 'depth' or both are supported");
  }
  // We are not currently supporting only depth sensor
  if (config == nullptr) {
    std::string error_message = "Current device configuration not supported";
    // USB 2 removes most resolution/frame-rate profiles, so a config that
    // works on USB 3 can fail to match here — point the user at the cable.
    auto usb = getUsbConnectionType(*device);
    if (usb and usb->first == UsbConnectionType::usb2) {
      error_message +=
          ": device is connected via USB " + usb->second +
          ", which limits the available resolutions and frame rates. Try a "
          "USB 3 port and cable, or lower the configured resolution";
    }
    VIAM_DEVICE_LOG(logger, error) << "[createConfig] " << error_message;
    throw std::runtime_error(error_message + ", check device logs");
  }

  return config;
}

/********************** DEVICE LIFECYCLE ************************/
template <typename ViamDeviceT>
bool destroyDevice(std::shared_ptr<boost::synchronized_value<ViamDeviceT>> &dev,
                   viam::sdk::LogSource &logger) noexcept {
  try {
    if (not dev) {
      VIAM_DEVICE_LOG(logger, error)
          << "[destroyDevice] trying to destroy an unexistent device";
      return false;
    }
    { // Begin scope for device lock
      auto device = dev->synchronize();
      VIAM_DEVICE_LOG(logger, info)
          << "[destroyDevice] destroying device " << device->serial_number;

      // Stop streaming if still running
      if (device->started) {
        VIAM_DEVICE_LOG(logger, info)
            << "[destroyDevice] stopping pipe " << device->serial_number;
        device->pipe->stop();
        device->started = false;
      }

      // Clear all resources
      VIAM_DEVICE_LOG(logger, info)
          << "[destroyDevice] clearing resources " << device->serial_number;
      device->pipe.reset();
      device->device.reset();
      device->config.reset();
      device->align.reset();
      device->point_cloud_filter.reset();

      VIAM_DEVICE_LOG(logger, info)
          << "[destroyDevice] device destroyed: " << device->serial_number;
    } // End scope for device lock
    dev = nullptr;
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[destroyDevice] Exception caught while destroying device: "
        << e.what();
    dev = nullptr;
    return false;
  } catch (...) {
    VIAM_DEVICE_LOG(logger, error)
        << "[destroyDevice] Unknown exception caught while destroying device";
    dev = nullptr;
    return false;
  }
  return true;
}

template <typename ViamConfigT, typename ViamDeviceT, typename DeviceT,
          typename ConfigT, typename ColorSensorT, typename DepthSensorT,
          typename VideoStreamProfileT>
std::shared_ptr<boost::synchronized_value<ViamDeviceT>>
createDevice(std::string const &serial_number, std::shared_ptr<DeviceT> dev,
             std::unordered_set<std::string> const &supported_camera_models,
             ViamConfigT const &viamConfig, viam::sdk::LogSource &logger) {
  VIAM_DEVICE_LOG(logger, info)
      << "[createDevice] creating device serial number: " << serial_number;
  auto camera_model = getCameraModel(dev);
  if (not camera_model) {
    VIAM_DEVICE_LOG(logger, error)
        << "[createDevice] Failed to register camera serial number: "
        << serial_number << " since no camera model found";
    return nullptr;
  }
  VIAM_DEVICE_LOG(logger, info)
      << "[createDevice] Found camera model: " << *camera_model;
  if (supported_camera_models.count(*camera_model) == 0) {
    std::ostringstream supported;
    bool first = true;
    for (auto const &m : supported_camera_models) {
      if (not first) {
        supported << ", ";
      }
      supported << m;
      first = false;
    }
    VIAM_DEVICE_LOG(logger, error)
        << "[createDevice] Failed to register camera serial number: "
        << serial_number << " since camera model " << *camera_model
        << " is not in the supported set {" << supported.str() << "}";
    return nullptr;
  } else {
    VIAM_DEVICE_LOG(logger, info)
        << "[createDevice] Camera model is supported: " << *camera_model;
  }

  std::shared_ptr<ConfigT> config =
      createConfig<DeviceT, ConfigT, ColorSensorT, DepthSensorT,
                   VideoStreamProfileT, ViamConfigT>(dev, viamConfig, logger);
  if (config == nullptr) {
    VIAM_DEVICE_LOG(logger, error)
        << "[createDevice] failed to create config for device: "
        << serial_number;
    return nullptr;
  }

  VIAM_DEVICE_LOG(logger, info)
      << "[createDevice] Config created for: " << serial_number;
  auto my_dev = boost::synchronized_value<ViamDeviceT>();
  my_dev->pipe = std::make_shared<std::decay_t<decltype(*my_dev->pipe)>>();
  my_dev->device = dev;
  my_dev->serial_number = serial_number;
  my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
  my_dev->align = std::make_shared<std::decay_t<decltype(*my_dev->align)>>(
      RS2_STREAM_COLOR);
  my_dev->config = config;

  VIAM_DEVICE_LOG(logger, info) << "[createDevice] created " << serial_number;
  return std::make_shared<boost::synchronized_value<ViamDeviceT>>(my_dev);
}

/********************** STREAMING LIFECYCLE ************************/
template <typename ViamDeviceT, typename FrameSetT, typename ViamConfigT>
void startDevice(std::string const &serialNumber,
                 std::shared_ptr<boost::synchronized_value<ViamDeviceT>> dev,
                 boost::synchronized_value<FrameSetT> &frameSetStorage,
                 std::uint64_t const maxFrameAgeMs,
                 ViamConfigT const &viamConfig, viam::sdk::LogSource &logger) {
  VIAM_DEVICE_LOG(logger, info)
      << "[startDevice] starting device " << serialNumber;
  if (not dev) {
    std::ostringstream buffer;
    buffer << "[startDevice] unable to start device " << serialNumber
           << " since " << "viam device wrapper does not exist";
    throw std::runtime_error(buffer.str());
  }
  { // Begin scope for dev_ptr lock
    auto dev_ptr = dev->synchronize();
    if (dev_ptr->started) {
      std::ostringstream buffer;
      buffer << "[startDevice] unable to start already started device "
             << serialNumber;
      throw std::invalid_argument(buffer.str());
    }

    dev_ptr->config->enable_device(serialNumber);
    dev_ptr->pipe->start(*dev_ptr->config, [maxFrameAgeMs, &frameSetStorage,
                                            viamConfig](auto const &frame) {
      frameCallback(frame, maxFrameAgeMs, frameSetStorage, viamConfig);
    });

    dev_ptr->started = true;
  } // End scope for dev_ptr lock
  VIAM_DEVICE_LOG(logger, info)
      << "[startDevice]  device started " << serialNumber;
}

template <typename ViamDeviceT>
bool stopDevice(std::shared_ptr<boost::synchronized_value<ViamDeviceT>> &dev,
                viam::sdk::LogSource &logger) noexcept {
  try {
    if (not dev) {
      VIAM_DEVICE_LOG(logger, error)
          << "[stopDevice] trying to stop a device that does not exist";
      return false;
    }
    { // Begin scope for dev_ptr lock
      auto dev_ptr = dev->synchronize();
      if (not dev_ptr->started) {
        // Idempotent "ensure stopped": already stopped is success, so a restart
        // can safely retry startDevice after a failed start.
        VIAM_DEVICE_LOG(logger, debug)
            << "[stopDevice] device already stopped: "
            << dev_ptr->serial_number;
        return true;
      }

      dev_ptr->pipe->stop();
      dev_ptr->started = false;

    } // End scope for dev_ptr lock
    return true;
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[stopDevice] Exception caught while stopping device: " << e.what();
  } catch (...) {
    VIAM_DEVICE_LOG(logger, error)
        << "[stopDevice] Unknown exception caught while stopping device";
  }
  return false;
}
} // namespace device
} // namespace realsense
