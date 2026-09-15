#pragma once
#include "device.hpp"
#include "download_utils.hpp"
#include "encoding.hpp"
#include "extrinsics.hpp"
#include "firmware_update.hpp"
#include "sensors.hpp"
#include "time.hpp"
#include "utils.hpp"
#include "watchdog.hpp"
#include "zip_utils.hpp"
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>

#include <librealsense2/rs.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/thread/synchronized_value.hpp>
namespace realsense {
static const std::unordered_set<std::string> SUPPORTED_CAMERA_MODELS = {
    "D415", "D435", "D435I"};
static constexpr std::uint64_t MAX_FRAME_AGE_MS =
    1e3; // time until a frame is considered stale, in miliseconds (equal to 1
static constexpr size_t MAX_GRPC_MESSAGE_SIZE =
    33554432; // 32MB gRPC message size limit
static constexpr std::uint64_t MAX_FRAME_SET_TIME_DIFF_MS =
    2; // max time difference between frames in a frameset to be considered
       // simultaneous, in miliseconds (equal to 2 ms)
static constexpr std::uint64_t TIMESTAMP_WARNING_LOG_INTERVAL_MS =
    60000; // 1
           // minute

enum class DoCommand : uint8_t {
  UPDATE_FIRMWARE,
  // Runtime depth-sensor tuning. Each is a single-key payload; the value
  // is the desired option setting.
  SET_LASER_POWER,         // double 0..360
  SET_DEPTH_EMITTER,       // bool
  SET_DEPTH_VISUAL_PRESET, // string: default | hand | high_accuracy | ...
  SET_DEPTH_EXPOSURE_US,   // double µs
  SET_DEPTH_AUTO_EXPOSURE, // bool
  SET_DEPTH_GAIN,          // double 0..248
  GET_DEPTH_OPTIONS,       // no payload (any value) — returns current values
  UNKNOWN = std::numeric_limits<uint8_t>::max()
};

static const std::unordered_map<std::string, uint8_t> DoCommandMap{
    {{"update_firmware", static_cast<uint8_t>(DoCommand::UPDATE_FIRMWARE)},
     {"set_laser_power", static_cast<uint8_t>(DoCommand::SET_LASER_POWER)},
     {"set_depth_emitter", static_cast<uint8_t>(DoCommand::SET_DEPTH_EMITTER)},
     {"set_depth_visual_preset",
      static_cast<uint8_t>(DoCommand::SET_DEPTH_VISUAL_PRESET)},
     {"set_depth_exposure_us",
      static_cast<uint8_t>(DoCommand::SET_DEPTH_EXPOSURE_US)},
     {"set_depth_auto_exposure",
      static_cast<uint8_t>(DoCommand::SET_DEPTH_AUTO_EXPOSURE)},
     {"set_depth_gain", static_cast<uint8_t>(DoCommand::SET_DEPTH_GAIN)},
     {"get_depth_options", static_cast<uint8_t>(DoCommand::GET_DEPTH_OPTIONS)},
     {"unknown", static_cast<uint8_t>(DoCommand::UNKNOWN)}}};

const std::string service_name = "viam_realsense";

const std::string kColorSourceName = "color";
const std::string kDepthSourceName = "depth";
const std::string kPcdMimeType = "pointcloud/pcd";

template <typename SynchronizedContextT> class Realsense;

template <typename SynchronizedContextT> class RealsenseContext {
public:
  RealsenseContext(std::shared_ptr<SynchronizedContextT> ctx)
      : rs_context_(ctx) {
    setupCallback();
  }

  auto query_devices() const {
    auto rs_context = rs_context_->synchronize();
    return rs_context->query_devices();
  }

  void addInstance(Realsense<SynchronizedContextT> *instance) {
    instances_->insert(instance);
  }

  void removeInstance(Realsense<SynchronizedContextT> *instance) {
    instances_->erase(instance);
  }

  // Clear the devices changed callback (e.g., during firmware update)
  void clearDevicesChangedCallback() {
    auto rs_context = rs_context_->synchronize();
    rs_context->set_devices_changed_callback([](rs2::event_information &) {});
  }

  // Set a custom devices changed callback (e.g., for firmware update event
  // detection)
  template <typename CallbackT>
  void setDevicesChangedCallback(CallbackT callback) {
    auto rs_context = rs_context_->synchronize();
    rs_context->set_devices_changed_callback(callback);
  }

  // Restore the default devices changed callback
  void restoreDevicesChangedCallback() { setupCallback(); }

private:
  std::shared_ptr<SynchronizedContextT> rs_context_;
  boost::synchronized_value<
      std::unordered_set<Realsense<SynchronizedContextT> *>>
      instances_{std::unordered_set<Realsense<SynchronizedContextT> *>{}};

  void setupCallback() {
    auto rs_context = rs_context_->synchronize();
    // Set the callback to notify all instances when devices change
    rs_context->set_devices_changed_callback(
        [this](rs2::event_information &info) { notifyAllInstances(info); });
  }

  void notifyAllInstances(rs2::event_information &info) {
    auto instances_guard = instances_.synchronize();
    std::vector<Realsense<SynchronizedContextT> *> failed_instances;

    for (Realsense<SynchronizedContextT> *instance : *instances_guard) {
      if (instance != nullptr) {
        try {
          instance->handleDeviceChange(info);
        } catch (const std::exception &e) {
          std::cerr << "[RealsenseContext] Error notifying instance: "
                    << e.what() << std::endl;
          // Consider whether to remove failed instances
          failed_instances.push_back(instance);
        }
      }
    }

    for (auto *failed : failed_instances) {
      instances_guard->erase(failed);
    }
  }
};

struct RsResourceConfig {
  std::string resource_name{};
  std::string serial_number{};
  std::vector<sensors::SensorType> sensors{};
  std::optional<int> width{};
  std::optional<int> height{};
  // When true, depth frames returned by get_images() are spatially aligned
  // to the color frame using librealsense's rs2::align filter. This makes
  // depth_np[v, u] correspond to the same physical point as color_np[v, u]
  // (depth pixel grid is resampled to color frame), which is required for
  // any consumer that combines a 2D color-image mask with the depth array.
  // Defaults to false to preserve the historical behavior of returning the
  // raw, unaligned streams.
  bool align_color_depth{false};

  // Depth sensor tuning (all optional — if unset, the driver doesn't touch
  // that option and the camera keeps its factory/firmware default).
  std::optional<double> laser_power{}; // 0..360
  std::optional<bool> depth_emitter_enabled{};
  // One of: "default" | "hand" | "high_accuracy" | "high_density"
  //       | "medium_density" | "remove_ir_pattern"
  std::optional<std::string> depth_visual_preset{};
  std::optional<double> depth_exposure_us{};
  std::optional<bool> depth_auto_exposure{};
  std::optional<double> depth_gain{};

  RsResourceConfig() = default;

  explicit RsResourceConfig(std::string const &serial_number,
                            std::string const &resource_name,
                            std::vector<sensors::SensorType> const &sensors,
                            std::optional<int> width = std::nullopt,
                            std::optional<int> height = std::nullopt,
                            bool align_color_depth = false)
      : serial_number(serial_number), resource_name(resource_name),
        sensors(sensors), width(width), height(height),
        align_color_depth(align_color_depth) {}
};

struct DeviceFunctions {
  std::function<bool(
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      viam::sdk::LogSource &)>
      stopDevice;
  std::function<bool(
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      viam::sdk::LogSource &)>
      destroyDevice;
  std::function<void(const rs2::device &, viam::sdk::LogSource &)>
      printDeviceInfo;
  std::function<
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>(
          std::string const &, std::shared_ptr<rs2::device>,
          std::unordered_set<std::string> const &,
          realsense::RsResourceConfig const &, viam::sdk::LogSource &)>
      createDevice;
  std::function<void(
      std::string const &,
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      boost::synchronized_value<rs2::frameset> &, std::uint64_t,
      realsense::RsResourceConfig const &, viam::sdk::LogSource &)>
      startDevice;
  std::function<bool(std::shared_ptr<rs2::device>, std::string &current,
                     std::string &recommended)>
      getFirmwareVersions;
};

template <typename SynchronizedContextT>
class Realsense final : public viam::sdk::Camera {
public:
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext<SynchronizedContextT>> ctx,
            std::shared_ptr<
                boost::synchronized_value<std::unordered_set<std::string>>>
                assigned_serials)
      : Realsense(deps, cfg, ctx, createDefaultDeviceFunctions(),
                  assigned_serials) {}
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext<SynchronizedContextT>> ctx,
            DeviceFunctions device_funcs,
            std::shared_ptr<
                boost::synchronized_value<std::unordered_set<std::string>>>
                assigned_serials)
      : Camera(cfg.name()), config_(configure(deps, cfg)), realsense_ctx_(ctx),
        device_funcs_(device_funcs), assigned_serials_(assigned_serials) {

    std::string requested_serial_number = config_->serial_number;
    VIAM_RESOURCE_LOG(info)
        << "[constructor] start for resource " << config_->resource_name
        << " with serial number " << requested_serial_number;

    // This will the initial set of connected devices (i.e. the devices that
    // were connected before the callback was set)
    auto device_list = realsense_ctx_->query_devices();
    VIAM_RESOURCE_LOG(info)
        << "[constructor] start for resource " << config_->resource_name
        << " number of devices found: " << device_list.size();
    if (not assign_and_initialize_device(device_list)) {
      if (not requested_serial_number.empty()) {
        VIAM_RESOURCE_LOG(error) << "[constructor] failed to start device "
                                 << requested_serial_number;
        throw std::runtime_error("failed to start device " +
                                 requested_serial_number);
      } else {
        VIAM_RESOURCE_LOG(error) << "[constructor] failed to start a device";
        throw std::runtime_error("failed to start a device");
      }
    }
    realsense_ctx_->addInstance(this);
    physical_camera_assigned_ = true;

    VIAM_RESOURCE_LOG(info)
        << "Realsense constructor end " << requested_serial_number;
  }
  ~Realsense() {
    // Stop the watchdog FIRST so its thread joins before we touch
    // device_/latest_frameset_. Otherwise a watchdog-triggered restart
    // could race with destructor teardown.
    watchdog_.reset();
    if (device_) {
      { // Begin scope for device_guard lock
        auto device_guard = device_->synchronize();
        VIAM_RESOURCE_LOG(info) << "[destructor] Realsense destructor start "
                                << device_guard->serial_number;
        { // Begin scope for serials_guard lock
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_guard->serial_number);
        } // End scope for serials_guard lock
      } // End scope for device_guard lock
      realsense_ctx_->removeInstance(this);
    }

    // Now call stopDevice and destroyDevice (these will lock internally)
    device_funcs_.stopDevice(device_, this->logger_);
    device_funcs_.destroyDevice(device_, this->logger_);
    VIAM_RESOURCE_LOG(info) << "[destructor] Realsense destructor end";
  }

  DoCommand get_do_command(const viam::sdk::ProtoStruct &command) const {
    for (auto const &proto_command : command) {
      if (DoCommandMap.count(proto_command.first)) {
        return static_cast<DoCommand>(DoCommandMap.at(proto_command.first));
      }
    }
    return DoCommand::UNKNOWN;
  }

  // Find the live depth sensor on the currently-attached rs2::device and
  // invoke `fn(depth_sensor)`. The device lock is released before `fn` runs
  // so blocking USB I/O (e.g., set_option) does not block frame callbacks.
  template <typename F> bool withDepthSensor(F &&fn) {
    if (not device_) {
      return false;
    }
    std::optional<rs2::depth_sensor> ds;
    {
      auto guard = device_->synchronize();
      if (not guard->device) {
        return false;
      }
      for (auto &s : guard->device->query_sensors()) {
        if (s.template is<rs2::depth_sensor>()) {
          ds = s.template as<rs2::depth_sensor>();
          break;
        }
      }
    }
    if (not ds) {
      return false;
    }
    fn(*ds);
    return true;
  }

  // Apply a single rs2_option to the live depth sensor and report status as
  // a ProtoStruct response. Note: runtime overrides are not persisted across
  // a pipeline restart — the next reconfigure re-applies values from the
  // resource config.
  viam::sdk::ProtoStruct setDepthOption(rs2_option opt, double value,
                                        char const *name) {
    viam::sdk::ProtoStruct response;
    bool sensor_present = withDepthSensor([&](rs2::depth_sensor &ds) {
      if (not ds.supports(opt)) {
        response["success"] = false;
        response["error"] = std::string("sensor does not support ") + name;
        return;
      }
      try {
        ds.set_option(opt, static_cast<float>(value));
        response["success"] = true;
        response["option"] = std::string(name);
        response["value"] = value;
      } catch (std::exception const &e) {
        response["success"] = false;
        response["error"] = std::string(e.what());
      }
    });
    if (not sensor_present) {
      response["success"] = false;
      response["error"] = "no live depth sensor available";
    }
    return response;
  }

  // Extract a typed argument from a single-key command, or return a populated
  // error ProtoStruct via `err_out` if the type doesn't match. Returns
  // std::nullopt when an error has been written.
  template <typename T>
  static std::optional<T> extractArg(viam::sdk::ProtoStruct const &command,
                                     char const *cmd_name, char const *expected,
                                     viam::sdk::ProtoStruct &err_out) {
    auto const &val = command.begin()->second;
    if (not val.template is_a<T>()) {
      err_out["success"] = false;
      err_out["error"] = std::string(cmd_name) + " expects " + expected;
      return std::nullopt;
    }
    return *val.template get<T>();
  }

  viam::sdk::ProtoStruct
  do_command(const viam::sdk::ProtoStruct &command) override {
    VIAM_RESOURCE_LOG(info)
        << "[do_command] Received do_command, waiting for lock...";

    // Only 1 command is allowed at a time
    if (command.size() > 1) {
      viam::sdk::ProtoStruct response;
      response["success"] = false;
      response["error"] = "do_command must contain exactly one parameter";
      VIAM_RESOURCE_LOG(error) << "[do_command] Invalid command: contains "
                               << command.size() << " parameters, expected 1";
      return response;
    }

    // Acquire lock to prevent concurrent do_command calls
    // This is especially important for long-running operations like firmware
    // updates If this blocks, another do_command is currently executing
    std::lock_guard<std::mutex> lock(do_command_mutex_);

    VIAM_RESOURCE_LOG(info) << "[do_command] Lock acquired, processing command";
    DoCommand do_command = get_do_command(command);

    try {
      // Runtime depth-sensor tuning commands. Each is a single-key payload.
      viam::sdk::ProtoStruct err;
      switch (do_command) {
      case DoCommand::SET_LASER_POWER: {
        auto v =
            extractArg<double>(command, "set_laser_power", "a number", err);
        if (not v)
          return err;
        return setDepthOption(RS2_OPTION_LASER_POWER, *v, "laser_power");
      }
      case DoCommand::SET_DEPTH_EMITTER: {
        auto v = extractArg<bool>(command, "set_depth_emitter", "a bool", err);
        if (not v)
          return err;
        return setDepthOption(RS2_OPTION_EMITTER_ENABLED, *v ? 1.0 : 0.0,
                              "emitter_enabled");
      }
      case DoCommand::SET_DEPTH_VISUAL_PRESET: {
        auto v = extractArg<std::string>(command, "set_depth_visual_preset",
                                         "a string", err);
        if (not v)
          return err;
        auto preset = device::visualPresetFromString(*v);
        if (not preset) {
          err["success"] = false;
          err["error"] = std::string("unknown visual_preset: ") + *v;
          return err;
        }
        return setDepthOption(RS2_OPTION_VISUAL_PRESET,
                              static_cast<double>(*preset), "visual_preset");
      }
      case DoCommand::SET_DEPTH_EXPOSURE_US: {
        auto v = extractArg<double>(command, "set_depth_exposure_us",
                                    "a number", err);
        if (not v)
          return err;
        return setDepthOption(RS2_OPTION_EXPOSURE, *v, "exposure_us");
      }
      case DoCommand::SET_DEPTH_AUTO_EXPOSURE: {
        auto v =
            extractArg<bool>(command, "set_depth_auto_exposure", "a bool", err);
        if (not v)
          return err;
        return setDepthOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE, *v ? 1.0 : 0.0,
                              "enable_auto_exposure");
      }
      case DoCommand::SET_DEPTH_GAIN: {
        auto v = extractArg<double>(command, "set_depth_gain", "a number", err);
        if (not v)
          return err;
        return setDepthOption(RS2_OPTION_GAIN, *v, "gain");
      }
      case DoCommand::GET_DEPTH_OPTIONS: {
        viam::sdk::ProtoStruct r;
        bool sensor_present = withDepthSensor([&](rs2::depth_sensor &ds) {
          auto safe_get = [&](rs2_option o, char const *k) {
            if (not ds.supports(o)) {
              return;
            }
            try {
              r[k] = static_cast<double>(ds.get_option(o));
            } catch (std::exception const &e) {
              VIAM_RESOURCE_LOG(warn) << "[get_depth_options] failed to read "
                                      << k << ": " << e.what();
            }
          };
          safe_get(RS2_OPTION_LASER_POWER, "laser_power");
          safe_get(RS2_OPTION_EMITTER_ENABLED, "emitter_enabled");
          safe_get(RS2_OPTION_EXPOSURE, "exposure_us");
          safe_get(RS2_OPTION_ENABLE_AUTO_EXPOSURE, "auto_exposure");
          safe_get(RS2_OPTION_GAIN, "gain");
          // visual_preset is exposed as a string for symmetry with the SET
          // path; the integer ProtoValue would force callers to know the
          // rs2_rs400_visual_preset enum layout.
          if (ds.supports(RS2_OPTION_VISUAL_PRESET)) {
            try {
              auto preset = static_cast<rs2_rs400_visual_preset>(
                  static_cast<int>(ds.get_option(RS2_OPTION_VISUAL_PRESET)));
              if (auto name = device::visualPresetToString(preset)) {
                r["visual_preset"] = std::string(*name);
              }
            } catch (std::exception const &e) {
              VIAM_RESOURCE_LOG(warn)
                  << "[get_depth_options] failed to read visual_preset: "
                  << e.what();
            }
          }
        });
        r["sensor_present"] = sensor_present;
        return r;
      }
      default:
        break;
      }

      // Check if command contains "firmware_update"
      if (do_command == DoCommand::UPDATE_FIRMWARE) {
        VIAM_RESOURCE_LOG(info) << "[do_command] Received update_firmware";
#ifdef __APPLE__
        // Firmware update is not supported on macOS yet
        viam::sdk::ProtoStruct response;
        response["success"] = false;
        response["error"] = "Firmware update is not supported on macOS";
        VIAM_RESOURCE_LOG(error)
            << "[do_command] Firmware update not supported on macOS";
        return response;
#else
        // Extract the firmware URL from the command
        // Command structure: {"update_firmware": "url"} or {"update_firmware":
        // ""} Note: command.size() is already validated to be exactly 1 at the
        // top of do_command
        auto it = command.begin();
        auto const &fw_param = it->second;

        if (!fw_param.is_a<std::string>()) {
          viam::sdk::ProtoStruct response;
          response["success"] = false;
          response["error"] = "Firmware update URL must be a string";
          VIAM_RESOURCE_LOG(error) << "[do_command] Invalid firmware update "
                                      "parameter type (expected string)";
          return response;
        }

        std::string firmware_url = *fw_param.get<std::string>();
        return handleFirmwareUpdate(firmware_url);
#endif
      }

      VIAM_RESOURCE_LOG(error) << "[do_command] Unknown command";
      viam::sdk::ProtoStruct response;
      response["error"] =
          "Unknown command. Supported commands: update_firmware";
      return response;

    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[do_command] Error: " << e.what();
      viam::sdk::ProtoStruct response;
      response["error"] = std::string("Command failed: ") + e.what();
      return response;
    }
  }

  viam::sdk::ProtoStruct get_status() override {
    viam::sdk::ProtoStruct status;
    status["physical_camera_assigned"] = physical_camera_assigned_.get();
    status["is_recovery_mode"] = is_recovery_mode_.get();
    if (device_) {
      auto device_guard = device_->synchronize();
      status["serial_number"] = device_guard->serial_number;
      status["streaming"] = device_guard->started;
    }
    return status;
  }

  viam::sdk::Camera::image_collection
  get_images(std::vector<std::string> filter_source_names,
             const viam::sdk::ProtoStruct &extra) override {
    try {
      if (is_recovery_mode_.get()) {
        std::string error_msg =
            "Camera is in recovery/DFU mode and cannot stream images. "
            "Please update the firmware using do_command with firmware_update "
            "parameter.";
        VIAM_RESOURCE_LOG(error) << "[get_images] " << error_msg;
        throw std::runtime_error(error_msg);
      }

      bool should_process_color = false;
      bool should_process_depth = false;

      if (filter_source_names.empty()) {
        should_process_color = true;
        should_process_depth = true;
      } else {
        for (const auto &name : filter_source_names) {
          if (name == kColorSourceName) {
            should_process_color = true;
          }
          if (name == kDepthSourceName) {
            should_process_depth = true;
          }
        }
      }

      // get() copies the frameset under the synchronized_value's lock, so it
      // is race-free wrt frameCallback's locked assignment.
      auto fs = latest_frameset_.get();
      if (not fs) {
        VIAM_RESOURCE_LOG(error) << "[get_images] no frameset available";
        throw std::runtime_error("no frameset available");
      }
      VIAM_RESOURCE_LOG(debug) << "[get_images] start";
      std::string serial_number = config_->serial_number;

      // Optionally align depth to color so that depth_np[v, u] and
      // color_np[v, u] refer to the same physical point. The IR/depth and
      // RGB sensors on a RealSense are physically offset (~15-55 mm) and
      // have different FOVs, so without this step a 2D mask drawn on the
      // color image cannot be used directly to look up depth values. This
      // is opt-in to preserve the historical behavior of returning the raw
      // unaligned streams.
      if (config_->align_color_depth) {
        // librealsense's align filter caches state keyed to the stream
        // profiles it has seen and is not thread safe, so an aligner must
        // never be shared across cameras (upstream guidance is one rs2::align
        // per pipeline; see librealsense#10565). Use this device's own
        // aligner under the device lock — it is created in createDevice and
        // rebuilt on reconnect, which also drops caches that would otherwise
        // go stale when the pipeline restarts. This mirrors how
        // get_point_cloud uses the per-device point_cloud_filter.
        if (not device_) {
          VIAM_RESOURCE_LOG(error)
              << "[get_images] no device available to align depth to color";
          throw std::runtime_error(
              "no device available to align depth to color");
        }
        auto my_dev = device_->synchronize();
        if (not my_dev->align) {
          throw std::runtime_error(
              "device has no aligner to align depth to color");
        }
        fs = my_dev->align->process(fs);
      }

      std::vector<sensors::SensorType> sensors = config_->sensors;

      viam::sdk::Camera::image_collection response;
      for (auto const &sensor : sensors) {
        if (sensor == sensors::SensorType::color) {
          if (!should_process_color) {
            continue;
          }
          auto color = fs.get_color_frame();
          response.images.emplace_back(
              encoding::encodeVideoFrameToResponse(color));
          std::uint64_t timestamp =
              static_cast<std::uint64_t>(std::llround(color.get_timestamp()));

          std::chrono::milliseconds latestTimestamp(timestamp);
          response.metadata.captured_at = viam::sdk::time_pt{
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  latestTimestamp)};
        } else if (sensor == sensors::SensorType::depth) {
          if (!should_process_depth) {
            continue;
          }
          auto depth = fs.get_depth_frame();
          response.images.emplace_back(
              encoding::encodeDepthFrameToResponse(depth));
          std::uint64_t timestamp =
              static_cast<std::uint64_t>(std::llround(depth.get_timestamp()));

          std::chrono::milliseconds latestTimestamp(timestamp);
          response.metadata.captured_at = viam::sdk::time_pt{
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  latestTimestamp)};
        } else {
          throw std::invalid_argument(
              "unknown sensor: " +
              sensors::sensor_type_to_string(sensor, this->logger_));
        }
      }

      // if both color and depth are requested, use the older timestamp of the
      // two
      if (utils::contains(sensors::SensorType::color, sensors) and
          utils::contains(sensors::SensorType::depth, sensors) and
          should_process_color and should_process_depth) {
        auto const color = fs.get_color_frame();
        auto const depth = fs.get_depth_frame();
        auto const timeDiffMs = static_cast<std::uint64_t>(std::llround(
            std::abs(color.get_timestamp() - depth.get_timestamp())));
        auto const colorTS =
            static_cast<std::uint64_t>(std::llround(color.get_timestamp()));
        auto const depthTS =
            static_cast<std::uint64_t>(std::llround(depth.get_timestamp()));
        // log if the timestamps differ more than MAX_FRAME_SET_TIME_DIFF_MS,
        // at most once every TIMESTAMP_WARNING_LOG_INTERVAL_MS at warning
        // level and always at debug level
        if (timeDiffMs > MAX_FRAME_SET_TIME_DIFF_MS) {
          std::uint64_t now_ms = time::getNowMs();
          bool should_warn = false;
          {
            auto guard = last_timestamp_warning_log_time_ms_.synchronize();
            if (now_ms - *guard > TIMESTAMP_WARNING_LOG_INTERVAL_MS) {
              *guard = now_ms;
              should_warn = true;
            }
          }

          if (should_warn) {
            VIAM_RESOURCE_LOG(warn)
                << "color and depth timestamps differ by " << timeDiffMs
                << "ms, using older timestamp. "
                   "(This warning throttled to once per "
                << TIMESTAMP_WARNING_LOG_INTERVAL_MS / 1000
                << "s; see debug for all"
                   " occurrences)";
          } else {
            VIAM_RESOURCE_LOG(debug)
                << "color and depth timestamps differ by " << timeDiffMs
                << "ms, using older timestamp. color: " << colorTS
                << "ms depth: " << depthTS << "ms";
          }
        }

        // use the older of the two timestamps
        std::uint64_t timestamp = std::min(depthTS, colorTS);

        std::chrono::milliseconds latestTimestamp(timestamp);
        response.metadata.captured_at = viam::sdk::time_pt{
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                latestTimestamp)};
      }

      VIAM_RESOURCE_LOG(debug) << "[get_images] end";
      return response;
    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[get_images] error: " << e.what();
      throw std::runtime_error("failed to create images: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::point_cloud
  get_point_cloud(std::string mime_type,
                  const viam::sdk::ProtoStruct &extra) override {
    try {
      if (is_recovery_mode_.get()) {
        std::string error_msg =
            "Camera is in recovery/DFU mode and cannot stream point clouds. "
            "Please update the firmware using do_command with firmware_update "
            "parameter.";
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] " << error_msg;
        throw std::runtime_error(error_msg);
      }
      // get() copies the frameset under the synchronized_value's lock, so it
      // is race-free wrt frameCallback's locked assignment.
      auto fs = latest_frameset_.get();
      if (not fs) {
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] no frameset available";
        throw std::runtime_error("no frameset available");
      }
      VIAM_RESOURCE_LOG(debug) << "[get_point_cloud] start";

      double nowMs = time::getNowMs();

      rs2::video_frame color_frame = fs.get_color_frame();
      if (not color_frame) {
        throw std::invalid_argument("no color frame");
      }

      time::throwIfTooOld(nowMs, color_frame.get_timestamp(), MAX_FRAME_AGE_MS,
                          "no recent color frame: check USB connection");

      rs2::depth_frame depth_frame = fs.get_depth_frame();
      if (not depth_frame) {
        throw std::invalid_argument("no depth frame");
      }
      time::throwIfTooOld(nowMs, depth_frame.get_timestamp(), MAX_FRAME_AGE_MS,
                          "no recent depth frame: check USB connection");

      if (color_frame.get_data() == nullptr) {
        throw std::runtime_error("[get_point_cloud] color data is null");
      }

      if (depth_frame.get_data() == nullptr) {
        throw std::runtime_error("[get_point_cloud] depth data is null");
      }

      if (not device_) {
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] no device available";
        throw std::runtime_error("no device available");
      }
      std::vector<std::uint8_t> data;
      { // Begin scope for my_dev lock
        auto my_dev = device_->synchronize();

        if (not my_dev->started) {
          throw std::runtime_error("device is not started");
        }

        data = encoding::encodeRGBPointsToPCD(
            my_dev->point_cloud_filter->process(fs), this->logger_);
      } // End scope for my_dev lock

      if (data.size() > MAX_GRPC_MESSAGE_SIZE) {
        VIAM_RESOURCE_LOG(error)
            << "[get_point_cloud] data size exceeds gRPC message size limit";
        throw std::runtime_error("point cloud size " +
                                 std::to_string(data.size()) +
                                 " exceeds gRPC message size limit of " +
                                 std::to_string(MAX_GRPC_MESSAGE_SIZE));
      }

      VIAM_RESOURCE_LOG(debug) << "[get_point_cloud] end";
      return viam::sdk::Camera::point_cloud{kPcdMimeType, data};
    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[get_point_cloud] error: " << e.what();
      throw std::runtime_error("failed to create pointcloud: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::properties get_properties() override {
    try {
      VIAM_RESOURCE_LOG(debug) << "[get_properties] start";
      if (not device_) {
        VIAM_RESOURCE_LOG(error) << "[get_properties] no device available";
        throw std::runtime_error("no device available");
      }
      auto fillResp = [this](viam::sdk::Camera::properties &p,
                             rs2_intrinsics const &props,
                             const rs2::stream_profile &reference_stream) {
        p.supports_pcd = true;
        p.intrinsic_parameters.width_px = props.width;
        p.intrinsic_parameters.height_px = props.height;
        p.intrinsic_parameters.focal_x_px = props.fx;
        p.intrinsic_parameters.focal_y_px = props.fy;
        p.intrinsic_parameters.center_x_px = props.ppx;
        p.intrinsic_parameters.center_y_px = props.ppy;

        // Identity: the point cloud is emitted in the same frame these
        // intrinsics describe. See extrinsics::get_reported_extrinsics.
        p.extrinsic_parameters =
            realsense::extrinsics::get_reported_extrinsics(reference_stream);

        switch (props.model) {
        case RS2_DISTORTION_BROWN_CONRADY:
        case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
          p.distortion_parameters.model = "brown_conrady";
          break;
        case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
          p.distortion_parameters.model = "inverse_brown_conrady";
          break;
        default:
          p.distortion_parameters.model = "";
          break;
        }

        // RealSense reports coeffs in OpenCV order [k1, k2, p1, p2, k3], but
        // rdk's (Inverse)BrownConrady distorter expects [k1, k2, k3, p1, p2]
        // (all radial terms, then tangential). Reorder to match rdk's contract.
        if (not p.distortion_parameters.model.empty()) {
          p.distortion_parameters.parameters = {
              props.coeffs[0], // RadialK1     (k1)
              props.coeffs[1], // RadialK2     (k2)
              props.coeffs[4], // RadialK3     (k3)
              props.coeffs[2], // TangentialP1 (p1)
              props.coeffs[3], // TangentialP2 (p2)
          };
        }

        std::stringstream coeffs_stream;
        for (size_t i = 0; i < p.distortion_parameters.parameters.size(); ++i) {
          if (i > 0)
            coeffs_stream << ", ";
          coeffs_stream << p.distortion_parameters.parameters[i];
        }

        VIAM_RESOURCE_LOG(debug)
            << "[get_properties] properties: ["
            << "width: " << p.intrinsic_parameters.width_px << ", "
            << "height: " << p.intrinsic_parameters.height_px << ", "
            << "focal_x: " << p.intrinsic_parameters.focal_x_px << ", "
            << "focal_y: " << p.intrinsic_parameters.focal_y_px << ", "
            << "center_x: " << p.intrinsic_parameters.center_x_px << ", "
            << "center_y: " << p.intrinsic_parameters.center_y_px << ", "
            << "distortion_model: " << p.distortion_parameters.model << ", "
            << "distortion_coeffs: [" << coeffs_stream.str() << "]" << "]";
      };
      viam::sdk::Camera::properties response{};
      { // Begin scope for my_dev lock
        auto my_dev = device_->synchronize();
        std::string serial_number = my_dev->serial_number;
        if (not my_dev->started or not my_dev->pipe) {
          std::ostringstream buffer;
          buffer << service_name << ": device with serial number "
                 << serial_number << " is not longer started";
          throw std::invalid_argument(buffer.str());
        }

        auto profile = my_dev->pipe->get_active_profile();
        // Fetch streams defensively: a stream absent from the active profile
        // (e.g. sensors: ["depth"] has no color stream) throws rather than
        // returning an invalid profile, so we catch and leave unset.
        rs2::video_stream_profile depth_stream, color_stream;
        try {
          depth_stream = profile.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>();
        } catch (const std::exception &e) {
          VIAM_RESOURCE_LOG(debug)
              << "[get_properties] depth stream not in active profile: "
              << e.what();
        }
        try {
          color_stream = profile.get_stream(RS2_STREAM_COLOR)
                             .as<rs2::video_stream_profile>();
        } catch (const std::exception &e) {
          VIAM_RESOURCE_LOG(debug)
              << "[get_properties] color stream not in active profile: "
              << e.what();
        }

        // The camera reference frame is the color imager when color is
        // configured (the stream most callers derive poses from), otherwise
        // depth. Everything this resource reports is expressed in that one
        // frame: the intrinsics below, the point cloud get_point_cloud() emits
        // (it aligns depth to color before deprojecting — see
        // device::PointCloudFilter), and the box pose from get_geometries().
        // Extrinsics are therefore identity; see
        // extrinsics::get_reported_extrinsics for why publishing the real
        // depth<->color baseline here double-corrects consumers.
        const rs2::video_stream_profile &reference_stream =
            color_stream ? color_stream : depth_stream;
        if (not reference_stream) {
          throw std::runtime_error(
              "neither color nor depth stream is available");
        }
        auto props = reference_stream.get_intrinsics();
        fillResp(response, props, reference_stream);
      } // End scope for my_dev lock

      VIAM_RESOURCE_LOG(debug) << "[get_properties] end";
      return response;
    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[get_properties] error: " << e.what();
      throw std::runtime_error("failed to create properties: " +
                               std::string(e.what()));
    }
  }
  std::vector<viam::sdk::GeometryConfig>
  get_geometries(const viam::sdk::ProtoStruct &extra) override {
    // Geometries are model-specific. The pose is the offset from the camera
    // reference origin (the RGB/color sensor) to the center of the bounding
    // box, and the box dimensions {x, y, z} match the physical module size in
    // mm. The camera optical frame is +X right, +Y down, +Z forward (out of
    // the lens), so x = width, y = height, z = depth.
    //
    // The origin is the color sensor because get_properties reports color
    // intrinsics and the images/derived poses callers work with are in the
    // color frame. Box dimensions and the depth-imager placement come from the
    // Intel RealSense D400-Series Datasheet (337029-005); the color sensor's
    // position is then derived from the color->depth extrinsics. Those are the
    // physical baseline librealsense reports for the model, used here only to
    // place the box at build time — not the identity extrinsics get_properties
    // publishes (see extrinsics::get_reported_extrinsics). The poses below are
    // constants; nothing here reads extrinsics at runtime.
    //   - Module dimensions: Table 3-43 (D415), Table 3-44 (D435/D435i).
    //   - The depth left imager is offset from the module centerline (the box
    //     center) per Table 4-15 (17.5 mm for D435/D435i, 20 mm for D415), on
    //     the -X side: the right imager is one stereo baseline (50 mm D435,
    //     55 mm D415) further along +X, which would fall off the module edge
    //     if the left imager were on the +X side.
    //   - The color sensor sits a further ~14.7 mm along -X from the depth left
    //     imager for D435/D435i (color->depth extrinsics ~= {-14.7, 0, 0} mm),
    //     so it is ~32.2 mm in -X from the box center, with no Y/Z component.
    //     The offset from the origin (color sensor) to the box center is
    //     therefore +X (positive): +32.2 mm for D435/D435i.
    //   - Z offset: the color sensor shares the depth origin's Z plane, which
    //     sits behind the front cover glass per Table 4-13 (4.2 mm for
    //     D435/D435i, 1.1 mm for D415). The glass is the front face of the box,
    //     so the box center is (depth/2 - recession) behind the origin. The
    //     imagers are on the housing's horizontal centerline (Figure 4-6), and
    //     the color->depth extrinsics have no Y component, so y = 0.
    // NOTE: when adding support for additional RealSense camera models,
    // update this switch accordingly.
    std::optional<std::string> model;
    if (device_) {
      auto my_dev = device_->synchronize();
      if (my_dev->device) {
        model = device::getCameraModel(my_dev->device);
      }
    }
    if (model && (*model == "D415")) {
      // D415: 99 (w) x 23 (h) x 20 (d) mm. Z = 20/2 - 1.1 = 8.9 mm.
      // X = 20 (centerline->left imager) + color->depth baseline.
      return {viam::sdk::GeometryConfig(viam::sdk::pose{35, 0, -8.9},
                                        viam::sdk::box({99, 23, 20}), "box")};
    }
    // Default: D435 / D435i geometry.
    // D435: 90 (w) x 25 (h) x 25 (d) mm. Z = 25/2 - 4.2 = 8.3 mm.
    // X = 17.5 (centerline->left imager) + 14.7 (color->depth baseline) = 32.2.
    return {viam::sdk::GeometryConfig(viam::sdk::pose{32.2, 0, -8.3},
                                      viam::sdk::box({90, 25, 25}), "box")};
  }

  static std::vector<std::string> validate(viam::sdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    // If sensors is provided, validate it, if not, we'll enable both color and
    // depth by default
    if (attrs.count("sensors")) {
      VIAM_SDK_LOG(info) << "[validate] Validating sensors config";
      if (not attrs["sensors"].is_a<viam::sdk::ProtoList>()) {
        VIAM_SDK_LOG(error) << "[validate] sensors is not a list";
        throw std::invalid_argument("sensors must be a list");
      }
      auto sensors_proto =
          attrs["sensors"].get_unchecked<viam::sdk::ProtoList>();

      if (sensors_proto.size() == 0 or sensors_proto.size() > 2) {
        VIAM_SDK_LOG(error)
            << "[validate] sensors field must contain 1 or 2 elements";
        throw std::invalid_argument(
            "sensors field must contain 1 or 2 elements");
      }
      if (sensors_proto.size() == 1) {
        VIAM_SDK_LOG(info) << "[validate] Validating that sensors is a string";
        if (not sensors_proto[0].is_a<std::string>()) {
          VIAM_SDK_LOG(error) << "[validate] sensors is not a string";
          throw std::invalid_argument("sensors must be a string");
        }
        std::string sensors_param =
            sensors_proto[0].get_unchecked<std::string>();
        if (sensors_param != "color" and sensors_param != "depth") {
          VIAM_SDK_LOG(error)
              << "[validate] sensors must be either 'color' or 'depth'";
          throw std::invalid_argument(
              "sensors must be either 'color' or 'depth'");
        }
      }
      if (sensors_proto.size() == 2) {
        VIAM_SDK_LOG(info) << "[validate] Validating that sensors is a string";
        if (not sensors_proto[0].is_a<std::string>() or
            not sensors_proto[1].is_a<std::string>()) {
          VIAM_SDK_LOG(error) << "[validate] sensors is not a string";
          throw std::invalid_argument("sensors must be a string");
        }
        std::string sensors_param_1 =
            sensors_proto[0].get_unchecked<std::string>();
        std::string sensors_param_2 =
            sensors_proto[1].get_unchecked<std::string>();
        if ((sensors_param_1 != "color" and sensors_param_1 != "depth") or
            (sensors_param_2 != "color" and sensors_param_2 != "depth")) {
          VIAM_SDK_LOG(error)
              << "[validate] sensors must be either 'color' or 'depth'";
          throw std::invalid_argument(
              "sensors must be either 'color' or 'depth'");
        }
        if (sensors_param_1 == sensors_param_2) {
          VIAM_SDK_LOG(error) << "[validate] sensors cannot contain duplicates";
          throw std::invalid_argument("sensors cannot contain duplicates");
        }
      }
    } else {
      VIAM_SDK_LOG(info) << "[validate] sensors not specified, will default to "
                            "[\"color\", \"depth\"]";
    }

    if (attrs.count("serial_number")) {
      VIAM_SDK_LOG(info)
          << "[validate] Validating that serial_number is a string";
      if (not attrs["serial_number"].is_a<std::string>()) {
        std::string error_message = "serial_number is not a string";
        VIAM_SDK_LOG(error) << "[validate] " << error_message;
        throw std::invalid_argument(error_message);
      }

      std::string serial = attrs["serial_number"].get_unchecked<std::string>();
      if (serial.empty()) {
        VIAM_SDK_LOG(info) << "[validate] serial_number is empty, "
                              "first available device will be used";
      }
    } else {
      VIAM_SDK_LOG(info) << "[validate] serial_number not specified, "
                            "first available device will be used";
    }

    if (attrs.count("width_px")) {
      if (not attrs["width_px"].is_a<double>()) {
        throw std::invalid_argument("width_px must be a double");
      }
      int width = attrs["width_px"].get_unchecked<double>();
      if (width <= 0) {
        throw std::invalid_argument("width_px must be positive");
      }
    }

    if (attrs.count("height_px")) {
      if (not attrs["height_px"].is_a<double>()) {
        throw std::invalid_argument("height_px must be a double");
      }
      int height = attrs["height_px"].get_unchecked<double>();
      if (height <= 0) {
        throw std::invalid_argument("height_px must be positive");
      }
    }

    if (attrs.count("laser_power")) {
      if (not attrs["laser_power"].is_a<double>()) {
        throw std::invalid_argument("laser_power must be a number (0..360)");
      }
      double lp = attrs["laser_power"].get_unchecked<double>();
      if (lp < 0 or lp > 360) {
        throw std::invalid_argument("laser_power must be in [0, 360]");
      }
    }
    if (attrs.count("depth_emitter_enabled") and
        not attrs["depth_emitter_enabled"].is_a<bool>()) {
      throw std::invalid_argument("depth_emitter_enabled must be a bool");
    }
    if (attrs.count("depth_visual_preset")) {
      if (not attrs["depth_visual_preset"].is_a<std::string>()) {
        throw std::invalid_argument("depth_visual_preset must be a string");
      }
      std::string vp =
          attrs["depth_visual_preset"].get_unchecked<std::string>();
      if (not device::visualPresetFromString(vp)) {
        std::string msg = "depth_visual_preset must be one of:";
        for (auto const &[name, _] : device::kVisualPresetNames) {
          msg += " ";
          msg += name;
        }
        throw std::invalid_argument(msg);
      }
    }
    if (attrs.count("depth_exposure_us")) {
      if (not attrs["depth_exposure_us"].is_a<double>()) {
        throw std::invalid_argument("depth_exposure_us must be a number");
      }
      double e = attrs["depth_exposure_us"].get_unchecked<double>();
      if (e < 1 or e > 200000) {
        throw std::invalid_argument(
            "depth_exposure_us must be in [1, 200000] (microseconds)");
      }
    }
    if (attrs.count("depth_auto_exposure") and
        not attrs["depth_auto_exposure"].is_a<bool>()) {
      throw std::invalid_argument("depth_auto_exposure must be a bool");
    }
    if (attrs.count("depth_gain")) {
      if (not attrs["depth_gain"].is_a<double>()) {
        throw std::invalid_argument("depth_gain must be a number");
      }
      double g = attrs["depth_gain"].get_unchecked<double>();
      if (g < 0 or g > 248) {
        throw std::invalid_argument("depth_gain must be in [0, 248]");
      }
    }

    if (attrs.count("align_color_depth")) {
      if (not attrs["align_color_depth"].is_a<bool>()) {
        throw std::invalid_argument("align_color_depth must be a bool");
      }
      bool align = attrs["align_color_depth"].get_unchecked<bool>();
      if (align and attrs.count("sensors")) {
        // Aligning requires both streams in the pipeline. The default
        // (no `sensors` field) enables both, so we only need to check the
        // case where the user explicitly listed sensors.
        auto sensors_list =
            attrs["sensors"].get_unchecked<viam::sdk::ProtoList>();
        bool has_color = false, has_depth = false;
        for (const auto &s : sensors_list) {
          if (not s.is_a<std::string>())
            continue;
          auto name = s.get_unchecked<std::string>();
          if (name == "color")
            has_color = true;
          if (name == "depth")
            has_depth = true;
        }
        if (not(has_color and has_depth)) {
          throw std::invalid_argument(
              "align_color_depth requires both 'color' and 'depth' in the "
              "sensors list");
        }
      }
    }

    // If we reach here, the serial number is valid
    return {};
  }
  static inline viam::sdk::Model model{"viam", "camera", "realsense"};

  // Handles device changes for this instance
  void handleDeviceChange(rs2::event_information &info) {
    VIAM_RESOURCE_LOG(info) << "[handleDeviceChange] Processing for serial: "
                            << config_->serial_number;

    deviceChangedCallback(info);
  }

  // Constructs the stale-frame watchdog. Called once on first successful
  // device initialization in assign_and_initialize_device. The watchdog
  // lives for the rest of the Realsense lifetime.
  void start_watchdog() {
    if (watchdog_) {
      return; // already running, nothing to do
    }
    auto restart_fn = [this]() -> bool {
      // Serialize the stop/start against do_command (e.g. firmware update),
      // which also rebuilds the pipeline.
      std::lock_guard<std::mutex> guard(do_command_mutex_);
      if (is_recovery_mode_.get() || !device_) {
        VIAM_RESOURCE_LOG(info)
            << "[watchdog] skip restart: recovery mode or no device";
        return false;
      }
      auto cfg = config_.get();
      VIAM_RESOURCE_LOG(warn)
          << "[watchdog] restarting rs2::pipeline after sustained "
          << "staleness on serial " << cfg.serial_number;
      if (!device_funcs_.stopDevice(device_, this->logger_)) {
        VIAM_RESOURCE_LOG(error) << "[watchdog] stopDevice failed";
        return false;
      }
      try {
        device_funcs_.startDevice(cfg.serial_number, device_, latest_frameset_,
                                  MAX_FRAME_AGE_MS, cfg, this->logger_);
        VIAM_RESOURCE_LOG(info)
            << "[watchdog] pipeline restart succeeded on " << cfg.serial_number;
        return true;
      } catch (const std::exception &e) {
        VIAM_RESOURCE_LOG(error)
            << "[watchdog] startDevice threw during restart: " << e.what();
        return false;
      }
    };
    auto recovery_check = [this]() { return is_recovery_mode_.get(); };
    // deviceChangedCallback nulls device_ when the camera is removed. The
    // watchdog checks this after recovery_check, so a DFU-mode device (which
    // leaves device_ null too) reports recovery rather than absence.
    auto device_absent = [this]() { return !device_; };
    auto reattach = [this]() -> bool {
      // do_command_mutex_ for the same reason restart_fn takes it: a firmware
      // update rebuilds the device and must not race this. Then attach_mutex_,
      // which is what deviceChangedCallback holds. Lock order is always
      // do_command_mutex_ -> attach_mutex_, never the reverse.
      std::lock_guard<std::mutex> guard(do_command_mutex_);
      std::lock_guard<std::mutex> attach_guard(attach_mutex_);
      if (device_) {
        return true; // the device-change callback re-attached first
      }
      auto device_list = realsense_ctx_->query_devices();
      if (device_list.size() == 0) {
        return false;
      }
      // Re-enters start_watchdog() on success, which no-ops because watchdog_
      // is already set (we are running on its thread).
      return assign_and_initialize_device(device_list);
    };
    // Read latest_frameset_ on every poll. get() copies under the
    // synchronized_value's lock (race-free wrt frameCallback); returns an
    // empty frameset until the first frame arrives.
    auto get_fs = [this]() -> rs2::frameset { return latest_frameset_.get(); };
    watchdog_ = std::make_unique<watchdog::StaleFrameWatchdog<rs2::frameset>>(
        std::move(get_fs), std::move(recovery_check), std::move(restart_fn),
        this->logger_, watchdog::StaleFrameWatchdog<rs2::frameset>::Tunables{},
        std::move(device_absent), std::move(reattach));
    VIAM_RESOURCE_LOG(info)
        << "[watchdog] constructed for serial " << config_->serial_number;
  }

private:
  boost::synchronized_value<RsResourceConfig> config_;
  std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> device_;
  boost::synchronized_value<rs2::frameset> latest_frameset_;
  std::shared_ptr<boost::synchronized_value<std::unordered_set<std::string>>>
      assigned_serials_;
  boost::synchronized_value<bool> physical_camera_assigned_;
  boost::synchronized_value<std::uint64_t> last_timestamp_warning_log_time_ms_;
  boost::synchronized_value<bool> is_recovery_mode_{false};
  std::shared_ptr<rs2::device> recovery_device_ptr_;

  // Mutex to serialize do_command calls and prevent concurrent execution
  // This is critical for long-running operations like firmware updates
  std::mutex do_command_mutex_;

  // Guards device attach/detach so librealsense's device-change thread and the
  // watchdog reattach path cannot both run assign_and_initialize_device for
  // the same serial. Kept separate from do_command_mutex_ so the callback
  // thread never blocks behind a multi-minute firmware update.
  // Lock order: do_command_mutex_ before attach_mutex_, never the reverse.
  std::mutex attach_mutex_;

  DeviceFunctions device_funcs_;
  std::shared_ptr<RealsenseContext<SynchronizedContextT>> realsense_ctx_;

  // Stale-frame watchdog: restarts the rs2::pipeline once a frame is stale
  // (older than stale_threshold_ms, default 10s) for 3 consecutive polls
  // (debounce) — see watchdog::Tunables. Declared LAST so its thread joins
  // first in ~Realsense.
  std::unique_ptr<watchdog::StaleFrameWatchdog<rs2::frameset>> watchdog_;

  void deviceChangedCallback(rs2::event_information &info) {
    std::cout << "[deviceChangedCallback] Device connection status changed"
              << std::endl;

    // Serialize against the watchdog's reattach path — both call
    // assign_and_initialize_device, and for an explicitly configured serial
    // the assigned_serials_ check does not stop two threads claiming the same
    // camera. See attach_mutex_ for the lock order.
    std::lock_guard<std::mutex> attach_guard(attach_mutex_);

    try {
      std::string const required_serial_number = config_->serial_number;
      { // Begin scope for current_device lock
        // Check if device_ is null before accessing it
        // This can happen after firmware update completes and device_ is
        // cleared
        if (device_) {
          auto current_device = device_->synchronize();
          if (info.was_removed(*current_device->device)) {
            std::cerr << "[deviceChangedCallback] Device removed: "
                      << current_device->serial_number << std::endl;
            { // Begin scope for serials_guard lock
              auto serials_guard = assigned_serials_->synchronize();
              serials_guard->erase(current_device->serial_number);
            } // End scope for serials_guard lock
            device_ = nullptr;
            physical_camera_assigned_ = false;
          }
        }
      } // End scope for current_device lock

      // Handling added devices, if any
      auto added_devices = info.get_new_devices();
      if (not physical_camera_assigned_) {
        if (assign_and_initialize_device(added_devices)) {
          physical_camera_assigned_ = true;
          std::cout << "[deviceChangedCallback] Device assigned successfully";
        } else {
          std::cerr << "[deviceChangedCallback] No matching device found";
        }
      }
    } catch (std::exception &e) {
      std::cerr << "[deviceChangedCallback] Exception occurred: " << e.what()
                << std::endl;
    }
  }

  viam::sdk::ProtoStruct handleFirmwareUpdate(const std::string &firmware_url) {
    // Note: This function is called from do_command which already holds the
    // mutex lock
    VIAM_RESOURCE_LOG(info)
        << "[handleFirmwareUpdate] Starting firmware update";
    VIAM_RESOURCE_LOG(info)
        << "[handleFirmwareUpdate] Firmware URL parameter: "
        << (firmware_url.empty() ? "(auto-detect)" : firmware_url);

    viam::sdk::ProtoStruct response;

    // Declare these outside try block so they're accessible in catch block
    std::shared_ptr<rs2::device> rs_device;
    std::string device_serial_number;

    try {

      // If not in recovery mode and no firmware URL was provided, check whether
      // the device already runs the recommended version before touching any
      // state — no callback clearing, no stream disruption needed.
      if (!is_recovery_mode_.get() && firmware_url.empty() && device_) {
        auto device_guard = device_->synchronize();
        auto pre_check_device = device_guard->device;
        std::string current, recommended;
        if (device_funcs_.getFirmwareVersions(pre_check_device, current,
                                              recommended)) {
          VIAM_RESOURCE_LOG(info)
              << "[handleFirmwareUpdate] Current firmware: " << current
              << ", recommended: " << recommended;
          if (current == recommended) {
            std::string msg =
                std::string(
                    "Firmware is already at the recommended version (") +
                current +
                "). No update needed. To force an update to a specific "
                "version, specify the firmware URL directly using: "
                "{\"update_firmware\": \"https://your-firmware-url.zip\"}. "
                "Find firmware URLs at: "
                "https://dev.realsenseai.com/docs/firmware-releases-d400";
            VIAM_RESOURCE_LOG(info) << "[handleFirmwareUpdate] " << msg;
            response["success"] = true;
            response["message"] = msg;
            return response;
          }
        }
      }

      // Temporarily clear the device change callback to prevent interference
      // It will be automatically restored when this scope exits
      realsense_ctx_->clearDevicesChangedCallback();
      auto callback_restorer = std::shared_ptr<void>(nullptr, [this](void *) {
        realsense_ctx_->restoreDevicesChangedCallback();
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Restored device change callback";
      });

      // Get the rs2::device and its serial number for tracking

      // Handle recovery mode vs normal mode devices differently
      if (is_recovery_mode_.get()) {
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Device is in recovery mode, using "
            << "stored recovery device pointer";

        if (!recovery_device_ptr_) {
          response["success"] = false;
          response["error"] = std::string(
              "Firmware update failed: No recovery device available");
          VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] Firmware update "
                                      "failed: No recovery device available";
          return response;
        }

        rs_device = recovery_device_ptr_;
        // Get the firmware update ID as the serial number for recovery devices
        device_serial_number = config_->serial_number;

        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Recovery device firmware update ID: "
            << device_serial_number;
      } else {
        // Normal mode device
        if (!device_) {
          response["success"] = false;
          response["error"] =
              std::string("Firmware update failed: No device available");
          VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] Firmware update "
                                      "failed: No device available";
          return response;
        }

        // Stop the device before firmware update
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Stopping device before update";
        if (!device_funcs_.stopDevice(device_, this->logger_)) {
          response["success"] = false;
          response["error"] = std::string("Firmware update failed: Failed to "
                                          "stop device before firmware update");
          return response;
        }

        {
          auto device_guard = device_->synchronize();
          rs_device = device_guard->device;
          device_serial_number = device_guard->serial_number;
        }

        if (!rs_device) {
          response["success"] = false;
          response["error"] =
              std::string("Firmware update failed: Device pointer is null");
          return response;
        }
      }

      VIAM_RESOURCE_LOG(info) << "[handleFirmwareUpdate] Updating device with "
                              << "serial number: " << device_serial_number;

      auto update_result = viam::realsense::firmware_update::updateFirmware(
          rs_device, device_serial_number, firmware_url, realsense_ctx_,
          logger_);

      // Check if firmware update succeeded
      if (update_result.first) {
        // Firmware was flashed — device will reboot and reconnect on its own.
        device_ = nullptr;
        recovery_device_ptr_ = nullptr;
        physical_camera_assigned_ = false;
        is_recovery_mode_ = false;

        {
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_serial_number);
        }

        response["success"] = true;
        response["message"] = update_result.second.count("message")
                                  ? update_result.second.at("message")
                                  : "Firmware update completed successfully. "
                                    "Device will reconnect shortly.";
      } else {
        // Failure - clear device state so it can be reassigned when it
        // reconnects
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Firmware update failed, clearing device "
               "state. Device will be reassigned when it reconnects (either in "
               "normal mode or recovery/DFU mode).";

        // Clear device state - device may exit DFU mode automatically (on some
        // errors) or stay in DFU/recovery mode (on others like power loss).
        // Either way, it will be reassigned when detected by
        // deviceChangedCallback
        device_ = nullptr;
        physical_camera_assigned_ = false;

        // Remove serial from assigned set so it can be reassigned
        {
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_serial_number);
        }

        // Return error
        response["success"] = false;
        if (update_result.second.count("error")) {
          response["error"] = update_result.second.at("error");
          // Extract error string for logging
          auto error_ptr = response["error"].get<std::string>();
          if (error_ptr) {
            VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] " << *error_ptr;
          } else {
            VIAM_RESOURCE_LOG(error)
                << "[handleFirmwareUpdate] Firmware update failed (error "
                   "message unavailable)";
          }
        } else {
          response["error"] = "Firmware update failed";
          VIAM_RESOURCE_LOG(error)
              << "[handleFirmwareUpdate] Firmware update failed";
        }
      }

    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] Error: " << e.what();

      // Clear device state on exception - device will reconnect automatically
      if (!is_recovery_mode_.get() && device_) {
        VIAM_RESOURCE_LOG(info) << "[handleFirmwareUpdate] Exception occurred, "
                                   "clearing device state. "
                                   "Device will be reassigned when it exits "
                                   "DFU mode and reconnects.";

        device_ = nullptr;
        physical_camera_assigned_ = false;

        // Remove serial from assigned set so it can be reassigned
        {
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_serial_number);
        }
      }

      response["success"] = false;
      response["error"] = std::string("Firmware update failed: ") + e.what();
    }

    // Callback will be automatically restored by callback_restorer destructor

    return response;
  }

  template <typename DeviceListT>
  bool assign_and_initialize_device(DeviceListT const &device_list) {
    if (physical_camera_assigned_) {
      std::cerr << "[assign_and_initialize_device] Camera is already assigned"
                << std::endl;
      return true;
    }
    std::string requested_serial_number = config_->serial_number;
    VIAM_RESOURCE_LOG(info) << "[assign_and_initialize_device] starting device "
                            << config_->serial_number;
    // This will the initial set of connected devices (i.e. the devices that
    // were connected before the callback was set)
    VIAM_RESOURCE_LOG(info)
        << "[assign_and_initialize_device] Number of connected devices: "
        << device_list.size() << "\n";

    int devCount = device_list.size();
    for (int i = 0; i < devCount; i++) {
      try {
        VIAM_RESOURCE_LOG(debug) << "[assign_and_initialize_device] Attempting "
                                    "to access device at index "
                                 << i;
        auto dev = device_list[i];
        VIAM_RESOURCE_LOG(debug) << "[assign_and_initialize_device] "
                                    "Successfully accessed device at index "
                                 << i;

        device_funcs_.printDeviceInfo(dev, this->logger_);

        auto dev_ptr = std::make_shared<std::decay_t<decltype(dev)>>(dev);

        // Check if device is in recovery/DFU mode
        bool is_recovery = dev.template is<rs2::update_device>();
        std::string connected_device_serial_number;

        if (is_recovery) {
          // Recovery mode devices only have firmware update ID
          if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
            connected_device_serial_number =
                dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
            VIAM_RESOURCE_LOG(warn)
                << "[assign_and_initialize_device] Device at index " << i
                << " is in recovery/DFU mode with update ID: "
                << connected_device_serial_number;
          } else {
            VIAM_RESOURCE_LOG(error)
                << "[assign_and_initialize_device] Recovery device at index "
                << i << " does not support firmware update ID";
            continue;
          }
        } else {
          // Normal mode devices have serial number
          if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            connected_device_serial_number =
                dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          } else {
            VIAM_RESOURCE_LOG(error)
                << "[assign_and_initialize_device] Normal device at index " << i
                << " does not support serial number";
            continue;
          }
        }

        VIAM_RESOURCE_LOG(info)
            << "[assign_and_initialize_device] trying connecting to device: "
            << connected_device_serial_number;

        // Atomically check and insert serial number
        { // Begin scope for serials_guard lock
          auto serials_guard = assigned_serials_->synchronize();
          if ((requested_serial_number.empty() &&
               serials_guard->count(connected_device_serial_number) == 0) ||
              (requested_serial_number == connected_device_serial_number)) {
            VIAM_RESOURCE_LOG(info)
                << "[assign_and_initialize_device] grabbing device: "
                << connected_device_serial_number;

            serials_guard->insert(connected_device_serial_number);
          } else {
            VIAM_RESOURCE_LOG(info)
                << "[assign_and_initialize_device] Could not grab device: "
                << connected_device_serial_number;
            continue;
          }
        } // End scope for serials_guard lock

        // Handle recovery mode devices differently - they can't stream
        if (is_recovery) {
          VIAM_RESOURCE_LOG(warn)
              << "[assign_and_initialize_device] Device "
              << connected_device_serial_number
              << " is in recovery/DFU mode. Skipping streaming initialization. "
              << "Only firmware updates are supported for this device.";
          is_recovery_mode_ = true;
          recovery_device_ptr_ =
              dev_ptr; // Store the device pointer for firmware updates
          physical_camera_assigned_ = true;
          return true;
        }

        // Normal device initialization with streaming support
        VIAM_RESOURCE_LOG(info)
            << "[assign_and_initialize_device] calling createDevice for: "
            << connected_device_serial_number;
        realsense::RsResourceConfig config_copy = config_.get();
        device_ = device_funcs_.createDevice(connected_device_serial_number,
                                             dev_ptr, SUPPORTED_CAMERA_MODELS,
                                             config_copy, this->logger_);
        BOOST_ASSERT(device_ != nullptr);

        VIAM_RESOURCE_LOG(info)
            << "[assign_and_initialize_device] calling startDevice for: "
            << connected_device_serial_number;
        device_funcs_.startDevice(connected_device_serial_number, device_,
                                  latest_frameset_, MAX_FRAME_AGE_MS,
                                  config_copy, this->logger_);
        physical_camera_assigned_ = true;
        is_recovery_mode_ = false;
        start_watchdog();
        return true;
      } catch (const std::exception &e) {
        VIAM_RESOURCE_LOG(error) << "[assign_and_initialize_device] Failed to "
                                    "access/initialize device at index "
                                 << i << ": " << e.what();
        // Continue trying other devices
        continue;
      }
    }
    return false;
  }

  RsResourceConfig configure(viam::sdk::Dependencies dependencies,
                             viam::sdk::ResourceConfig configuration) {
    auto attrs = configuration.attributes();

    /*
     * Validation already checks that serial_number exists, it is a string
     * and it is not empty, so we can safely extract it without additional
     * checks.
     */
    std::string serial;
    if (attrs.count("serial_number")) {
      serial = attrs["serial_number"].get_unchecked<std::string>();
    }

    // Default to both color and depth sensors if not specified
    std::vector<sensors::SensorType> sensors;
    if (attrs.count("sensors")) {
      auto sensors_list =
          attrs["sensors"].get_unchecked<viam::sdk::ProtoList>();
      for (const auto &sensor : sensors_list) {
        auto sensor_type = sensors::string_to_sensor_type(
            sensor.get_unchecked<std::string>(), this->logger_);
        if (sensor_type == sensors::SensorType::unknown) {
          throw std::runtime_error("Invalid sensor type: " +
                                   sensor.get_unchecked<std::string>());
        }
        sensors.push_back(sensor_type);
      }
    } else {
      // Default: enable both color and depth sensors
      sensors.push_back(sensors::SensorType::color);
      sensors.push_back(sensors::SensorType::depth);
    }
    std::optional<int> width;
    if (attrs.count("width_px")) {
      width = static_cast<int>(attrs["width_px"].get_unchecked<double>());
    }
    std::optional<int> height;
    if (attrs.count("height_px")) {
      height = static_cast<int>(attrs["height_px"].get_unchecked<double>());
    }
    bool align_color_depth = false;
    if (attrs.count("align_color_depth")) {
      align_color_depth = attrs["align_color_depth"].get_unchecked<bool>();
    }
    auto native_config =
        realsense::RsResourceConfig(serial, configuration.name(), sensors,
                                    width, height, align_color_depth);

    if (attrs.count("laser_power")) {
      native_config.laser_power = attrs["laser_power"].get_unchecked<double>();
    }
    if (attrs.count("depth_emitter_enabled")) {
      native_config.depth_emitter_enabled =
          attrs["depth_emitter_enabled"].get_unchecked<bool>();
    }
    if (attrs.count("depth_visual_preset")) {
      native_config.depth_visual_preset =
          attrs["depth_visual_preset"].get_unchecked<std::string>();
    }
    if (attrs.count("depth_exposure_us")) {
      native_config.depth_exposure_us =
          attrs["depth_exposure_us"].get_unchecked<double>();
    }
    if (attrs.count("depth_auto_exposure")) {
      native_config.depth_auto_exposure =
          attrs["depth_auto_exposure"].get_unchecked<bool>();
    }
    if (attrs.count("depth_gain")) {
      native_config.depth_gain = attrs["depth_gain"].get_unchecked<double>();
    }

    return native_config;
  }

public:
  // The DeviceFunctions the module runs with. Public so tests can drive the
  // real device lifecycle against a software device.
  static DeviceFunctions createDefaultDeviceFunctions() {
    return DeviceFunctions{
        .stopDevice =
            [](std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>> &device,
               viam::sdk::LogSource &logger) {
              return device::stopDevice(device, logger);
            },
        .destroyDevice =
            [](std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>> &device,
               viam::sdk::LogSource &logger) {
              return device::destroyDevice(device, logger);
            },
        .printDeviceInfo =
            [](const auto &dev, viam::sdk::LogSource &logger) {
              device::printDeviceInfo(dev, logger);
            },
        .createDevice =
            [](std::string const &serial, std::shared_ptr<rs2::device> dev_ptr,
               std::unordered_set<std::string> const &supported_models,
               realsense::RsResourceConfig const &config,
               viam::sdk::LogSource &logger) {
              return device::createDevice<
                  realsense::RsResourceConfig, device::ViamRSDevice<>,
                  rs2::device, rs2::config, rs2::color_sensor,
                  rs2::depth_sensor, rs2::video_stream_profile>(
                  serial, dev_ptr, supported_models, config, logger);
            },
        .startDevice =
            [](const std::string &serial,
               std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>> &device,
               boost::synchronized_value<rs2::frameset> &latest_frameset,
               std::uint64_t maxFrameSetFrameMs,
               realsense::RsResourceConfig const &viamConfig,
               viam::sdk::LogSource &logger) {
              return device::startDevice(serial, device, latest_frameset,
                                         maxFrameSetFrameMs, viamConfig,
                                         logger);
            },
        .getFirmwareVersions = [](std::shared_ptr<rs2::device> dev,
                                  std::string &current,
                                  std::string &recommended) -> bool {
          if (!dev || !dev->supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) ||
              !dev->supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
            return false;
          }
          current = dev->get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
          recommended =
              dev->get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
          return true;
        }};
  };
};
}; // namespace realsense
