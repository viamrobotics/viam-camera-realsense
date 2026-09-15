#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/log/logging.hpp>

#include "device.hpp"
#include "log_capture.hpp"
#include "realsense.hpp"
#include "sensors.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrictMock;

namespace realsense {
namespace device {
template <typename DeviceT, typename PipeT, typename AligntT, typename ConfigT>
std::ostream &
operator<<(std::ostream &os,
           const ViamRSDevice<DeviceT, PipeT, AligntT, ConfigT> &device) {
  os << "ViamRSDevice{serial: " << device.serial_number
     << ", started: " << device.started << "}";
  return os;
}
} // namespace device
} // namespace realsense

namespace realsense {
namespace device {
namespace test {

// Mock classes for testing
class MockDevice : public rs2::device {
public:
  MOCK_METHOD(bool, supports, (rs2_camera_info), (const));
  MOCK_METHOD(const char *, get_info, (rs2_camera_info), (const));
  // Note: query_sensors() not mocked - only SimpleDevice has it for template
  // tests
};

// MockSensor: GMock mock for EXPECT_CALL tests with option support
class MockSensor {
public:
  MOCK_METHOD(bool, supports, (rs2_option), (const));
  MOCK_METHOD(void, set_option, (rs2_option, float));

  // Template method for sensor type checking (used by enableGlobalTimestamp,
  // etc.)
  template <typename T> bool is() const {
    return std::is_same<T, rs2::color_sensor>::value ? is_color_ : is_depth_;
  }

  void set_sensor_type(bool is_color, bool is_depth) {
    is_color_ = is_color;
    is_depth_ = is_depth;
  }

private:
  bool is_color_ = false;
  bool is_depth_ = false;
};

// MockVideoStreamProfile: GMock-based mock for profile comparison tests
class MockVideoStreamProfile {
public:
  MOCK_METHOD(int, width, (), (const));
  MOCK_METHOD(int, height, (), (const));
  MOCK_METHOD(int, fps, (), (const));
  // Note: format() and stream_index() not mocked - not used in tests
};

// SimpleVideoStreamProfile: Non-GMock simple mock for value-based tests
class SimpleVideoStreamProfile {
public:
  SimpleVideoStreamProfile() = default;
  SimpleVideoStreamProfile(rs2_format format, int width, int height, int fps,
                           int stream_index)
      : format_(format), width_(width), height_(height), fps_(fps),
        stream_index_(stream_index) {}

  rs2_format format() const { return format_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int fps() const { return fps_; }
  int stream_index() const { return stream_index_; }

  rs2_format format_ = RS2_FORMAT_RGB8;
  int width_ = 640;
  int height_ = 480;
  int fps_ = 30;
  int stream_index_ = 0;
};

// SimpleStreamProfile: Wrapper that can convert to SimpleVideoStreamProfile
class SimpleStreamProfile {
public:
  // All fake profiles are video profiles.
  template <typename T> bool is() const { return true; }

  template <typename T> T as() const {
    return T{format_, width_, height_, fps_, stream_index_};
  }

  rs2_format format_;
  int width_;
  int height_;
  int fps_;
  int stream_index_;
};

// SimpleSensorImpl: GMock implementation for SimpleSensor
class SimpleSensorImpl {
public:
  MOCK_METHOD(bool, supports, (rs2_option), (const));
  MOCK_METHOD(void, set_option, (rs2_option, float));
};

// SimpleSensor: Copyable wrapper for template function tests (wraps shared_ptr
// to GMock)
class SimpleSensor {
public:
  SimpleSensor() : impl_(std::make_shared<SimpleSensorImpl>()) {}

  void set_sensor_type(bool is_color, bool is_depth) {
    is_color_ = is_color;
    is_depth_ = is_depth;
  }

  template <typename T> bool is() const {
    return std::is_same<T, rs2::color_sensor>::value ? is_color_ : is_depth_;
  }

  bool supports(rs2_option option) const { return impl_->supports(option); }

  void set_option(rs2_option option, float value) {
    impl_->set_option(option, value);
  }

  std::vector<SimpleStreamProfile> get_stream_profiles() const {
    return stream_profiles_;
  }

  void set_stream_profiles(const std::vector<SimpleStreamProfile> &profiles) {
    stream_profiles_ = profiles;
  }

  // Access to the GMock implementation for setting expectations
  std::shared_ptr<SimpleSensorImpl> mock() const { return impl_; }

private:
  bool is_color_ = false;
  bool is_depth_ = false;
  std::vector<SimpleStreamProfile> stream_profiles_;
  std::shared_ptr<SimpleSensorImpl> impl_;
};

// SimpleDevice: Device wrapper for template function tests
class SimpleDevice {
public:
  std::vector<SimpleSensor> query_sensors() const { return sensors_; }

  void set_sensors(const std::vector<SimpleSensor> &sensors) {
    sensors_ = sensors;
  }

  bool supports(rs2_camera_info info) const {
    return info == RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR and
           usb_type_.has_value();
  }

  const char *get_info(rs2_camera_info) const { return usb_type_->c_str(); }

  void set_usb_type(const std::string &usb_type) { usb_type_ = usb_type; }

private:
  std::vector<SimpleSensor> sensors_;
  std::optional<std::string> usb_type_;
};

class MockConfig {
public:
  MOCK_METHOD(void, enable_stream,
              (rs2_stream, int, int, int, rs2_format, int));
  MOCK_METHOD(void, enable_device, (const std::string &));
};

// SimpleConfig: Non-GMock config for value-based tests (used in template
// functions)
class SimpleConfig {
public:
  void enable_stream(rs2_stream stream, int stream_index, int width, int height,
                     rs2_format format, int fps) {
    if (enable_stream_func_) {
      enable_stream_func_(stream, stream_index, width, height, format, fps);
    }
  }

  void set_enable_stream_func(
      std::function<void(rs2_stream, int, int, int, rs2_format, int)> func) {
    enable_stream_func_ = func;
  }

private:
  std::function<void(rs2_stream, int, int, int, rs2_format, int)>
      enable_stream_func_;
};

class MockPipeline {
public:
  MOCK_METHOD(void, start, (const rs2::config &));
  MOCK_METHOD(void, start,
              (const rs2::config &, std::function<void(const rs2::frame &)>));
  MOCK_METHOD(void, stop, ());
};

class MockAlign {
public:
  MockAlign(rs2_stream) {} // Constructor for align
};

// Test fixture
class DeviceTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Setup common test data
    serial_number_ = "test_device_123456";
    supported_models_ = {"D415", "D435", "D435i", "D455"};

    // Setup mock device
    mock_device_ = std::make_shared<MockDevice>();
    mock_config_ = std::make_shared<MockConfig>();
    mock_pipeline_ = std::make_shared<MockPipeline>();

    // Setup test config with proper constructor parameters
    test_config_ =
        RsResourceConfig(serial_number_, // serial_number
                         "camera1",      // name
                         std::vector<realsense::sensors::SensorType>{
                             realsense::sensors::SensorType::color,
                             realsense::sensors::SensorType::depth}, // sensors
                         std::optional<int>{640},                    // width
                         std::optional<int>{480}                     // height
        );
  }

  std::string serial_number_;
  std::unordered_set<std::string> supported_models_;
  std::shared_ptr<MockDevice> mock_device_;
  std::shared_ptr<MockConfig> mock_config_;
  std::shared_ptr<MockPipeline> mock_pipeline_;
  RsResourceConfig test_config_;
};

class RealsenseTestEnvironment : public ::testing::Environment {
public:
  void SetUp() override {
    // Create the instance here, before any tests run
    instance_ = std::make_unique<viam::sdk::Instance>();
  }

  void TearDown() override {
    // Clean up the instance
    instance_.reset();
  }

private:
  std::unique_ptr<viam::sdk::Instance> instance_;
};

// Test getCameraModel function
TEST_F(DeviceTest, GetCameraModel_ValidDevice_ReturnsModel) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Intel RealSense D435"));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), "D435");
}

TEST_F(DeviceTest, GetCameraModel_D415Device_ReturnsD415) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Intel RealSense D415"));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), "D415");
}

TEST_F(DeviceTest, GetCameraModel_UnsupportedDevice_ReturnsNullopt) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(false));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  EXPECT_FALSE(result.has_value());
}

TEST_F(DeviceTest, GetCameraModel_InvalidNameFormat_ReturnsNullopt) {
  // Setup expectations
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Invalid Name"));

  // Execute
  auto result = getCameraModel(mock_device_);

  // Verify
  EXPECT_FALSE(result.has_value());
}

// Test printDeviceInfo function
TEST_F(DeviceTest, PrintDeviceInfo_ValidDevice_LogsInfo) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Setup expectations for supported info types
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_NAME))
      .WillOnce(Return("Intel RealSense D435"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))
      .WillOnce(Return("123456789"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_PRODUCT_LINE))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_PRODUCT_LINE))
      .WillOnce(Return("D400 Series"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_PRODUCT_ID))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_PRODUCT_ID))
      .WillOnce(Return("0B07"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("3.1"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.12.7.100"));

  EXPECT_CALL(*mock_device_,
              supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_,
              get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.12.7.100"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID))
      .WillOnce(Return("5.12.7.100"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_PHYSICAL_PORT))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_PHYSICAL_PORT))
      .WillOnce(Return("USB 3.1 Port"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_DEBUG_OP_CODE))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE))
      .WillOnce(Return("Debug Op Code Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_ADVANCED_MODE))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_ADVANCED_MODE))
      .WillOnce(Return("Advanced Mode Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return("Camera Locked Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER))
      .WillOnce(Return("ASIC Serial Number Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_DFU_DEVICE_PATH))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_DFU_DEVICE_PATH))
      .WillOnce(Return("DFU Device Path Info"));

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_IP_ADDRESS))
      .WillOnce(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_IP_ADDRESS))
      .WillOnce(Return("IP Address Info"));

  // Execute - should not throw and should log device info
  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  // Verify logs were produced
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "printDeviceInfo should produce info logs";

  // Verify no errors were logged
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0) << "Should not log errors for valid device";

  // Verify device info is in the logs
  bool found_device_info = false;
  for (const auto &log : all_logs) {
    if (log.message.find("DeviceInfo") != std::string::npos ||
        log.message.find("Intel RealSense D435") != std::string::npos ||
        log.message.find("123456789") != std::string::npos) {
      found_device_info = true;
      break;
    }
  }
  EXPECT_TRUE(found_device_info) << "Should log device information";
}

// Helper: default all RS2_CAMERA_INFO_* supports() calls to false.
// Tests override specific fields as needed.
static void setAllDeviceInfoUnsupported(MockDevice &mock) {
  EXPECT_CALL(mock, supports(::testing::_))
      .Times(::testing::AnyNumber())
      .WillRepeatedly(Return(false));
}

// Helper: check if any log in a list contains a substring.
static bool
anyLogContains(const std::vector<test_utils::CapturedLogRecord> &logs,
               const std::string &substr) {
  return std::any_of(logs.begin(), logs.end(),
                     [&](const test_utils::CapturedLogRecord &r) {
                       return r.message.find(substr) != std::string::npos;
                     });
}

TEST_F(DeviceTest, PrintDeviceInfo_USB2Connection_LogsWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("2.1"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  ASSERT_EQ(warn_logs.size(), 1)
      << "Should log exactly one warning for USB 2.x connection";
  EXPECT_NE(warn_logs[0].message.find("USB 2.1"), std::string::npos)
      << "Warning should mention the USB version";
  EXPECT_NE(warn_logs[0].message.find("USB 3.x is recommended"),
            std::string::npos)
      << "Warning should recommend USB 3.x";
}

TEST_F(DeviceTest, PrintDeviceInfo_USB3Connection_NoWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("3.1"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  EXPECT_EQ(warn_logs.size(), 0)
      << "Should not warn when connected via USB 3.x";
}

TEST_F(DeviceTest, PrintDeviceInfo_USBDescriptorUnsupported_NoWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);
  // USB type descriptor not supported — should not warn

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  EXPECT_EQ(warn_logs.size(), 0)
      << "Should not warn when USB type descriptor is not supported";
}

TEST_F(DeviceTest, PrintDeviceInfo_USB3Connection_LogsConfirmation) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("3.2"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto info_logs = log_capture.get_logs_by_level(viam::sdk::log_level::info);
  EXPECT_TRUE(anyLogContains(info_logs, "connected via USB 3.2"))
      << "Should log an info confirmation of the USB 3 connection";
  EXPECT_TRUE(anyLogContains(info_logs, "Full resolution"))
      << "Confirmation should state full capabilities are available";
}

TEST_F(DeviceTest, PrintDeviceInfo_USBTypeUnknown_NoWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  // Some platforms/firmware report "Unknown" — must not trigger the USB 2
  // warning (telling the user to swap cables) nor the USB 3 confirmation.
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("Unknown"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  EXPECT_EQ(warn_logs.size(), 0)
      << "Should not warn when the USB type descriptor is unrecognized";

  auto info_logs = log_capture.get_logs_by_level(viam::sdk::log_level::info);
  EXPECT_FALSE(anyLogContains(info_logs, "Full resolution"))
      << "Should not claim USB 3 capabilities for an unrecognized descriptor";
}

// Test getUsbConnectionType helper
TEST_F(DeviceTest, GetUsbConnectionType_USB3_ReturnsUsb3) {
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("3.2"));

  auto result = getUsbConnectionType(*mock_device_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, UsbConnectionType::usb3);
  EXPECT_EQ(result->second, "3.2");
}

TEST_F(DeviceTest, GetUsbConnectionType_USB2_ReturnsUsb2) {
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("2.1"));

  auto result = getUsbConnectionType(*mock_device_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, UsbConnectionType::usb2);
  EXPECT_EQ(result->second, "2.1");
}

TEST_F(DeviceTest, GetUsbConnectionType_UnknownDescriptor_ReturnsUnknown) {
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("Unknown"));

  auto result = getUsbConnectionType(*mock_device_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, UsbConnectionType::unknown);
  EXPECT_EQ(result->second, "Unknown");
}

TEST_F(DeviceTest, GetUsbConnectionType_EmptyDescriptor_ReturnsUnknown) {
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(""));

  auto result = getUsbConnectionType(*mock_device_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, UsbConnectionType::unknown);
}

TEST_F(DeviceTest, GetUsbConnectionType_Unsupported_ReturnsNullopt) {
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(false));

  auto result = getUsbConnectionType(*mock_device_);
  EXPECT_FALSE(result.has_value());
}

TEST_F(DeviceTest, PrintDeviceInfo_OutdatedFirmware_LogsWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.15.1.55"));

  EXPECT_CALL(*mock_device_,
              supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_,
              get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.17.0.10"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  ASSERT_EQ(warn_logs.size(), 1)
      << "Should log exactly one warning for outdated firmware";
  EXPECT_NE(warn_logs[0].message.find("5.15.1.55"), std::string::npos)
      << "Warning should mention current firmware version";
  EXPECT_NE(warn_logs[0].message.find("5.17.0.10"), std::string::npos)
      << "Warning should mention recommended firmware version";
}

TEST_F(DeviceTest, PrintDeviceInfo_MatchingFirmware_NoWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.17.0.10"));

  EXPECT_CALL(*mock_device_,
              supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_,
              get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.17.0.10"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  EXPECT_EQ(warn_logs.size(), 0)
      << "Should not warn when firmware matches recommended";
}

TEST_F(DeviceTest,
       PrintDeviceInfo_FirmwareVersionOnly_NoRecommended_NoWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  // Only firmware version supported, no recommended version
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.15.1.55"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto warn_logs = log_capture.get_warning_logs();
  EXPECT_EQ(warn_logs.size(), 0)
      << "Should not warn when recommended firmware version is not available";
}

TEST_F(DeviceTest, PrintDeviceInfo_CameraLocked_LogsInfo) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return("YES"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  // Camera locked is info-level (most cameras ship locked by default)
  auto warn_logs = log_capture.get_warning_logs();
  EXPECT_EQ(warn_logs.size(), 0) << "Camera locked should not produce a warn";

  auto info_logs = log_capture.get_logs_by_level(viam::sdk::log_level::info);
  EXPECT_TRUE(anyLogContains(info_logs, "locked"))
      << "Should log info about camera being locked";
  EXPECT_TRUE(anyLogContains(info_logs, "Advanced mode"))
      << "Should mention advanced mode in locked info";
}

TEST_F(DeviceTest, PrintDeviceInfo_CameraNotLocked_NoLockedMessage) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return("NO"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  auto all_logs = log_capture.get_records();
  EXPECT_FALSE(anyLogContains(all_logs, "Camera is locked"))
      << "Should not log locked message when camera is not locked";
}

TEST_F(DeviceTest, PrintDeviceInfo_MultipleIssues_LogsAllWarnings) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  setAllDeviceInfoUnsupported(*mock_device_);

  // USB 2.x
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
      .WillRepeatedly(Return("2.1"));

  // Outdated firmware
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.15.1.55"));
  EXPECT_CALL(*mock_device_,
              supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_,
              get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION))
      .WillRepeatedly(Return("5.17.0.10"));

  // Camera locked (info-level, not warn)
  EXPECT_CALL(*mock_device_, supports(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_device_, get_info(RS2_CAMERA_INFO_CAMERA_LOCKED))
      .WillRepeatedly(Return("YES"));

  EXPECT_NO_THROW(printDeviceInfo(*mock_device_, logger));

  // Should have exactly 2 warnings (USB + firmware), not 3
  auto warn_logs = log_capture.get_warning_logs();
  ASSERT_EQ(warn_logs.size(), 2)
      << "Should log two warnings: USB 2.x and outdated firmware";

  // Order-independent checks
  EXPECT_TRUE(anyLogContains(warn_logs, "USB 2.1"))
      << "Should warn about USB 2.x connection";
  EXPECT_TRUE(anyLogContains(warn_logs, "5.15.1.55"))
      << "Should warn about outdated firmware";

  // Camera locked should be in info logs
  auto info_logs = log_capture.get_logs_by_level(viam::sdk::log_level::info);
  EXPECT_TRUE(anyLogContains(info_logs, "locked"))
      << "Should log info about camera being locked";
}

// Test SensorTypeTraits
TEST_F(DeviceTest, SensorTypeTraits_ColorSensor_CorrectValues) {
  EXPECT_EQ(SensorTypeTraits<rs2::color_sensor>::stream_type, RS2_STREAM_COLOR);
  EXPECT_EQ(SensorTypeTraits<rs2::color_sensor>::format_type, RS2_FORMAT_RGB8);
}

TEST_F(DeviceTest, SensorTypeTraits_DepthSensor_CorrectValues) {
  EXPECT_EQ(SensorTypeTraits<rs2::depth_sensor>::stream_type, RS2_STREAM_DEPTH);
  EXPECT_EQ(SensorTypeTraits<rs2::depth_sensor>::format_type, RS2_FORMAT_Z16);
}

// Test checkIfMatchingColorDepthProfiles
TEST_F(DeviceTest,
       CheckIfMatchingColorDepthProfiles_MatchingProfiles_ReturnsTrue) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockVideoStreamProfile color_profile;
  MockVideoStreamProfile depth_profile;

  // Setup matching profiles
  EXPECT_CALL(color_profile, width()).WillRepeatedly(Return(640));
  EXPECT_CALL(color_profile, height()).WillRepeatedly(Return(480));
  EXPECT_CALL(color_profile, fps()).WillRepeatedly(Return(30));

  EXPECT_CALL(depth_profile, width()).WillRepeatedly(Return(640));
  EXPECT_CALL(depth_profile, height()).WillRepeatedly(Return(480));
  EXPECT_CALL(depth_profile, fps()).WillRepeatedly(Return(30));

  // Execute
  bool result =
      checkIfMatchingColorDepthProfiles(color_profile, depth_profile, logger);

  // Verify function result
  EXPECT_TRUE(result);

  // Verify info log with resolution details
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log matching profile info";

  // Verify the log contains resolution/fps info
  bool found_profile_info = false;
  for (const auto &log : all_logs) {
    if (log.message.find("640") != std::string::npos &&
        log.message.find("480") != std::string::npos &&
        log.message.find("30") != std::string::npos) {
      found_profile_info = true;
      break;
    }
  }
  EXPECT_TRUE(found_profile_info) << "Should log profile dimensions";
}

TEST_F(DeviceTest,
       CheckIfMatchingColorDepthProfiles_DifferentResolution_ReturnsFalse) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockVideoStreamProfile color_profile;
  MockVideoStreamProfile depth_profile;

  // Setup non-matching profiles
  EXPECT_CALL(color_profile, width()).WillRepeatedly(Return(640));
  EXPECT_CALL(color_profile, height()).WillRepeatedly(Return(480));
  EXPECT_CALL(color_profile, fps()).WillRepeatedly(Return(30));

  EXPECT_CALL(depth_profile, width()).WillRepeatedly(Return(1280));
  EXPECT_CALL(depth_profile, height()).WillRepeatedly(Return(720));
  EXPECT_CALL(depth_profile, fps()).WillRepeatedly(Return(30));

  // Execute
  bool result =
      checkIfMatchingColorDepthProfiles(color_profile, depth_profile, logger);

  // Verify function result
  EXPECT_FALSE(result);

  // Verify NO logs are produced for non-matching profiles
  auto all_logs = log_capture.get_records();
  EXPECT_EQ(all_logs.size(), 0) << "Should not log for non-matching profiles";
}

// Test ViamRSDevice structure
TEST_F(DeviceTest, ViamRSDevice_DefaultConstruction_ValidState) {
  ViamRSDevice<> device;

  // Default values should be reasonable
  EXPECT_TRUE(device.serial_number.empty());
  EXPECT_FALSE(device.started);
  EXPECT_EQ(device.device, nullptr);
  EXPECT_EQ(device.pipe, nullptr);
  EXPECT_EQ(device.point_cloud_filter, nullptr);
  EXPECT_EQ(device.align, nullptr);
  EXPECT_EQ(device.config, nullptr);
}

TEST_F(DeviceTest, ViamRSDevice_SetValues_StateUpdated) {
  ViamRSDevice<> device;

  device.serial_number = "test123";
  device.started = true;
  device.device = mock_device_;

  EXPECT_EQ(device.serial_number, "test123");
  EXPECT_TRUE(device.started);
  EXPECT_EQ(device.device, mock_device_);
}

// Test destroyDevice function
TEST_F(DeviceTest, DestroyDevice_ValidDevice_ReturnsTrue) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Create a test device
  auto device = std::make_shared<boost::synchronized_value<ViamRSDevice<>>>();
  {
    auto dev_guard = device->synchronize();
    dev_guard->serial_number = "test123";
    dev_guard->started = false;
    dev_guard->pipe = std::make_shared<rs2::pipeline>();
    dev_guard->device = std::make_shared<rs2::device>();
    dev_guard->config = std::make_shared<rs2::config>();
    dev_guard->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    dev_guard->point_cloud_filter = std::make_shared<PointCloudFilter>();
  }

  // Execute
  bool result = destroyDevice(device, logger);

  // Verify function result
  EXPECT_TRUE(result);
  EXPECT_EQ(device, nullptr);

  // Verify logs - device not started, so should NOT have "stopping pipe" log
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log destruction process";

  // Verify no errors
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0) << "Should not log errors for valid device";

  // Verify expected INFO logs (destroying, clearing resources, destroyed)
  // Should NOT contain "stopping pipe" since started=false
  bool found_destroying = false;
  bool found_stopping = false;
  bool found_destroyed = false;

  for (const auto &log : all_logs) {
    if (log.message.find("destroying") != std::string::npos &&
        log.message.find("test123") != std::string::npos) {
      found_destroying = true;
    }
    if (log.message.find("stopping pipe") != std::string::npos) {
      found_stopping = true;
    }
    if (log.message.find("device destroyed") != std::string::npos) {
      found_destroyed = true;
    }
  }

  EXPECT_TRUE(found_destroying) << "Should log 'destroying device'";
  EXPECT_FALSE(found_stopping)
      << "Should NOT log 'stopping pipe' when device not started";
  EXPECT_TRUE(found_destroyed) << "Should log 'device destroyed'";
}

TEST_F(DeviceTest, StopDevice_NotStartedDevice_ReturnsTrueIdempotent) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // A device that is not currently streaming.
  auto device = std::make_shared<boost::synchronized_value<ViamRSDevice<>>>();
  {
    auto dev_guard = device->synchronize();
    dev_guard->serial_number = "test123";
    dev_guard->started = false;
    dev_guard->pipe = std::make_shared<rs2::pipeline>();
  }

  // stopDevice is idempotent: stopping an already-stopped device succeeds (no
  // error), so the watchdog restart can retry startDevice after a failed start.
  bool result = stopDevice(device, logger);

  EXPECT_TRUE(result);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0)
      << "already-stopped device should not log an error";
}

TEST_F(DeviceTest, StopDevice_NullDevice_ReturnsFalse) {
  viam::sdk::LogSource logger;
  std::shared_ptr<boost::synchronized_value<ViamRSDevice<>>> device = nullptr;
  EXPECT_FALSE(stopDevice(device, logger));
}

TEST_F(DeviceTest, DestroyDevice_NullDevice_ReturnsFalse) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  std::shared_ptr<boost::synchronized_value<ViamRSDevice<>>> device = nullptr;

  // Execute
  bool result = destroyDevice(device, logger);

  // Verify function result
  EXPECT_FALSE(result);

  // Verify error log for null device
  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1) << "Should log error for null device";
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("trying to destroy an unexistent device"));
}

TEST_F(DeviceTest, DestroyDevice_StartedDevice_StopsAndDestroys) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Create a test device that's started
  auto device = std::make_shared<boost::synchronized_value<
      ViamRSDevice<rs2::device, MockPipeline, MockAlign, MockConfig>>>();
  {
    auto dev_guard = device->synchronize();
    dev_guard->serial_number = "test123";
    dev_guard->started = true; // Device is started
    dev_guard->pipe = std::make_shared<MockPipeline>();
    dev_guard->device = std::make_shared<MockDevice>();
    dev_guard->config = std::make_shared<MockConfig>();
    dev_guard->align = std::make_shared<MockAlign>(RS2_STREAM_COLOR);
    dev_guard->point_cloud_filter = std::make_shared<PointCloudFilter>();
  }

  // Execute
  bool result = destroyDevice(device, logger);

  // Verify function result
  EXPECT_TRUE(result);
  EXPECT_EQ(device, nullptr);

  // Verify logs - device WAS started, so SHOULD have "stopping pipe" log
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log stop and destruction process";

  // Verify no errors
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0)
      << "Should not log errors for valid started device";

  // Verify expected INFO logs (destroying, stopping pipe, clearing, destroyed)
  bool found_destroying = false;
  bool found_stopping = false;
  bool found_destroyed = false;

  for (const auto &log : all_logs) {
    if (log.message.find("destroying") != std::string::npos &&
        log.message.find("test123") != std::string::npos) {
      found_destroying = true;
    }
    if (log.message.find("stopping pipe") != std::string::npos) {
      found_stopping = true;
    }
    if (log.message.find("device destroyed") != std::string::npos) {
      found_destroyed = true;
    }
  }

  EXPECT_TRUE(found_destroying) << "Should log 'destroying device'";
  EXPECT_TRUE(found_stopping)
      << "Should log 'stopping pipe' when device was started";
  EXPECT_TRUE(found_destroyed) << "Should log 'device destroyed'";
}

// Test enableGlobalTimestamp function
TEST_F(DeviceTest, EnableGlobalTimestamp_SupportedSensor_EnablesOption) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockSensor mock_sensor;
  mock_sensor.set_sensor_type(true, false); // Color sensor

  // Setup expectations - sensor supports global timestamp
  EXPECT_CALL(mock_sensor, supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      .WillOnce(Return(true));
  EXPECT_CALL(mock_sensor, set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0))
      .Times(1);

  // Execute
  enableGlobalTimestamp(mock_sensor, logger);

  // Verify info log
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log info message";

  // Verify the log contains success message
  bool found_enabled_msg = false;
  for (const auto &log : all_logs) {
    if (log.message.find("Enabled Global Timestamp") != std::string::npos &&
        log.message.find("color") != std::string::npos) {
      found_enabled_msg = true;
      break;
    }
  }
  EXPECT_TRUE(found_enabled_msg)
      << "Should log 'Enabled Global Timestamp for sensor: color'";

  // Verify no errors
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0)
      << "Should not log errors for successful operation";
}

TEST_F(DeviceTest, EnableGlobalTimestamp_UnsupportedSensor_DoesNothing) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockSensor mock_sensor;
  mock_sensor.set_sensor_type(false, true); // Depth sensor

  // Setup expectations - sensor does NOT support global timestamp
  EXPECT_CALL(mock_sensor, supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      .WillOnce(Return(false));
  EXPECT_CALL(mock_sensor, set_option(_, _)).Times(0); // Should not be called

  // Execute
  enableGlobalTimestamp(mock_sensor, logger);

  // Verify no logs (function returns early if not supported)
  auto all_logs = log_capture.get_records();
  EXPECT_EQ(all_logs.size(), 0) << "Should not log when option not supported";
}

TEST_F(DeviceTest, EnableGlobalTimestamp_SetOptionFails_LogsWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockSensor mock_sensor;
  mock_sensor.set_sensor_type(true, false); // Color sensor

  // Setup expectations - sensor supports option but set_option throws
  EXPECT_CALL(mock_sensor, supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      .WillOnce(Return(true));
  EXPECT_CALL(mock_sensor, set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0))
      .WillOnce(::testing::Throw(std::runtime_error("Hardware error")));

  // Execute
  enableGlobalTimestamp(mock_sensor, logger);

  // Verify warning log (failure is benign, so it should not be an error)
  auto warning_logs = log_capture.get_warning_logs();
  ASSERT_EQ(warning_logs.size(), 1)
      << "Should log warning when set_option fails";
  EXPECT_THAT(warning_logs[0].message,
              ::testing::HasSubstr("Failed to enable Global Timestamp"));
  EXPECT_THAT(warning_logs[0].message, ::testing::HasSubstr("Hardware error"));

  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0) << "Should not log at error level";
}

// Test disableAutoExposurePriority function
TEST_F(DeviceTest, DisableAutoExposurePriority_ColorSensor_DisablesOption) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockSensor mock_sensor;
  mock_sensor.set_sensor_type(true, false); // Color sensor

  // Setup expectations - color sensor supports auto-exposure priority
  EXPECT_CALL(mock_sensor, supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      .WillOnce(Return(true));
  EXPECT_CALL(mock_sensor, set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0))
      .Times(1);

  // Execute
  disableAutoExposurePriority(mock_sensor, logger);

  // Verify info log
  auto all_logs = log_capture.get_records();
  EXPECT_GT(all_logs.size(), 0) << "Should log info message";

  // Verify the log contains success message
  bool found_disabled_msg = false;
  for (const auto &log : all_logs) {
    if (log.message.find("Disabled Auto-Exposure Priority") !=
            std::string::npos &&
        log.message.find("color sensor") != std::string::npos) {
      found_disabled_msg = true;
      break;
    }
  }
  EXPECT_TRUE(found_disabled_msg)
      << "Should log 'Disabled Auto-Exposure Priority for color sensor'";

  // Verify no errors
  auto error_logs = log_capture.get_error_logs();
  EXPECT_EQ(error_logs.size(), 0)
      << "Should not log errors for successful operation";
}

TEST_F(DeviceTest, DisableAutoExposurePriority_OptionUnsupported_DoesNothing) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockSensor mock_sensor;
  mock_sensor.set_sensor_type(false, true); // Stereo sensor (e.g. D405)

  // Sensor does not expose the auto-exposure priority option
  EXPECT_CALL(mock_sensor, supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      .WillOnce(Return(false));
  EXPECT_CALL(mock_sensor, set_option(_, _)).Times(0);

  // Execute
  disableAutoExposurePriority(mock_sensor, logger);

  // Verify no logs (function returns early if option not supported)
  auto all_logs = log_capture.get_records();
  EXPECT_EQ(all_logs.size(), 0) << "Should not log when option not supported";
}

TEST_F(DeviceTest, DisableAutoExposurePriority_SetOptionFails_LogsWarning) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  MockSensor mock_sensor;
  mock_sensor.set_sensor_type(true, false); // Color sensor

  // Setup expectations - sensor supports option but set_option throws
  EXPECT_CALL(mock_sensor, supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      .WillOnce(Return(true));
  EXPECT_CALL(mock_sensor, set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0))
      .WillOnce(::testing::Throw(std::runtime_error("Option not supported")));

  // Execute
  disableAutoExposurePriority(mock_sensor, logger);

  // Verify warning log (not error - this is a non-critical failure)
  auto warning_logs = log_capture.get_warning_logs();
  ASSERT_EQ(warning_logs.size(), 1)
      << "Should log warning when set_option fails";
  EXPECT_THAT(warning_logs[0].message,
              ::testing::HasSubstr("Failed to disable Auto-Exposure Priority"));
  EXPECT_THAT(warning_logs[0].message,
              ::testing::HasSubstr("Option not supported"));
}

TEST_F(DeviceTest,
       CreateSingleSensorConfig_ActualCall_OnlyEnablesGlobalTimestamp) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Create mock device with a color sensor
  auto mock_device = std::make_shared<SimpleDevice>();
  SimpleSensor color_sensor;
  color_sensor.set_sensor_type(true, false); // Color sensor

  // Setup stream profiles that will match
  SimpleStreamProfile profile;
  profile.format_ = RS2_FORMAT_RGB8;
  profile.width_ = 640;
  profile.height_ = 480;
  profile.fps_ = 30;
  profile.stream_index_ = 0;
  color_sensor.set_stream_profiles({profile});

  // Set expectations: enableGlobalTimestamp should be called exactly once
  EXPECT_CALL(*color_sensor.mock(), supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      .Times(1)
      .WillOnce(Return(true));
  EXPECT_CALL(*color_sensor.mock(),
              set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f))
      .Times(1);

  // Verify disableAutoExposurePriority is NOT called in single-sensor mode
  EXPECT_CALL(*color_sensor.mock(), supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      .Times(0);
  EXPECT_CALL(*color_sensor.mock(),
              set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, _))
      .Times(0);

  mock_device->set_sensors({color_sensor});

  // Create test config
  RsResourceConfig viam_config(
      "test_serial", "test_camera", {realsense::sensors::SensorType::color},
      std::optional<int>{640}, std::optional<int>{480});

  // ACTUALLY CALL createSingleSensorConfig!
  auto result =
      createSingleSensorConfig<SimpleDevice, SimpleConfig, rs2::color_sensor,
                               SimpleVideoStreamProfile, RsResourceConfig>(
          mock_device, viam_config, logger);

  // Verify the function was called successfully
  EXPECT_NE(result, nullptr)
      << "createSingleSensorConfig should return a config";

  // Verify logs show enableGlobalTimestamp was called
  auto all_logs = log_capture.get_records();
  bool found_global_timestamp_log = false;
  bool found_creating_config_log = false;

  for (const auto &log : all_logs) {
    if (log.message.find("Enabled Global Timestamp") != std::string::npos &&
        log.message.find("color") != std::string::npos) {
      found_global_timestamp_log = true;
    }
    if (log.message.find("Creating config for single sensor") !=
        std::string::npos) {
      found_creating_config_log = true;
    }
  }

  EXPECT_TRUE(found_global_timestamp_log)
      << "Should log that global timestamp was enabled";
  EXPECT_TRUE(found_creating_config_log)
      << "Should log that config is being created";
}

TEST_F(DeviceTest,
       CreateSwD2CAlignConfig_ActualCall_EnablesGlobalAndDisablesAutoExp) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Create mock device with both color and depth sensors
  auto mock_device = std::make_shared<SimpleDevice>();

  SimpleSensor color_sensor;
  color_sensor.set_sensor_type(true, false); // Color sensor

  SimpleSensor depth_sensor;
  depth_sensor.set_sensor_type(false, true); // Depth sensor

  // Setup stream profiles that will match
  SimpleStreamProfile color_profile;
  color_profile.format_ = RS2_FORMAT_RGB8;
  color_profile.width_ = 640;
  color_profile.height_ = 480;
  color_profile.fps_ = 30;
  color_profile.stream_index_ = 0;
  color_sensor.set_stream_profiles({color_profile});

  SimpleStreamProfile depth_profile;
  depth_profile.format_ = RS2_FORMAT_Z16;
  depth_profile.width_ = 640;
  depth_profile.height_ = 480;
  depth_profile.fps_ = 30;
  depth_profile.stream_index_ = 0;
  depth_sensor.set_stream_profiles({depth_profile});

  // Set expectations for color sensor:
  // - enableGlobalTimestamp should be called once
  // - disableAutoExposurePriority should be called once
  EXPECT_CALL(*color_sensor.mock(), supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      .Times(1)
      .WillOnce(Return(true));
  EXPECT_CALL(*color_sensor.mock(),
              set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f))
      .Times(1);
  EXPECT_CALL(*color_sensor.mock(), supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      .Times(1)
      .WillOnce(Return(true));
  EXPECT_CALL(*color_sensor.mock(),
              set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f))
      .Times(1);

  // Set expectations for depth sensor:
  // - enableGlobalTimestamp should be called once
  // - disableAutoExposurePriority should NOT be called
  EXPECT_CALL(*depth_sensor.mock(), supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      .Times(1)
      .WillOnce(Return(true));
  EXPECT_CALL(*depth_sensor.mock(),
              set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f))
      .Times(1);
  EXPECT_CALL(*depth_sensor.mock(), supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      .Times(0);
  EXPECT_CALL(*depth_sensor.mock(),
              set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, _))
      .Times(0);

  mock_device->set_sensors({color_sensor, depth_sensor});

  // Create test config for both sensors
  RsResourceConfig viam_config("test_serial", "test_camera",
                               {realsense::sensors::SensorType::color,
                                realsense::sensors::SensorType::depth},
                               std::optional<int>{640},
                               std::optional<int>{480});

  // ACTUALLY CALL createSwD2CAlignConfig!
  auto result =
      createSwD2CAlignConfig<SimpleDevice, SimpleConfig, rs2::color_sensor,
                             rs2::depth_sensor, SimpleVideoStreamProfile,
                             RsResourceConfig>(mock_device, viam_config,
                                               logger);

  // Verify the function was called successfully
  EXPECT_NE(result, nullptr) << "createSwD2CAlignConfig should return a config";

  // GMock will automatically verify all EXPECT_CALL expectations above

  // Verify logs show the expected calls
  auto all_logs = log_capture.get_records();
  bool found_color_global_log = false;
  bool found_depth_global_log = false;
  bool found_disable_auto_exp_log = false;
  bool found_matching_profiles_log = false;

  for (const auto &log : all_logs) {
    if (log.message.find("Enabled Global Timestamp") != std::string::npos) {
      if (log.message.find("color") != std::string::npos) {
        found_color_global_log = true;
      }
      if (log.message.find("depth") != std::string::npos) {
        found_depth_global_log = true;
      }
    }
    if (log.message.find("Disabled Auto-Exposure Priority") !=
        std::string::npos) {
      found_disable_auto_exp_log = true;
    }
    if (log.message.find("Found matching color and depth stream profiles") !=
        std::string::npos) {
      found_matching_profiles_log = true;
    }
  }

  EXPECT_TRUE(found_color_global_log)
      << "Should log that global timestamp was enabled for color";
  EXPECT_TRUE(found_depth_global_log)
      << "Should log that global timestamp was enabled for depth";
  EXPECT_TRUE(found_disable_auto_exp_log)
      << "Should log that auto-exposure was disabled for color";
  EXPECT_TRUE(found_matching_profiles_log)
      << "Should log that matching profiles were found";
}

// Helper: a SimpleDevice with one color sensor whose only stream profile
// (1920x1080) cannot satisfy a 640x480 request — the shape of a USB 2
// connection where high-res profiles exist but the requested one doesn't.
static std::shared_ptr<SimpleDevice> makeDeviceWithoutMatchingProfile() {
  auto device = std::make_shared<SimpleDevice>();
  SimpleSensor color_sensor;
  color_sensor.set_sensor_type(true, false);
  EXPECT_CALL(*color_sensor.mock(), supports(_))
      .Times(::testing::AnyNumber())
      .WillRepeatedly(Return(false));
  SimpleStreamProfile profile;
  profile.format_ = RS2_FORMAT_RGB8;
  profile.width_ = 1920;
  profile.height_ = 1080;
  profile.fps_ = 30;
  profile.stream_index_ = 0;
  color_sensor.set_stream_profiles({profile});
  device->set_sensors({color_sensor});
  return device;
}

TEST_F(DeviceTest, CreateConfig_NoMatchingProfileOnUSB2_ErrorMentionsUSB) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  auto device = makeDeviceWithoutMatchingProfile();
  device->set_usb_type("2.1");

  RsResourceConfig viam_config(
      "test_serial", "test_camera", {realsense::sensors::SensorType::color},
      std::optional<int>{640}, std::optional<int>{480});

  try {
    createConfig<SimpleDevice, SimpleConfig, rs2::color_sensor,
                 rs2::depth_sensor, SimpleVideoStreamProfile, RsResourceConfig>(
        device, viam_config, logger);
    FAIL() << "createConfig should throw when no profile matches";
  } catch (const std::runtime_error &e) {
    std::string message = e.what();
    EXPECT_NE(message.find("Current device configuration not supported"),
              std::string::npos);
    EXPECT_NE(message.find("USB 2.1"), std::string::npos)
        << "Error should point at the USB 2 connection as a likely cause";
  }

  auto error_logs = log_capture.get_error_logs();
  EXPECT_TRUE(anyLogContains(error_logs, "USB 2.1"))
      << "Error log should mention the USB 2 connection";
}

TEST_F(DeviceTest, CreateConfig_NoMatchingProfileNoUSBInfo_PlainError) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // No USB type set — device does not report a descriptor
  auto device = makeDeviceWithoutMatchingProfile();

  RsResourceConfig viam_config(
      "test_serial", "test_camera", {realsense::sensors::SensorType::color},
      std::optional<int>{640}, std::optional<int>{480});

  try {
    createConfig<SimpleDevice, SimpleConfig, rs2::color_sensor,
                 rs2::depth_sensor, SimpleVideoStreamProfile, RsResourceConfig>(
        device, viam_config, logger);
    FAIL() << "createConfig should throw when no profile matches";
  } catch (const std::runtime_error &e) {
    std::string message = e.what();
    EXPECT_NE(message.find("Current device configuration not supported"),
              std::string::npos);
    EXPECT_EQ(message.find("USB"), std::string::npos)
        << "Error should not speculate about USB when the type is unknown";
  }
}

} // namespace test
} // namespace device
} // namespace realsense

GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(
      new realsense::device::test::RealsenseTestEnvironment);
  return RUN_ALL_TESTS();
}
