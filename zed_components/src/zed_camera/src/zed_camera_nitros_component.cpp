#include "zed_camera_nitros_component.hpp"

#include <sys/resource.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <limits>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <vector>
#include <sstream>

#include "sl_logging.hpp"

#ifdef FOUND_HUMBLE
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_IRON
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined FOUND_FOXY
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#error Unsupported ROS2 distro
#endif

#include <sl/Camera.hpp>

#include "sl_tools.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace stereolabs
{

ZedCameraNitros::ZedCameraNitros(const rclcpp::NodeOptions & options)
: Node("zed_camera_nitros", options),
  mQos(QOS_QUEUE_SIZE),
  mDiagUpdater(this),
  mGrabFreqTimer(get_clock())
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "      ZED Camera Component ");
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************");

  const size_t SDK_MAJOR_REQ = 4;
  const size_t SDK_MINOR_REQ = 2;

  if (ZED_SDK_MAJOR_VERSION < SDK_MAJOR_REQ ||
    (ZED_SDK_MAJOR_VERSION == SDK_MAJOR_REQ &&
    ZED_SDK_MINOR_VERSION < SDK_MINOR_REQ))
  {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "This version of the ZED ROS2 wrapper is designed to work with ZED SDK "
      "v" << static_cast<int>(SDK_MAJOR_REQ)
          << "." << static_cast<int>(SDK_MINOR_REQ) << " or newer.");
    RCLCPP_INFO_STREAM(
      get_logger(), "* Detected SDK v"
        << ZED_SDK_MAJOR_VERSION << "."
        << ZED_SDK_MINOR_VERSION << "."
        << ZED_SDK_PATCH_VERSION << "-"
        << ZED_SDK_BUILD_ID);
    RCLCPP_INFO(get_logger(), "Node stopped");
    exit(EXIT_FAILURE);
  }
  
  // ----> Start a "one shot timer" to initialize the node and make `shared_from_this` available
  std::chrono::milliseconds init_msec(static_cast<int>(50.0));
  mInitTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(init_msec),
    std::bind(&ZedCameraNitros::init, this));
  // <---- Start a "one shot timer" to initialize the node and make `shared_from_this` available
}

void ZedCameraNitros::init() {
  mInitTimer->cancel();

  // Initialize parameters
  initParameters();

  // ----> Diagnostic initialization
  mDiagUpdater.add(
    "ZED Diagnostic", this,
    &ZedCameraNitros::callback_updateDiagnostic);
  std::string hw_id = std::string("Stereolabs camera: ") + mCameraName;
  mDiagUpdater.setHardwareID(hw_id);
  // <---- Diagnostic initialization

  // ----> Start camera
  if (!startCamera()) {
    exit(EXIT_FAILURE);
  }
  // <---- Start camera

  // Dynamic parameters callback
  // mParamChangeCallbackHandle = add_on_set_parameters_callback(
    // std::bind(&ZedCameraNitros::callback_setParameters, this, _1));
}

void ZedCameraNitros::callback_updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DEBUG_COMM("*** Update Diagnostic ***");

  if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      sl::toString(mConnStatus).c_str());
    return;
  }

  if (mGrabStatus == sl::ERROR_CODE::SUCCESS) {
    double freq = 1. / mGrabPeriodMean_sec->getAvg();
    double freq_perc = 100. * freq / mPubFrameRate;
    stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

    double frame_proc_sec = mElabPeriodMean_sec->getAvg();
    // double frame_grab_period = 1. / mCamGrabFrameRate;
    double frame_grab_period = 1. / mPubFrameRate;
    stat.addf(
      "Capture", "Tot. Processing Time: %.6f sec (Max. %.3f sec)",
      frame_proc_sec, frame_grab_period);

    if (frame_proc_sec > frame_grab_period) {
      mSysOverloadCount++;
    }

    if (mSysOverloadCount >= 10) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "System overloaded. Consider reducing "
        "'general.pub_frame_rate' or 'general.grab_resolution'");
    } else {
      mSysOverloadCount = 0;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Camera grabbing");
    }

    stat.add("Input mode", "Live Camera");
  } else if (mGrabStatus == sl::ERROR_CODE::LAST) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Camera initializing");
  } else {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Camera error: %s", sl::toString(mGrabStatus).c_str());
  }

  if (sl_tools::isZEDX(mCamRealModel)) {
    stat.addf("Camera Temp.", "%.1f °C", mTempImu);

    if (mTempImu > 70.f) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "High Camera temperature");
    }
  }
} 

void ZedCameraNitros::initParameters() {
  // Init Debug parameters
  getDebugParams();

  // Init General parameters
  getGeneralParams();

  // Init Video parameters
  getVideoParams();

  // Init Sensors parameters
  // SENSORS parameters
  if (!sl_tools::isZED(mCamUserModel)) {
    getSensorsParams();
  }

  // Init Advanced parameters
  getAdvancedParams();
}

void ZedCameraNitros::getDebugParams() {
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** DEBUG parameters ***");

  getParam("debug.sdk_verbose", mVerbose, mVerbose, " * SDK Verbose: ");

  getParam("debug.debug_common", _debugCommon, _debugCommon);
  RCLCPP_INFO(
    get_logger(), " * Debug Common: %s",
    _debugCommon ? "TRUE" : "FALSE");

  getParam("debug.debug_sensors", _debugSensors, _debugSensors);
  RCLCPP_INFO(
    get_logger(), " * Debug sensors: %s",
    _debugSensors ? "TRUE" : "FALSE");

  getParam("debug.debug_camera_controls", _debugCamCtrl, _debugCamCtrl);
  RCLCPP_INFO(
    get_logger(), " * Debug Control settings: %s",
    _debugCamCtrl ? "TRUE" : "FALSE");

  getParam("debug.debug_advanced", _debugAdvanced, _debugAdvanced);
  RCLCPP_INFO(
    get_logger(), " * Debug Advanced: %s",
    _debugAdvanced ? "TRUE" : "FALSE");
  
  getParam("debug.debug_video_depth", _debugVideoDepth, _debugVideoDepth);
  RCLCPP_INFO(
    get_logger(), " * Debug Video/Depth: %s",
    _debugVideoDepth ? "TRUE" : "FALSE");

  mDebugMode = _debugCommon || _debugCamCtrl || _debugSensors || _debugAdvanced || _debugVideoDepth;

  if (mDebugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), " + Debug Mode enabled +");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  DEBUG_STREAM_COMM(
    "[ROS2] Using RMW_IMPLEMENTATION "
      << rmw_get_implementation_identifier());
}

void ZedCameraNitros::getVideoParams() {
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** VIDEO parameters ***");

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  if (!sl_tools::isZEDX(mCamUserModel)) {
    getParam(
      "video.brightness", mCamBrightness, mCamBrightness,
      " * [DYN] Brightness: ", true);
    getParam(
      "video.contrast", mCamContrast, mCamContrast,
      " * [DYN] Contrast: ", true);
    getParam("video.hue", mCamHue, mCamHue, " * [DYN] Hue: ", true);
  }

  getParam(
    "video.saturation", mCamSaturation, mCamSaturation,
    " * [DYN] Saturation: ", true);
  getParam(
    "video.sharpness", mCamSharpness, mCamSharpness,
    " * [DYN] Sharpness: ", true);
  getParam("video.gamma", mCamGamma, mCamGamma, " * [DYN] Gamma: ", true);
  getParam(
    "video.auto_exposure_gain", mCamAutoExpGain, mCamAutoExpGain, "",
    true);
  RCLCPP_INFO(
    get_logger(), " * [DYN] Auto Exposure/Gain: %s",
    mCamAutoExpGain ? "TRUE" : "FALSE");
  if (mCamAutoExpGain) {
    mTriggerAutoExpGain = true;
  }
  getParam(
    "video.exposure", mCamExposure, mCamExposure,
    " * [DYN] Exposure: ", true);
  getParam("video.gain", mCamGain, mCamGain, " * [DYN] Gain: ", true);
  getParam("video.auto_whitebalance", mCamAutoWB, mCamAutoWB, "", true);
  RCLCPP_INFO(
    get_logger(), " * [DYN] Auto White Balance: %s",
    mCamAutoWB ? "TRUE" : "FALSE");
  if (mCamAutoWB) {
    mTriggerAutoWB = true;
  }
  int wb = 42;
  getParam(
    "video.whitebalance_temperature", wb, wb,
    " * [DYN] White Balance Temperature: ", true);
  mCamWBTemp = wb * 100;

  if (sl_tools::isZEDX(mCamUserModel)) {
    getParam(
      "video.exposure_time", mGmslExpTime, mGmslExpTime,
      " * [DYN] ZED X Exposure time: ", true);
    getParam(
      "video.auto_exposure_time_range_min", mGmslAutoExpTimeRangeMin,
      mGmslAutoExpTimeRangeMin,
      " * [DYN] ZED X Auto Exp. time range min: ", true);
    getParam(
      "video.auto_exposure_time_range_max", mGmslAutoExpTimeRangeMax,
      mGmslAutoExpTimeRangeMax,
      " * [DYN] ZED X Auto Exp. time range max: ", true);
    if (mGmslAutoExpTimeRangeMax > mCamGrabFrameRate * 1000 ||
      mGmslAutoExpTimeRangeMax > 30000)
    {
      mGmslAutoExpTimeRangeMax = std::max(mCamGrabFrameRate * 1000, 30000);
      RCLCPP_WARN_STREAM(
        get_logger(),
        "The values of 'video.auto_exposure_time_range_max' is clamped to "
        "max(30000,'general.grab_frame_rate'x1000): "
          << mGmslAutoExpTimeRangeMax);
    }
    getParam(
      "video.exposure_compensation", mGmslExposureComp,
      mGmslExposureComp, " * [DYN] ZED X Exposure comp.: ", true);
    getParam(
      "video.analog_gain", mGmslAnalogGain, mGmslAnalogGain,
      " * [DYN] ZED X Analog Gain: ", true);
    getParam(
      "video.auto_analog_gain_range_min", mGmslAnalogGainRangeMin,
      mGmslAnalogGainRangeMin,
      " * [DYN] ZED X Auto Analog Gain range min: ", true);
    getParam(
      "video.auto_analog_gain_range_max", mGmslAnalogGainRangeMax,
      mGmslAnalogGainRangeMax,
      " * [DYN] ZED X Auto Analog Gain range max: ", true);
    getParam(
      "video.digital_gain", mGmslDigitalGain, mGmslDigitalGain,
      " * [DYN] ZED X Digital Gain: ", true);
    getParam(
      "video.auto_digital_gain_range_min", mGmslAutoDigitalGainRangeMin,
      mGmslAutoDigitalGainRangeMin,
      " * [DYN] ZED X Auto Digital Gain range min: ", true);
    getParam(
      "video.auto_digital_gain_range_max", mGmslAutoDigitalGainRangeMax,
      mGmslAutoDigitalGainRangeMax,
      " * [DYN] ZED X Auto Digital Gain range max: ", true);
    getParam(
      "video.denoising", mGmslDenoising, mGmslDenoising,
      " * [DYN] ZED X Auto Digital Gain range max: ", true);
  }
}

void ZedCameraNitros::getGeneralParams() {
  rclcpp::Parameter paramVal;

  RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

  std::string camera_model = "zed";
  getParam("general.camera_model", camera_model, camera_model);
  if (camera_model == "zed") {
    mCamUserModel = sl::MODEL::ZED;
  } else if (camera_model == "zedm") {
    mCamUserModel = sl::MODEL::ZED_M;
  } else if (camera_model == "zed2") {
    mCamUserModel = sl::MODEL::ZED2;
  } else if (camera_model == "zed2i") {
    mCamUserModel = sl::MODEL::ZED2i;
  } else if (camera_model == "zedx") {
    mCamUserModel = sl::MODEL::ZED_X;
    if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(mCamUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else if (camera_model == "zedxm") {
    mCamUserModel = sl::MODEL::ZED_XM;
    if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(mCamUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else if (camera_model == "virtual") {
    mCamUserModel = sl::MODEL::VIRTUAL_ZED_X;

    if (ZED_SDK_MAJOR_VERSION == 4 && ZED_SDK_MINOR_VERSION == 1 && ZED_SDK_PATCH_VERSION == 0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model '" << sl::toString(mCamUserModel).c_str()
                         << "' is available only with ZED SDK 4.1.1 or newer");
      exit(EXIT_FAILURE);
    }
    
    if (!IS_JETSON) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Camera model " << sl::toString(mCamUserModel).c_str()
                        << " is available only with NVIDIA Jetson devices.");
      exit(EXIT_FAILURE);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Camera model not valid in parameter values: " << camera_model);
  }
  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera model: " << camera_model << " - "
                                      << mCamUserModel);

  getParam("general.camera_name", mCameraName, mCameraName, " * Camera name: ");
  getParam(
    "general.serial_number", mCamSerialNumber, mCamSerialNumber,
    " * Camera SN: ");
  getParam(
    "general.camera_timeout_sec", mCamTimeoutSec, mCamTimeoutSec,
    " * Camera timeout [sec]: ");
  getParam(
    "general.camera_max_reconnect", mMaxReconnectTemp, mMaxReconnectTemp,
    " * Camera reconnection temptatives: ");
  getParam(
    "general.grab_frame_rate", mCamGrabFrameRate, mCamGrabFrameRate,
    " * Camera framerate: ");

  getParam("general.gpu_id", mGpuId, mGpuId, " * GPU ID: ");

  // TODO(walter) ADD SVO SAVE COMPRESSION PARAMETERS

  std::string resol = "AUTO";
  getParam("general.grab_resolution", resol, resol);
  if (resol == "AUTO") {
    mCamResol = sl::RESOLUTION::AUTO;
  } else if (sl_tools::isZEDX(mCamUserModel)) {
    if (resol == "HD1200") {
      mCamResol = sl::RESOLUTION::HD1200;
    } else if (resol == "HD1080") {
      mCamResol = sl::RESOLUTION::HD1080;
    } else if (resol == "SVGA") {
      mCamResol = sl::RESOLUTION::SVGA;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Not valid 'general.grab_resolution' value: '%s'. Using "
        "'AUTO' setting.",
        resol.c_str());
      mCamResol = sl::RESOLUTION::AUTO;
    }
    RCLCPP_INFO_STREAM(
      get_logger(), " * Camera resolution: "
        << sl::toString(mCamResol).c_str());
  } else {
    if (resol == "HD2K") {
      mCamResol = sl::RESOLUTION::HD2K;
    } else if (resol == "HD1080") {
      mCamResol = sl::RESOLUTION::HD1080;
    } else if (resol == "HD720") {
      mCamResol = sl::RESOLUTION::HD720;
    } else if (resol == "VGA") {
      mCamResol = sl::RESOLUTION::VGA;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Not valid 'general.grab_resolution' value: '%s'. Using "
        "'AUTO' setting.",
        resol.c_str());
      mCamResol = sl::RESOLUTION::AUTO;
    }
    RCLCPP_INFO_STREAM(
      get_logger(), " * Camera resolution: "
        << sl::toString(mCamResol).c_str());
  }

  std::string out_resol = "MEDIUM";
  getParam("general.pub_resolution", out_resol, out_resol);
  if (out_resol == "NATIVE") {
    mPubResolution = PubRes::NATIVE;
  } else if (out_resol == "CUSTOM") {
    mPubResolution = PubRes::CUSTOM;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Not valid 'general.pub_resolution' value: '%s'. Using default "
      "setting instead.",
      out_resol.c_str());
    out_resol = "NATIVE";
    mPubResolution = PubRes::NATIVE;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Publishing resolution: " << out_resol.c_str());

  if (mPubResolution == PubRes::CUSTOM) {
    getParam(
      "general.pub_downscale_factor", mCustomDownscaleFactor,
      mCustomDownscaleFactor, " * Publishing downscale factor: ");
  } else {
    mCustomDownscaleFactor = 1.0;
  }

  getParam(
    "general.optional_opencv_calibration_file", mOpencvCalibFile,
    mOpencvCalibFile, " * OpenCV custom calibration: ");

  getParam("general.self_calib", mCameraSelfCalib, mCameraSelfCalib);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera self calibration: " << (mCameraSelfCalib ? "TRUE" : "FALSE"));
  getParam("general.camera_flip", mCameraFlip, mCameraFlip);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera flip: " << (mCameraFlip ? "TRUE" : "FALSE"));

  // Dynamic parameters

  getParam("general.pub_frame_rate", mPubFrameRate, mPubFrameRate, "", false);
  if (mPubFrameRate > mCamGrabFrameRate) {
    RCLCPP_WARN(
      get_logger(),
      "'pub_frame_rate' cannot be bigger than 'grab_frame_rate'");
    mPubFrameRate = mCamGrabFrameRate;
  }
  if (mPubFrameRate < 0.1) {
    RCLCPP_WARN(
      get_logger(),
      "'pub_frame_rate' cannot be lower than 0.1 Hz or negative.");
    mPubFrameRate = mCamGrabFrameRate;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * [DYN] Publish framerate [Hz]:  " << mPubFrameRate);
}

void ZedCameraNitros::getSensorsParams() {
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** SENSORS STACK parameters ***");
  if (sl_tools::isZED(mCamUserModel)) {
    RCLCPP_WARN(
      get_logger(),
      "!!! SENSORS parameters are not used with ZED !!!");
    return;
  }

  getParam("sensors.publish_imu_tf", mPublishImuTF, mPublishImuTF);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Broadcast IMU TF [not for ZED]: "
      << (mPublishImuTF ? "TRUE" : "FALSE"));

  getParam("sensors.sensors_image_sync", mSensCameraSync, mSensCameraSync);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Sensors Camera Sync: "
      << (mSensCameraSync ? "TRUE" : "FALSE"));

  getParam("sensors.sensors_pub_rate", mSensPubRate, mSensPubRate);
  if (mSensPubRate < mCamGrabFrameRate) {
    mSensPubRate = mCamGrabFrameRate;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Sensors publishing rate: " << mSensPubRate << " Hz");
}

void ZedCameraNitros::getAdvancedParams() {
  rclcpp::Parameter paramVal;

  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  RCLCPP_INFO(get_logger(), "*** Advanced parameters ***");

  getParam(
    "advanced.thread_sched_policy", mThreadSchedPolicy,
    mThreadSchedPolicy, " * Thread sched. policy: ");

  if (mThreadSchedPolicy == "SCHED_FIFO" || mThreadSchedPolicy == "SCHED_RR") {
    if (!sl_tools::checkRoot()) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "'sudo' permissions required to set "
          << mThreadSchedPolicy
          << " thread scheduling policy. Using Linux "
          "default [SCHED_OTHER]");
      mThreadSchedPolicy = "SCHED_OTHER";
    } else {
      getParam(
        "advanced.thread_grab_priority", mThreadPrioGrab,
        mThreadPrioGrab, " * Grab thread priority: ");
      getParam(
        "advanced.thread_sensor_priority", mThreadPrioSens,
        mThreadPrioSens, " * Sensors thread priority: ");
    }
  }
}

bool ZedCameraNitros::startCamera() {
  RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

  // Create a ZED object
  mZed = std::make_shared<sl::Camera>();

  // ----> SDK version
  RCLCPP_INFO(
    get_logger(), "ZED SDK Version: %d.%d.%d - Build %s",
    ZED_SDK_MAJOR_VERSION, ZED_SDK_MINOR_VERSION,
    ZED_SDK_PATCH_VERSION, ZED_SDK_BUILD_ID);
  // <---- SDK version
  
  // ----> TF2 Transform
  mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  mTfListener = std::make_unique<tf2_ros::TransformListener>(
    *mTfBuffer);    // Start TF Listener thread
  mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // <---- TF2 Transform

  // ----> ZED configuration
  RCLCPP_INFO(get_logger(), "*** CAMERA OPENING ***");

  mInitParams.camera_fps = mCamGrabFrameRate;
  mInitParams.grab_compute_capping_fps = static_cast<float>(mPubFrameRate);
  mInitParams.camera_resolution = static_cast<sl::RESOLUTION>(mCamResol);

  if (mCamSerialNumber > 0) {
    mInitParams.input.setFromSerialNumber(mCamSerialNumber);
  }

  mInitParams.coordinate_system = ROS_COORDINATE_SYSTEM;
  mInitParams.coordinate_units = ROS_MEAS_UNITS;
  mInitParams.sdk_verbose = mVerbose;
  mInitParams.sdk_gpu_id = mGpuId;
  mInitParams.camera_image_flip = (mCameraFlip ? sl::FLIP_MODE::ON : sl::FLIP_MODE::OFF);

  if (!mOpencvCalibFile.empty()) {
    mInitParams.optional_opencv_calibration_file = mOpencvCalibFile.c_str();
  }

  mInitParams.camera_disable_self_calib = !mCameraSelfCalib;
  mInitParams.enable_image_enhancement = true;
  mInitParams.enable_right_side_measure = false;

  mInitParams.async_grab_camera_recovery = 
    true; // Camera recovery is handled asynchronously to provide information
          // about this status

  // NOTE: this is a temp fix to GMSL2 camera close issues
  // TODO: check if this issue has been fixed in the SDK
  if (sl_tools::isZEDX(mCamUserModel)) {
    RCLCPP_INFO(get_logger(), "Disable async recovery for GMSL2 cameras");
    mInitParams.async_grab_camera_recovery = false;
  }
  // <---- ZED configuration

  // ----> Try to connect to a camera
  sl_tools::StopWatch connectTimer(get_clock());

  mThreadStop = false;
  mGrabStatus = sl::ERROR_CODE::LAST;

  while (1) {
    rclcpp::sleep_for(500ms);

    mConnStatus = mZed->open(mInitParams);

    if (mConnStatus == sl::ERROR_CODE::SUCCESS) {
      DEBUG_STREAM_COMM("Opening successfull");
      break;
    }

    if (mConnStatus == sl::ERROR_CODE::INVALID_CALIBRATION_FILE) {
      if (mOpencvCalibFile.empty()) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Calibration file error: "
            << sl::toVerbose(mConnStatus));
      } else {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "If you are using a custom OpenCV calibration file, please check "
          "the correctness of the path of the calibration file "
          "in the parameter 'general.optional_opencv_calibration_file': '"
            << mOpencvCalibFile << "'.");
        RCLCPP_ERROR(
          get_logger(),
          "If the file exists, it may contain invalid information.");
      }
      return false;
    }

    RCLCPP_WARN(
    get_logger(), "Error opening camera: %s",
    sl::toString(mConnStatus).c_str());
    if (mConnStatus == sl::ERROR_CODE::CAMERA_DETECTION_ISSUE &&
      sl_tools::isZEDM(mCamUserModel))
    {
      RCLCPP_INFO(
        get_logger(),
        "Try to flip the USB3 Type-C connector and verify the USB3 "
        "connection");
    } else {
      RCLCPP_INFO(get_logger(), "Please verify the camera connection");
    }

    if (!rclcpp::ok() || mThreadStop) {
      RCLCPP_INFO(get_logger(), "ZED activation interrupted by user.");
      return false;
    }

    if (connectTimer.toc() > mMaxReconnectTemp * mCamTimeoutSec) {
      RCLCPP_ERROR(get_logger(), "Camera detection timeout");
      return false;
    }

    rclcpp::sleep_for(std::chrono::seconds(mCamTimeoutSec));
  }
  // ----> Try to connect to a camera

  // ----> Camera information
  sl::CameraInformation camInfo = mZed->getCameraInformation();

  float realFps = camInfo.camera_configuration.fps;
  if (realFps != static_cast<float>(mCamGrabFrameRate)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "!!! `general.grab_frame_rate` value is not valid: '"
        << mCamGrabFrameRate
        << "'. Automatically replaced with '" << realFps
        << "'. Please fix the parameter !!!");
    mCamGrabFrameRate = realFps;
  }

  // CUdevice devid;
  cuCtxGetDevice(&mGpuId);
  RCLCPP_INFO_STREAM(get_logger(), "ZED SDK running on GPU #" << mGpuId);

  // Camera model
  mCamRealModel = camInfo.camera_model;

  if (mCamRealModel == sl::MODEL::ZED) {
    if (mCamUserModel != sl::MODEL::ZED) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zed'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED_M) {
    if (mCamUserModel != sl::MODEL::ZED_M) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedm'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED2) {
    if (mCamUserModel != sl::MODEL::ZED2) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zed2'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED2i) {
    if (mCamUserModel != sl::MODEL::ZED2i) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zed2i'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED_X) {
    if (mCamUserModel != sl::MODEL::ZED_X) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedx'");
    }
  } else if (mCamRealModel == sl::MODEL::ZED_XM) {
    if (mCamUserModel != sl::MODEL::ZED_XM) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedxm'");
    }
  } else if (mCamRealModel == sl::MODEL::VIRTUAL_ZED_X) {
    if (mCamUserModel != sl::MODEL::VIRTUAL_ZED_X) {
      RCLCPP_WARN(
        get_logger(),
        "Camera model does not match user parameter. Please modify "
        "the value of the parameter 'general.camera_model' to 'zedxm'");
    }
  }

  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera Model  -> "
      << sl::toString(mCamRealModel).c_str());
  mCamSerialNumber = camInfo.serial_number;
  RCLCPP_INFO_STREAM(get_logger(), " * Serial Number -> " << mCamSerialNumber);

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Focal Lenght -> "
      << camInfo.camera_configuration.calibration_parameters
      .left_cam.focal_length_metric
      << " mm");

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Input\t -> "
      << sl::toString(mZed->getCameraInformation().input_type).c_str());
  if (mSvoMode) {
    RCLCPP_INFO(
      get_logger(), " * SVO resolution\t-> %ldx%ld",
      mZed->getCameraInformation().camera_configuration.resolution.width,
      mZed->getCameraInformation().camera_configuration.resolution.height);
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * SVO framerate\t-> "
        << (mZed->getCameraInformation().camera_configuration.fps));
  }

  // Firmwares
  mCamFwVersion = camInfo.camera_configuration.firmware_version;

  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Camera FW Version  -> " << mCamFwVersion);
  if (!sl_tools::isZED(mCamRealModel)) {
    mSensFwVersion = camInfo.sensors_configuration.firmware_version;
    RCLCPP_INFO_STREAM(
      get_logger(),
      " * Sensors FW Version -> " << mSensFwVersion);
  }

  // Camera/IMU transform
  if (!sl_tools::isZED(mCamRealModel)) {
    mSlCamImuTransf = camInfo.sensors_configuration.camera_imu_transform;

    DEBUG_SENS("Camera-IMU Transform:\n%s", mSlCamImuTransf.getInfos().c_str());
  }

  mCamWidth = camInfo.camera_configuration.resolution.width;
  mCamHeight = camInfo.camera_configuration.resolution.height;

  RCLCPP_INFO_STREAM(
    get_logger(), " * Camera grab frame size -> "
      << mCamWidth << "x" << mCamHeight);

  int pub_w, pub_h;
  pub_w = static_cast<int>(std::round(mCamWidth / mCustomDownscaleFactor));
  pub_h = static_cast<int>(std::round(mCamHeight / mCustomDownscaleFactor));

  if (pub_w > mCamWidth || pub_h > mCamHeight) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The publishing resolution ("
        << pub_w << "x" << pub_h
        << ") cannot be higher than the grabbing resolution ("
        << mCamWidth << "x" << mCamHeight
        << "). Using grab resolution for output messages.");
    pub_w = mCamWidth;
    pub_h = mCamHeight;
  }

  mMatResol = sl::Resolution(pub_w, pub_h);
  RCLCPP_INFO_STREAM(
    get_logger(), " * Publishing frame size  -> "
      << mMatResol.width << "x"
      << mMatResol.height);
  // <---- Camera information

  // ----> Check default camera settings
  if (_debugCamCtrl) {
    int value;
    sl::ERROR_CODE err;
    sl::VIDEO_SETTINGS setting;

    if (!sl_tools::isZEDX(mCamRealModel)) {
      setting = sl::VIDEO_SETTINGS::BRIGHTNESS;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "Default value for " << sl::toString(setting).c_str()
                             << ": " << value);

      setting = sl::VIDEO_SETTINGS::CONTRAST;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "Default value for " << sl::toString(setting).c_str()
                             << ": " << value);

      setting = sl::VIDEO_SETTINGS::HUE;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "Default value for " << sl::toString(setting).c_str()
                             << ": " << value);
    }

    setting = sl::VIDEO_SETTINGS::SATURATION;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::SHARPNESS;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::GAMMA;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::AEC_AGC;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::EXPOSURE;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::GAIN;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    setting = sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE;
    err = mZed->getCameraSettings(setting, value);
    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error Getting default param for "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
      exit(EXIT_FAILURE);
    }
    DEBUG_STREAM_CTRL(
      "Default value for " << sl::toString(setting).c_str()
                           << ": " << value);

    if (sl_tools::isZEDX(mCamRealModel)) {
      setting = sl::VIDEO_SETTINGS::EXPOSURE_TIME;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);

      // TODO(Walter) Enable when fixed in the SDK
      // setting = sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE;
      // err = mZed->getCameraSettings(setting, value_min, value_max);
      // if(err!=sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_ERROR_STREAM( get_logger(), "Error Getting default param for
      //   "
      //   << sl::toString(setting).c_str() << ": " <<
      //   sl::toString(err).c_str()); exit(EXIT_FAILURE);
      // }
      // DEBUG_STREAM_CTRL("[ZEDX] Default value for " <<
      // sl::toString(setting).c_str() << ": [" << value_min << "," <<
      // value_max
      // << "]");

      setting = sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);
  

      setting = sl::VIDEO_SETTINGS::ANALOG_GAIN;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);

      // TODO(Walter) Enable when fixed in the SDK
      // setting = sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE;
      // err = mZed->getCameraSettings(setting, value_min, value_max);
      // if(err!=sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_ERROR_STREAM( get_logger(), "Error Getting default param for
      //   "
      //   << sl::toString(setting).c_str() << ": " <<
      //   sl::toString(err).c_str()); exit(EXIT_FAILURE);
      // }
      // DEBUG_STREAM_CTRL("[ZEDX] Default value for " <<
      // sl::toString(setting).c_str() << ": [" << value_min << "," <<
      // value_max
      // << "]");

      setting = sl::VIDEO_SETTINGS::DIGITAL_GAIN;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);

      // TODO(Walter) Enable when fixed in the SDK
      // setting = sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE;
      // err = mZed->getCameraSettings(setting, value_min, value_max);
      // if(err!=sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_ERROR_STREAM( get_logger(), "Error Getting default param for
      //   "
      //   << sl::toString(setting).c_str() << ": " <<
      //   sl::toString(err).c_str()); exit(EXIT_FAILURE);
      // }
      // DEBUG_STREAM_CTRL("[ZEDX] Default value for " <<
      // sl::toString(setting).c_str() << ": [" << value_min << "," <<
      // value_max
      // << "]");

      setting = sl::VIDEO_SETTINGS::DENOISING;
      err = mZed->getCameraSettings(setting, value);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Error Getting default param for "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
        exit(EXIT_FAILURE);
      }
      DEBUG_STREAM_CTRL(
        "[ZEDX] Default value for "
          << sl::toString(setting).c_str() << ": " << value);
    }
  }
  // <----> Check default camera settings

  // ----> Camera Info messages
  mRgbCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRgbCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mLeftCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoRawMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mDepthCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  setTFCoordFrameNames();  // Requires mZedRealCamModel available only after
                           // camera opening

  fillCamInfo(
    mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId,
    mRightCamOptFrameId);
  fillCamInfo(
    mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId,
    mRightCamOptFrameId, true);
  mRgbCamInfoMsg = mLeftCamInfoMsg;
  mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
  mDepthCamInfoMsg = mLeftCamInfoMsg;
  // <---- Camera Info messages

  // Init Nitros Publishers
  initPublishers();

  mZed->setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
  mZed->setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
  // Force parameters with a dummy grab
  mZed->grab();

  // Initialialized timestamp to avoid wrong initial data
  // ----> Timestamp
  mFrameTimestamp = sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
  // <---- Timestamp

  // ----> Initialize Diagnostic statistics
  mElabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  mGrabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
  // <---- Initialize Diagnostic statistics

  // Init and start threads
  initThreads();

  return true;
}

void ZedCameraNitros::initThreads() {
  // Start grab thread
  mGrabThread = std::thread(&ZedCameraNitros::threadFunc_zedGrab, this);
}

void ZedCameraNitros::threadFunc_zedGrab() {
  DEBUG_STREAM_COMM("Grab thread started");

    // ----> Advanced thread settings
  DEBUG_STREAM_ADV("Grab thread settings");
  if (_debugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * Default GRAB thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }

  if (mThreadSchedPolicy == "SCHED_OTHER") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_BATCH") {
    sched_param par;
    par.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_FIFO") {
    sched_param par;
    par.sched_priority = mThreadPrioGrab;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else if (mThreadSchedPolicy == "SCHED_RR") {
    sched_param par;
    par.sched_priority = mThreadPrioGrab;
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to set thread params! - "
          << std::strerror(errno));
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), " ! Failed to set thread params! - Policy not supported");
  }

  if (_debugAdvanced) {
    int policy;
    sched_param par;
    if (pthread_getschedparam(pthread_self(), &policy, &par)) {
      RCLCPP_WARN_STREAM(
        get_logger(), " ! Failed to get thread policy! - "
          << std::strerror(errno));
    } else {
      DEBUG_STREAM_ADV(
        " * New GRAB thread (#"
          << pthread_self() << ") settings - Policy: "
          << sl_tools::threadSched2Str(policy).c_str()
          << " - Priority: " << par.sched_priority);
    }
  }
  // <---- Advanced thread settings

  mFrameCount = 0;

  // ----> Grab Runtime parameters
  mRunParams.enable_depth = false;
  mRunParams.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
  mRunParams.remove_saturated_areas = mRemoveSatAreas;
  // <---- Grab Runtime parameters

  while (1) {
    try {
      sl_tools::StopWatch grabElabTimer(get_clock());

      // ----> Interruption check
      if (!rclcpp::ok()) {
        DEBUG_STREAM_COMM("Ctrl+C received: stopping grab thread");
        break;
      }

      if (mThreadStop) {
        DEBUG_STREAM_COMM("Grab thread stopped");
        break;
      }
      // <---- Interruption check

      applyVideoSettings();

      // ----> Grab freq calculation
      double elapsed_sec = mGrabFreqTimer.toc();
      mGrabPeriodMean_sec->addValue(elapsed_sec);
      mGrabFreqTimer.tic();
      // <---- Grab freq calculation

      // Start processing timer for diagnostic
      grabElabTimer.tic();

      // ZED Grab
      mGrabStatus = mZed->grab(mRunParams);

      // ----> Grab errors?
      // Note: disconnection are automatically handled by the ZED SDK
      if (mGrabStatus != sl::ERROR_CODE::SUCCESS) {
        if (mGrabStatus == sl::ERROR_CODE::CAMERA_REBOOTING) {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Connection issue detected: "
              << sl::toString(mGrabStatus).c_str());
          rclcpp::sleep_for(1000ms);
          continue;
        } else if (mGrabStatus == sl::ERROR_CODE::CAMERA_NOT_INITIALIZED ||
          mGrabStatus == sl::ERROR_CODE::FAILURE)
        {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Camera issue detected: "
              << sl::toString(mGrabStatus).c_str() << ". Trying to recover the connection...");
          rclcpp::sleep_for(1000ms);
          continue;
        } else {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Critical camera error: " << sl::toString(mGrabStatus).c_str()
                                      << ". NODE KILLED.");
          mZed.reset();
          exit(EXIT_FAILURE);
        }
      }

      mFrameCount++;

      // ----> Timestamp
      mFrameTimestamp =
            sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::IMAGE));
      DEBUG_STREAM_COMM("Grab timestamp: " << mFrameTimestamp.nanoseconds() << " nsec");
      // <---- Timestamp

      // ----> Retrieve Image/Depth data if someone has subscribed to
      // Retrieve data if there are subscriber to topics
      DEBUG_STREAM_VD("Retrieving video/depth data");
      retrieveVideoDepth();

      rclcpp::Time pub_ts;
      publishVideoDepth(pub_ts);
      // <---- Retrieve Image/Depth data if someone has subscribed to

      // Diagnostic statistics update
      double mean_elab_sec = mElabPeriodMean_sec->addValue(grabElabTimer.toc());

    } catch (...) {
      rcutils_reset_error();
      DEBUG_STREAM_COMM("threadFunc_zedGrab: Generic exception.");
      continue;
    }
  }
  DEBUG_STREAM_COMM("Grab thread finished");
  // <---- Grab errors?
}

void ZedCameraNitros::applyVideoSettings() {
  sl::ERROR_CODE err;
  sl::VIDEO_SETTINGS setting;

  if (mFrameCount % 10 == 0) {
    std::lock_guard<std::mutex> lock(mDynParMutex);

    if (mTriggerAutoExpGain) {
      setting = sl::VIDEO_SETTINGS::AEC_AGC;
      err = mZed->setCameraSettings(setting, (mCamAutoExpGain ? 1 : 0));
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting AEC_AGC: "
            << sl::toString(err).c_str());
      } else {
        mTriggerAutoExpGain = false;
        DEBUG_STREAM_CTRL(
          "New setting for " << sl::toString(setting).c_str()
                             << ": "
                             << (mCamAutoExpGain ? 1 : 0));
      }
    }

    if (!mCamAutoExpGain) {
      int value;
      err = mZed->getCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamExposure) {
        mZed->setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, mCamExposure);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      err = mZed->getCameraSettings(sl::VIDEO_SETTINGS::GAIN, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamGain) {
        err = mZed->setCameraSettings(sl::VIDEO_SETTINGS::GAIN, mCamGain);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }

    if (mTriggerAutoWB) {
      setting = sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO;
      err = mZed->setCameraSettings(setting, (mCamAutoWB ? 1 : 0));
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      } else {
        mTriggerAutoWB = false;
        DEBUG_STREAM_CTRL(
          "New setting for " << sl::toString(setting).c_str()
                             << ": " << (mCamAutoWB ? 1 : 0));
      }
    }

    if (!mCamAutoWB) {
      int value;
      setting = sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamWBTemp) {
        err = mZed->setCameraSettings(setting, mCamWBTemp);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }

    // ----> BRIGHTNESS, CONTRAST, HUE controls not available for ZED X and
    // ZED X Mini
    if (!sl_tools::isZEDX(mCamRealModel)) {
      int value;
      setting = sl::VIDEO_SETTINGS::BRIGHTNESS;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamBrightness) {
        mZed->setCameraSettings(setting, mCamBrightness);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      setting = sl::VIDEO_SETTINGS::CONTRAST;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamContrast) {
        err = mZed->setCameraSettings(setting, mCamContrast);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      setting = sl::VIDEO_SETTINGS::HUE;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mCamHue) {
        mZed->setCameraSettings(setting, mCamHue);
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }
    // <---- BRIGHTNESS, CONTRAST, HUE controls not available for ZED X and
    // ZED X Mini

    setting = sl::VIDEO_SETTINGS::SATURATION;
    int value;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamSaturation) {
      mZed->setCameraSettings(setting, mCamSaturation);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::SHARPNESS;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamSharpness) {
      mZed->setCameraSettings(setting, mCamSharpness);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    setting = sl::VIDEO_SETTINGS::GAMMA;
    err = mZed->getCameraSettings(setting, value);
    if (err == sl::ERROR_CODE::SUCCESS && value != mCamGamma) {
      err = mZed->setCameraSettings(setting, mCamGamma);
    }

    if (err != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Error setting "
          << sl::toString(setting).c_str()
          << ": "
          << sl::toString(err).c_str());
    }

    if (sl_tools::isZEDX(mCamRealModel)) {
      if (!mCamAutoExpGain) {
        setting = sl::VIDEO_SETTINGS::EXPOSURE_TIME;
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslExpTime) {
          err = mZed->setCameraSettings(setting, mGmslExpTime);
          DEBUG_STREAM_CTRL(
            "New setting for "
              << sl::toString(setting).c_str() << ": "
              << mGmslExpTime << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Error setting " << sl::toString(setting).c_str()
                             << ": "
                             << sl::toString(err).c_str());
        }
      }

      // TODO(Walter) Enable when fixed in the SDK
      // err = mZed->getCameraSettings(
      //   sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE, value_min,
      //   value_max);
      // if (err == sl::ERROR_CODE::SUCCESS &&
      //   (value_min != mGmslAutoExpTimeRangeMin || value_max !=
      //   mGmslAutoExpTimeRangeMax))
      // {
      //   err = mZed->setCameraSettings(
      //     sl::VIDEO_SETTINGS::AUTO_EXPOSURE_TIME_RANGE,
      //     mGmslAutoExpTimeRangeMin, mGmslAutoExpTimeRangeMax);
      // } else if (err != sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_WARN_STREAM(
      //     get_logger(),
      //     "Error setting AUTO_EXPOSURE_TIME_RANGE: " <<
      //     sl::toString(err).c_str() );
      // }
      setting = sl::VIDEO_SETTINGS::EXPOSURE_COMPENSATION;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mGmslExposureComp) {
        err = mZed->setCameraSettings(setting, mGmslExposureComp);
        DEBUG_STREAM_CTRL(
          "New setting for " << sl::toString(setting).c_str()
                              << ": " << mGmslExposureComp
                              << " [Old " << value << "]");
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }

      setting = sl::VIDEO_SETTINGS::ANALOG_GAIN;
      if (!mCamAutoExpGain) {
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslAnalogGain) {
          err = mZed->setCameraSettings(setting, mGmslAnalogGain);
          DEBUG_STREAM_CTRL(
            "New setting for "
              << sl::toString(setting).c_str() << ": "
              << mGmslAnalogGain << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Error setting " << sl::toString(setting).c_str()
                             << ": "
                             << sl::toString(err).c_str());
        }

        // TODO(Walter) Enable when fixed in the SDK
        // err =
        //   mZed->getCameraSettings(sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
        //   value_min, value_max);
        // if (err == sl::ERROR_CODE::SUCCESS &&
        //   (value_min != mGmslAnalogGainRangeMin || value_max !=
        //   mGmslAnalogGainRangeMax))
        // {
        //   err = mZed->setCameraSettings(
        //     sl::VIDEO_SETTINGS::AUTO_ANALOG_GAIN_RANGE,
        //     mGmslAnalogGainRangeMin, mGmslAnalogGainRangeMax);
        // } else if (err != sl::ERROR_CODE::SUCCESS) {
        //   RCLCPP_WARN_STREAM(
        //     get_logger(),
        //     "Error setting AUTO_ANALOG_GAIN_RANGE: " <<
        //     sl::toString(err).c_str() );
        // }

        setting = sl::VIDEO_SETTINGS::DIGITAL_GAIN;
        err = mZed->getCameraSettings(setting, value);
        if (err == sl::ERROR_CODE::SUCCESS && value != mGmslDigitalGain) {
          err = mZed->setCameraSettings(setting, mGmslDigitalGain);
          DEBUG_STREAM_CTRL(
            "New setting for "
              << sl::toString(setting).c_str() << ": "
              << mGmslDigitalGain << " [Old " << value << "]");
        }

        if (err != sl::ERROR_CODE::SUCCESS) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Error setting " << sl::toString(setting).c_str()
                             << ": "
                             << sl::toString(err).c_str());
        }
      }

      // TODO(Walter) Enable when fixed in the SDK
      // err =
      //   mZed->getCameraSettings(sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      //   value_min, value_max);
      // if (err == sl::ERROR_CODE::SUCCESS &&
      //   (value_min != mGmslAutoDigitalGainRangeMin || value_max !=
      //   mGmslAutoDigitalGainRangeMax))
      // {
      //   err = mZed->setCameraSettings(
      //     sl::VIDEO_SETTINGS::AUTO_DIGITAL_GAIN_RANGE,
      //     mGmslAutoDigitalGainRangeMin, mGmslAnalogGainRangeMax);
      // } else if (err != sl::ERROR_CODE::SUCCESS) {
      //   RCLCPP_WARN_STREAM(
      //     get_logger(),
      //     "Error setting AUTO_DIGITAL_GAIN_RANGE: " <<
      //     sl::toString(err).c_str() );
      // }

      setting = sl::VIDEO_SETTINGS::DENOISING;
      err = mZed->getCameraSettings(setting, value);
      if (err == sl::ERROR_CODE::SUCCESS && value != mGmslDenoising) {
        err = mZed->setCameraSettings(setting, mGmslDenoising);
        DEBUG_STREAM_CTRL(
          "New setting for " << sl::toString(setting).c_str()
                              << ": " << mGmslDenoising
                              << " [Old " << value << "]");
      }

      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Error setting "
            << sl::toString(setting).c_str()
            << ": "
            << sl::toString(err).c_str());
      }
    }
  }
}

void ZedCameraNitros::setTFCoordFrameNames() {
  // ----> Coordinate frames
  mCameraFrameId = mCameraName + "_camera_center";
  mLeftCamFrameId = mCameraName + "_left_camera_frame";
  mLeftCamOptFrameId = mCameraName + "_left_camera_optical_frame";
  mRightCamFrameId = mCameraName + "_right_camera_frame";
  mRightCamOptFrameId = mCameraName + "_right_camera_optical_frame";

  // Print TF Frames
  RCLCPP_INFO_STREAM(get_logger(), " * Camera\t\t\t-> " << mCameraFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Left\t\t\t-> " << mLeftCamFrameId);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Left Optical\t\t-> " << mLeftCamOptFrameId);
  RCLCPP_INFO_STREAM(get_logger(), " * Right\t\t\t-> " << mRightCamFrameId);
  RCLCPP_INFO_STREAM(
    get_logger(),
    " * Right Optical\t\t-> " << mRightCamOptFrameId);
  
  if (!sl_tools::isZED(mCamRealModel)) {
    if (sl_tools::isZED2OrZED2i(mCamUserModel)) {
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * Left Temperature\t-> " << mTempLeftFrameId);
      RCLCPP_INFO_STREAM(
        get_logger(),
        " * Right Temperature\t-> " << mTempRightFrameId);
    }
  }
  // <---- Coordinate frames
}

void ZedCameraNitros::fillCamInfo(
  const std::shared_ptr<sl::Camera> zed,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> & leftCamInfoMsg,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> & rightCamInfoMsg,
  const std::string & leftFrameId, const std::string & rightFrameId,
  bool rawParam /*= false*/)
{
  sl::CalibrationParameters zedParam;

  if (rawParam) {
    zedParam = zed->getCameraInformation(mMatResol)
      .camera_configuration.calibration_parameters_raw;
  } else {
    zedParam = zed->getCameraInformation(mMatResol)
      .camera_configuration.calibration_parameters;
  }

  float baseline = zedParam.getCameraBaseline();

  // ----> Distortion models
  // ZED SDK params order: [ k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4]
  // Radial (k1, k2, k3, k4, k5, k6), Tangential (p1,p2) and Prism (s1, s2, s3,
  // s4) distortion. Prism not currently used.

  // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
  switch (mCamRealModel) {
    case sl::MODEL::ZED:  // PLUMB_BOB
      leftCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
      rightCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
      leftCamInfoMsg->d.resize(5);
      rightCamInfoMsg->d.resize(5);
      leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
      leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
      leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
      leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
      leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
      rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
      rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
      rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
      rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
      rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      break;

    case sl::MODEL::ZED2:    // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED2i:   // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_X:   // RATIONAL_POLYNOMIAL
    case sl::MODEL::ZED_XM:  // RATIONAL_POLYNOMIAL
    case sl::MODEL::VIRTUAL_ZED_X:  // RATIONAL_POLYNOMIAL
      leftCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      rightCamInfoMsg->distortion_model =
        sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
      leftCamInfoMsg->d.resize(8);
      rightCamInfoMsg->d.resize(8);
      leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
      leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
      leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
      leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
      leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
      leftCamInfoMsg->d[5] = zedParam.left_cam.disto[5];    // k4
      leftCamInfoMsg->d[6] = zedParam.left_cam.disto[6];    // k5
      leftCamInfoMsg->d[7] = zedParam.left_cam.disto[7];    // k6
      rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
      rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
      rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
      rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
      rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      rightCamInfoMsg->d[5] = zedParam.right_cam.disto[5];  // k4
      rightCamInfoMsg->d[6] = zedParam.right_cam.disto[6];  // k5
      rightCamInfoMsg->d[7] = zedParam.right_cam.disto[7];  // k6
      break;

    case sl::MODEL::ZED_M:
      if (zedParam.left_cam.disto[5] != 0 &&   // k4!=0
        zedParam.right_cam.disto[2] == 0 &&    // p1==0
        zedParam.right_cam.disto[3] == 0)      // p2==0
      {
        leftCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::EQUIDISTANT;
        rightCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::EQUIDISTANT;

        leftCamInfoMsg->d.resize(4);
        rightCamInfoMsg->d.resize(4);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];    // k3
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[5];    // k4
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4];  // k3
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[5];  // k4
      } else {
        leftCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
        rightCamInfoMsg->distortion_model =
          sensor_msgs::distortion_models::PLUMB_BOB;
        leftCamInfoMsg->d.resize(5);
        rightCamInfoMsg->d.resize(5);
        leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
        leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
        leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
        leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
        leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
        rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
        rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
        rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
        rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
        rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
      }
  }

  leftCamInfoMsg->k.fill(0.0);
  rightCamInfoMsg->k.fill(0.0);
  leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->k[8] = 1.0;
  rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->k[8] = 1.0;
  leftCamInfoMsg->r.fill(0.0);
  rightCamInfoMsg->r.fill(0.0);

  for (size_t i = 0; i < 3; i++) {
    // identity
    rightCamInfoMsg->r[i + i * 3] = 1;
    leftCamInfoMsg->r[i + i * 3] = 1;
  }

  if (rawParam) {
    // ROS frame (X forward, Z up, Y left)
    for (int i = 0; i < 9; i++) {
      rightCamInfoMsg->r[i] =
        zedParam.stereo_transform.getRotationMatrix().r[i];
    }
  }

  leftCamInfoMsg->p.fill(0.0);
  rightCamInfoMsg->p.fill(0.0);
  leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->p[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  rightCamInfoMsg->p[3] =
    static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
  rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->p[10] = 1.0;
  leftCamInfoMsg->width = rightCamInfoMsg->width =
    static_cast<uint32_t>(mMatResol.width);
  leftCamInfoMsg->height = rightCamInfoMsg->height =
    static_cast<uint32_t>(mMatResol.height);
  leftCamInfoMsg->header.frame_id = leftFrameId;
  rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZedCameraNitros::initPublishers() {
  RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

  std::string img_raw_nitros_topic = "nitros_image";
  std::string nitros_rbg_left_topic = img_raw_nitros_topic + "/left_image_raw";
  std::string nitros_rbg_right_topic = img_raw_nitros_topic + "/right_image_raw";
  std::string nitros_rbg_left_topic_info = img_raw_nitros_topic + "/left_info";
  std::string nitros_rbg_right_topic_info = img_raw_nitros_topic + "/right_info";

  // Nitros Image topics
  mNitrosPubLeftRgb = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
    this,
    nitros_rbg_left_topic,
    nvidia::isaac_ros::nitros::nitros_image_bgra8_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    output_qos_nitros
  );

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised Nitros on toipc: " << nitros_rbg_left_topic
  );

  mNitrosPubRightRgb = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
    this,
    nitros_rbg_right_topic,
    nvidia::isaac_ros::nitros::nitros_image_bgra8_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    output_qos_nitros
  );

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised Nitros on toipc: " << nitros_rbg_right_topic
  );

  mNitrosPubRightRgbInfo = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosCameraInfo>>(
    this,
    nitros_rbg_right_topic_info,
    nvidia::isaac_ros::nitros::nitros_camera_info_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    output_qos_nitros
  );

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised Nitros on toipc: " << nitros_rbg_right_topic_info
  );

  mNitrosPubLeftRgbInfo = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosCameraInfo>>(
    this,
    nitros_rbg_left_topic_info,
    nvidia::isaac_ros::nitros::nitros_camera_info_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    output_qos_nitros
  );

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Advertised Nitros on toipc: " << nitros_rbg_left_topic_info
  );
}

ZedCameraNitros::~ZedCameraNitros()
{
  RCLCPP_INFO(get_logger(), "ZED Camera component stopped");
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereolabs::ZedCameraNitros)