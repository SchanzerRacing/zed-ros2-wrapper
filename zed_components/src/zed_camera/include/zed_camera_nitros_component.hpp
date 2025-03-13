#ifndef ZED_CAMERA_NITROS_COMPONENT_HPP_
#define ZED_CAMERA_NITROS_COMPONENT_HPP_

#include <atomic>
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>
#include <unordered_set>

#include "sl_tools.hpp"
#include "sl_types.hpp"
#include "visibility_control.hpp"

namespace stereolabs
{

namespace {
    constexpr const char kDefaultQoS[] = "DEFAULT";
}

class ZedCameraNitros : public rclcpp::Node
{
public:
  ZED_COMPONENTS_PUBLIC
  explicit ZedCameraNitros(const rclcpp::NodeOptions & options);
  ~ZedCameraNitros();

protected:
  void init();
  void initParameters();
  void initServices();
  void initThreads();

  // -----> Get Parameters functions
  void getGeneralParams();
  void getVideoParams();
  void getSensorsParams();
  void getAdvancedParams();
  void getDebugParams();

  // <----- Get Parameters functions
  // ----> Thread functions
  void threadFunc_zedGrab();
  void threadFunc_pointcloudElab();
  void threadFunc_pubSensorsData();
  // <---- Thread functions

  bool startCamera();
  void retrieveVideoDepth();
  void publishVideoDepth(rclcpp::Time & out_pub_ts);
  void applyVideoSettings();

  // ----> Callbacks
  rcl_interfaces::msg::SetParametersResult callback_setParameters(
    std::vector<rclcpp::Parameter> parameters);
  void callback_updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);
  void callback_pubTemp();
  // <---- Callbacks

  void setTFCoordFrameNames();
  void initPublishers();
  void initSubscribers();
  void fillCamInfo(
    const std::shared_ptr<sl::Camera> zed,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> & leftCamInfoMsg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> & rightCamInfoMsg,
    const std::string & leftFrameId, const std::string & rightFrameId,
    bool rawParam = false);

  void publishNitrosImageWithInfo(
    sl::Mat & img,
    std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>> & pubImg,
    std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosCameraInfo>> & pubNitrosCamInfo,
    camInfoMsgPtr & camInfoMsg, std::string imgFrameId,
    rclcpp::Time t);
  
  void startTempPubTimer();

  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(), bool dynamic = false);

private:
  // ZED SDK
  std::shared_ptr<sl::Camera> mZed;
  sl::InitParameters mInitParams;
  sl::RuntimeParameters mRunParams;

  uint64_t mFrameCount = 0;

  std::string mTopicRoot = "~/";

    // ----> Thread Sync
  std::mutex mRecMutex;
  std::mutex mDynParMutex;
  std::mutex mPcMutex;
  std::condition_variable mPcDataReadyCondVar;
  std::atomic_bool mPcDataReady;
  // <---- Thread Sync

  // ----> Timestamps
  sl::Timestamp mLastTs_grab = 0;  // Used to calculate stable publish frequency
  rclcpp::Time mFrameTimestamp;
  rclcpp::Time mLastTs_pc;
  rclcpp::Time mPrevTs_pc;
  rclcpp::Time mLastClock;
  // <---- Timestamps

  sl::MODEL mCamUserModel = sl::MODEL::ZED;  // Default camera model
  sl::MODEL mCamRealModel;                   // Camera model requested to SDK
  unsigned int mCamFwVersion;                // Camera FW version
  unsigned int mSensFwVersion;               // Sensors FW version
  std::string mCameraName = "zed";           // Default camera name
  std::string mSvoFilepath = "";
  int mCamGrabFrameRate = 15;
  int mVerbose = 1;
  int mGpuId = -1;
  std::string mOpencvCalibFile;
  sl::RESOLUTION mCamResol =
    sl::RESOLUTION::HD1080;    // Default resolution: RESOLUTION_HD1080
  PubRes mPubResolution =
    PubRes::NATIVE;                     // Use native grab resolution by default
  double mCustomDownscaleFactor = 1.0;  // Used to rescale data with user factor

  // ----> Status Flags
  bool mSvoMode = false;
  bool mTriggerAutoExpGain = true;  // Triggered on start
  bool mTriggerAutoWB = true;       // Triggered on start
  // <---- Status Flags

  // -----------> Dynamic parameters handling
  OnSetParametersCallbackHandle::SharedPtr mParamChangeCallbackHandle;

  double mPubFrameRate = 15.0;
  int mCamBrightness = 4;
  int mCamContrast = 4;
  int mCamHue = 0;
  int mCamSaturation = 4;
  int mCamSharpness = 4;
  int mCamGamma = 8;
  bool mCamAutoExpGain = true;
  int mCamGain = 80;
  int mCamExposure = 80;
  bool mCamAutoWB = true;
  int mCamWBTemp = 42;
  int mDepthConf = 50;
  int mDepthTextConf = 100;
  double mPcPubRate = 15.0;
  double mFusedPcPubRate = 1.0;
  bool mRemoveSatAreas = true;

  int mGmslExpTime = 16666;
  int mGmslAutoExpTimeRangeMin = 28;
  int mGmslAutoExpTimeRangeMax = 30000;
  int mGmslExposureComp = 50;
  int mGmslAnalogGain = 8000;
  int mGmslAnalogGainRangeMin = 1000;
  int mGmslAnalogGainRangeMax = 16000;
  int mGmslDigitalGain = 128;
  int mGmslAutoDigitalGainRangeMin = 1;
  int mGmslAutoDigitalGainRangeMax = 256;
  int mGmslDenoising = 50;
  // <----------- Dynamic parameters handling

    // ----> QoS
  // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
  rclcpp::QoS mQos;
  rclcpp::PublisherOptions mPubOpt;
  rclcpp::SubscriptionOptions mSubOpt;
  // <---- QoS

  // ----> Frame IDs
  std::string mRgbFrameId;
  std::string mRgbOptFrameId;
  std::string mCameraFrameId;
  std::string mRightCamFrameId;
  std::string mRightCamOptFrameId;
  std::string mLeftCamFrameId;
  std::string mLeftCamOptFrameId;
  std::string mTempLeftFrameId;
  std::string mTempRightFrameId;
  // <---- Frame IDs

    // ----> Stereolabs Mat Info
  int mCamWidth;   // Camera frame width
  int mCamHeight;  // Camera frame height
  sl::Resolution mMatResol;
  // <---- Stereolabs Mat Info

    // ---- Nitros Image Publishers
  rclcpp::Time publishSensorsData(rclcpp::Time t = TIMEZERO_ROS);
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>> mNitrosPubRightRgb;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>> mNitrosPubLeftRgb;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosCameraInfo>> mNitrosPubRightRgbInfo;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosCameraInfo>> mNitrosPubLeftRgbInfo;
  const rclcpp::QoS output_qos_nitros = ::isaac_ros::common::AddQosParameter(
    *this, kDefaultQoS, "output_qos").keep_last(10);
  // <---- Nitros Image Publishers

  // ----> Threads and Timers
  sl::ERROR_CODE mGrabStatus;
  sl::ERROR_CODE mConnStatus;
  sl::FUSION_ERROR_CODE mFusionStatus = sl::FUSION_ERROR_CODE::MODULE_NOT_ENABLED;
  std::thread mGrabThread;        // Main grab thread
  std::thread mPcThread;          // Point Cloud publish thread
  std::thread mSensThread;        // Sensors data publish thread
  std::atomic<bool> mThreadStop;
  rclcpp::TimerBase::SharedPtr mInitTimer;
  rclcpp::TimerBase::SharedPtr mPathTimer;
  rclcpp::TimerBase::SharedPtr mFusedPcTimer;
  rclcpp::TimerBase::SharedPtr mTempPubTimer;    // Timer to retrieve and publish CMOS temperatures
  rclcpp::TimerBase::SharedPtr mGnssPubCheckTimer;
  // <---- Threads and Timers

  // -----> Parameter Variables
  bool mDebugMode = false;  // Debug mode active?
  bool _debugCommon = false;
  bool _debugSensors = false;
  bool _debugCamCtrl = false;
  bool _debugAdvanced = false;
  bool _debugVideoDepth = false;
  int mCamSerialNumber = 0;
  int mCamTimeoutSec = 5;
  int mMaxReconnectTemp = 5;
  bool mCameraSelfCalib = true;
  bool mCameraFlip = false;
  bool mPublishImuTF = false;
  bool mSensCameraSync = false;
  double mSensPubRate = 400.;

  std::string mThreadSchedPolicy;
  int mThreadPrioGrab;
  int mThreadPrioSens;
  // <----- Parameter Variables

  // <----- Diagnostic
  float mTempImu = NOT_VALID_TEMP;
  float mTempLeft = NOT_VALID_TEMP;
  float mTempRight = NOT_VALID_TEMP;
  diagnostic_updater::Updater mDiagUpdater;  // Diagnostic Updater
  std::unique_ptr<sl_tools::WinAvg> mGrabPeriodMean_sec;
  std::unique_ptr<sl_tools::WinAvg> mElabPeriodMean_sec;
  int mSysOverloadCount = 0;
    sl_tools::StopWatch mGrabFreqTimer;
  // -----> Diagnostic

  // Camera IMU transform
  sl::Transform mSlCamImuTransf;

  // ----> initialization Transform listener
  std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> mTfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
  // <---- initialization Transform listener

  // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
  // Camera infos
  camInfoMsgPtr mRgbCamInfoMsg;
  camInfoMsgPtr mLeftCamInfoMsg;
  camInfoMsgPtr mRightCamInfoMsg;
  camInfoMsgPtr mRgbCamInfoRawMsg;
  camInfoMsgPtr mLeftCamInfoRawMsg;
  camInfoMsgPtr mRightCamInfoRawMsg;
  camInfoMsgPtr mDepthCamInfoMsg;
  // <---- Messages
};

// ----> Template Function definitions
template<typename T>
void ZedCameraNitros::getParam(
  std::string paramName, T defValue, T & outVal,
  std::string log_info, bool dynamic)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

} // namespace stereolabs

#endif  // ZED_CAMERA_NITROS_COMPONENT_HPP_