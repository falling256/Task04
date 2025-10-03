// hikvision_camera_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>

#include <string>
#include <vector>
#include <memory>
#include <cstring>

using namespace std::chrono_literals;

class HikvisionCameraNode : public rclcpp::Node {
public:
  HikvisionCameraNode()
  : Node("hikvision_camera_node"), handle_(nullptr), device_info_copy_(nullptr)
  {
    // 连接参数
    this->declare_parameter<std::string>("mode", "auto");
    this->declare_parameter<std::string>("serial", "");
    this->declare_parameter<std::string>("ip", "");
    this->declare_parameter<std::string>("topic", "/image_raw");
    this->declare_parameter<int>("timeout_ms", 1000);
    this->declare_parameter<int>("reconnect_delay_ms", 2000);
    
    // 相机控制参数
    this->declare_parameter<double>("exposure_time", 10000.0);
    this->declare_parameter<bool>("auto_exposure", true);
    this->declare_parameter<double>("gain", 0.0);
    this->declare_parameter<bool>("auto_gain", true);
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<std::string>("pixel_format", "BGR8");

    // 获取连接参数
    mode_ = this->get_parameter("mode").as_string();
    serial_ = this->get_parameter("serial").as_string();
    ip_ = this->get_parameter("ip").as_string();
    topic_ = this->get_parameter("topic").as_string();
    timeout_ms_ = this->get_parameter("timeout_ms").as_int();
    reconnect_delay_ms_ = this->get_parameter("reconnect_delay_ms").as_int();
    
    // 获取相机控制参数
    exposure_time_ = this->get_parameter("exposure_time").as_double();
    auto_exposure_ = this->get_parameter("auto_exposure").as_bool();
    gain_ = this->get_parameter("gain").as_double();
    auto_gain_ = this->get_parameter("auto_gain").as_bool();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    pixel_format_ = this->get_parameter("pixel_format").as_string();

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, 10);
    
    // 设置参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikvisionCameraNode::on_parameter_changed, this, std::placeholders::_1));

    if (!connect_camera()) {
      RCLCPP_WARN(this->get_logger(), "首次连接失败，将由后台定时重试。");
      retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(reconnect_delay_ms_),
                                             [this]() { if (!handle_) connect_camera(); });
    } else {
      start_grab_timer();
    }
  }

  ~HikvisionCameraNode() override {
    stop_and_cleanup();
    if (device_info_copy_) {
      free(device_info_copy_);
      device_info_copy_ = nullptr;
    }
  }

private:
  std::string mode_, serial_, ip_, topic_;
  int timeout_ms_{1000};
  int reconnect_delay_ms_{2000};
  
  // 相机控制参数
  double exposure_time_{10000.0};
  bool auto_exposure_{true};
  double gain_{0.0};
  bool auto_gain_{true};
  double frame_rate_{30.0};
  std::string pixel_format_{"BGR8"};

  void* handle_;
  MV_CC_DEVICE_INFO* device_info_copy_; 

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr grab_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 参数回调函数
  rcl_interfaces::msg::SetParametersResult on_parameter_changed(
      const std::vector<rclcpp::Parameter> &parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters) {
      if (param.get_name() == "exposure_time") {
        double new_exposure = param.as_double();
        if (set_exposure_time(new_exposure)) {
          exposure_time_ = new_exposure;
          RCLCPP_INFO(this->get_logger(), "曝光时间设置为: %.1f", new_exposure);
        } else {
          result.successful = false;
          result.reason = "Failed to set exposure time";
        }
      } 
      else if (param.get_name() == "auto_exposure") {
        bool new_auto_exposure = param.as_bool();
        if (set_auto_exposure(new_auto_exposure)) {
          auto_exposure_ = new_auto_exposure;
          RCLCPP_INFO(this->get_logger(), "自动曝光设置为: %s", new_auto_exposure ? "true" : "false");
        } else {
          result.successful = false;
          result.reason = "Failed to set auto exposure";
        }
      }
      else if (param.get_name() == "gain") {
        double new_gain = param.as_double();
        if (set_gain(new_gain)) {
          gain_ = new_gain;
          RCLCPP_INFO(this->get_logger(), "增益设置为: %.1f", new_gain);
        } else {
          result.successful = false;
          result.reason = "Failed to set gain";
        }
      }
      else if (param.get_name() == "auto_gain") {
        bool new_auto_gain = param.as_bool();
        if (set_auto_gain(new_auto_gain)) {
          auto_gain_ = new_auto_gain;
          RCLCPP_INFO(this->get_logger(), "自动增益设置为: %s", new_auto_gain ? "true" : "false");
        } else {
          result.successful = false;
          result.reason = "Failed to set auto gain";
        }
      }
      else if (param.get_name() == "frame_rate") {
        double new_frame_rate = param.as_double();
        if (set_frame_rate(new_frame_rate)) {
          frame_rate_ = new_frame_rate;
          RCLCPP_INFO(this->get_logger(), "帧率设置为: %.1f", new_frame_rate);
        } else {
          result.successful = false;
          result.reason = "Failed to set frame rate";
        }
      }
    }
    return result;
  }

  // 设置曝光时间
  bool set_exposure_time(double exposure_time) {
    if (!handle_) return false;
    int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置曝光时间失败: %d", ret);
      return false;
    }
    return true;
  }

  // 自动曝光 
  bool set_auto_exposure(bool auto_exposure) {
    if (!handle_) return false;
    // MVS SDK使用枚举值：0=Off, 1=Once, 2=Continuous
    int ret = MV_CC_SetEnumValue(handle_, "ExposureAuto", auto_exposure ? 2 : 0);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置自动曝光失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置增益
  bool set_gain(double gain) {
    if (!handle_) return false;
    int ret = MV_CC_SetFloatValue(handle_, "Gain", gain);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置增益失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置自动增益
  bool set_auto_gain(bool auto_gain) {
    if (!handle_) return false;
    int ret = MV_CC_SetEnumValue(handle_, "GainAuto", auto_gain ? 2 : 0);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置自动增益失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置帧率
  bool set_frame_rate(double frame_rate) {
    if (!handle_) return false;
    int ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置帧率失败: %d", ret);
      return false;
    }
    return true;
  }

  // 应用相机参数设置
  bool apply_camera_parameters() {
    if (!handle_) return false;
    
    bool success = true;
    
    if (!set_auto_exposure(auto_exposure_)) success = false;
    if (!auto_exposure_ && !set_exposure_time(exposure_time_)) success = false;
    
    if (!set_auto_gain(auto_gain_)) success = false;
    if (!auto_gain_ && !set_gain(gain_)) success = false;
    
    if (!set_frame_rate(frame_rate_)) success = false;
    
    return success;
  }

  void stop_and_cleanup() {
    if (handle_) {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
  }

  void start_grab_timer() {
    if (grab_timer_) return;
    grab_timer_ = this->create_wall_timer(30ms, std::bind(&HikvisionCameraNode::grab_image, this));
  }

  bool connect_camera() {
    stop_and_cleanup();
    if (device_info_copy_) { free(device_info_copy_); device_info_copy_ = nullptr; }

    MV_CC_DEVICE_INFO_LIST dev_list;
    std::memset(&dev_list, 0, sizeof(dev_list));
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_EnumDevices failed: %d", ret);
      return false;
    }
    if (dev_list.nDeviceNum == 0) {
      RCLCPP_WARN(this->get_logger(), "No devices found");
      return false;
    }

    MV_CC_DEVICE_INFO* chosen = nullptr;

    if (mode_ == "auto") {
      chosen = dev_list.pDeviceInfo[0];
    } else if (mode_ == "by_serial") {
      for (unsigned int i=0;i<dev_list.nDeviceNum;i++) {
        MV_CC_DEVICE_INFO* p = dev_list.pDeviceInfo[i];

        if (p->nTLayerType == MV_USB_DEVICE) {
          if (serial_.size() && std::strcmp((char*)p->SpecialInfo.stUsb3VInfo.chSerialNumber, serial_.c_str())==0) {
            chosen = p; break;
          }
        } else if (p->nTLayerType == MV_GIGE_DEVICE) {
          if (serial_.size() && std::strcmp((char*)p->SpecialInfo.stGigEInfo.chSerialNumber, serial_.c_str())==0) {
            chosen = p; break;
          }
        }
      }
    } else if (mode_ == "by_ip") {
      for (unsigned int i=0;i<dev_list.nDeviceNum;i++) {
        MV_CC_DEVICE_INFO* p = dev_list.pDeviceInfo[i];
        if (p->nTLayerType == MV_GIGE_DEVICE) {
          uint32_t ipnum = p->SpecialInfo.stGigEInfo.nCurrentIp;
          unsigned int a = (ipnum >> 24) & 0xFF;
          unsigned int b = (ipnum >> 16) & 0xFF;
          unsigned int c = (ipnum >> 8) & 0xFF;
          unsigned int d = (ipnum >> 0) & 0xFF;
          char buf[32];
          snprintf(buf, sizeof(buf), "%u.%u.%u.%u", a, b, c, d);
          if (ip_ == std::string(buf)) { chosen = p; break; }
        }
      }
    }

    if (!chosen) {
      RCLCPP_WARN(this->get_logger(), "No matching device found (mode=%s)", mode_.c_str());
      return false;
    }

    device_info_copy_ = (MV_CC_DEVICE_INFO*)malloc(sizeof(MV_CC_DEVICE_INFO));
    if (!device_info_copy_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate device_info_copy_");
      return false;
    }
    std::memcpy(device_info_copy_, chosen, sizeof(MV_CC_DEVICE_INFO));

    ret = MV_CC_CreateHandle(&handle_, device_info_copy_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_CreateHandle failed: %d", ret);
      free(device_info_copy_); device_info_copy_ = nullptr;
      handle_ = nullptr;
      return false;
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_OpenDevice failed: %d", ret);
      MV_CC_DestroyHandle(handle_); handle_ = nullptr;
      free(device_info_copy_); device_info_copy_ = nullptr;
      return false;
    }

    if (!apply_camera_parameters()) {
      RCLCPP_WARN(this->get_logger(), "部分相机参数设置失败");
    }

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_StartGrabbing failed: %d", ret);
      MV_CC_CloseDevice(handle_); MV_CC_DestroyHandle(handle_); handle_ = nullptr;
      free(device_info_copy_); device_info_copy_ = nullptr;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera opened and grabbing started.");
    return true;
  }

  void grab_image() {
    if (!handle_) return;

    MV_FRAME_OUT frame;
    std::memset(&frame, 0, sizeof(frame));
    int ret = MV_CC_GetImageBuffer(handle_, &frame, timeout_ms_);
    if (ret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "GetImageBuffer returned %d. Attempting reconnect.", ret);
      stop_and_cleanup();
      if (!retry_timer_) {
        retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(reconnect_delay_ms_),
                                               [this](){
                                                 if (!handle_ && connect_camera()) {
                                                   if (grab_timer_) { /* already running */ }
                                                   else {
                                                     start_grab_timer();
                                                   }
                                                   retry_timer_.reset();
                                                 }
                                               });
      }
      return;
    }

    MV_CC_PIXEL_CONVERT_PARAM conv;
    std::memset(&conv, 0, sizeof(conv));
    conv.nWidth = frame.stFrameInfo.nWidth;
    conv.nHeight = frame.stFrameInfo.nHeight;
    conv.pSrcData = frame.pBufAddr;
    conv.nSrcDataLen = frame.stFrameInfo.nFrameLen;
    conv.enSrcPixelType = frame.stFrameInfo.enPixelType;
    conv.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    size_t out_size = (size_t)conv.nWidth * conv.nHeight * 3;
    std::vector<uint8_t> outbuf(out_size);
    conv.pDstBuffer = outbuf.data();
    conv.nDstBufferSize = static_cast<uint32_t>(out_size);

    int cret = MV_CC_ConvertPixelType(handle_, &conv);
    if (cret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "ConvertPixelType failed: %d", cret);
      MV_CC_FreeImageBuffer(handle_, &frame);
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera";
    msg.height = conv.nHeight;
    msg.width = conv.nWidth;
    msg.encoding = "bgr8";
    msg.is_bigendian = 0;
    msg.step = conv.nWidth * 3;
    msg.data.assign(outbuf.begin(), outbuf.end());

    pub_->publish(msg);

    MV_CC_FreeImageBuffer(handle_, &frame);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HikvisionCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
