// hikvision_camera_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>  
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>

#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <map>
#include <functional>
#include <atomic>
#include <algorithm>
#include <cctype>

using namespace std::chrono_literals;

static inline std::string to_upper_norm(const std::string &s) {
  std::string out;
  for (char c : s) {
    if (std::isalnum(static_cast<unsigned char>(c)))
      out.push_back(std::toupper(static_cast<unsigned char>(c)));
  }
  return out;
}

class HikvisionCameraNode : public rclcpp::Node {
public:
  HikvisionCameraNode()
  : Node("hikvision_camera_node"), 
    handle_(nullptr), 
    device_info_copy_(nullptr),
    is_grabbing_(false)
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
    pixel_format_ = to_upper_norm(this->get_parameter("pixel_format").as_string());

    // 初始化图像格式映射 - 使用正确的枚举类型
    pixel_format_map_ = {
        {"BGR8", PixelType_Gvsp_BGR8_Packed},
        {"RGB8", PixelType_Gvsp_RGB8_Packed},
        {"MONO8", PixelType_Gvsp_Mono8},
        {"MONO16", PixelType_Gvsp_Mono16}
    };

    // 初始化ROS编码映射
    ros_encoding_map_ = {
        {"BGR8", "bgr8"},
        {"RGB8", "rgb8"},
        {"MONO8", "mono8"},
        {"MONO16", "mono16"}
    };

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, 10);
    
    // 添加：硬件帧率发布器
    frame_rate_pub_ = this->create_publisher<std_msgs::msg::Float32>("/camera/hardware_frame_rate", 10);
    
    // 设置参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikvisionCameraNode::on_parameter_changed, this, std::placeholders::_1));

    // 健康检查定时器
    health_timer_ = this->create_wall_timer(5000ms, std::bind(&HikvisionCameraNode::health_check, this));
    
    // 添加：硬件帧率读取定时器（每秒读取一次）
    frame_rate_timer_ = this->create_wall_timer(1000ms, 
        std::bind(&HikvisionCameraNode::publish_hardware_frame_rate, this));

    // 立即尝试连接（最小阻塞时间）。如果失败，工作线程将持续重试。
    if (!connect_camera()) {
      RCLCPP_WARN(this->get_logger(), "首次连接失败，将由后台定时重试。");
      retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(reconnect_delay_ms_),
                                             [this]() { if (!handle_ || !is_grabbing_) connect_camera(); });
    } else {
      start_grab_timer();
    }

    RCLCPP_INFO(this->get_logger(), "节点启动完成");
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
  std::string pixel_format_{"BGR8"}; // 标准化大写

  // 图像格式映射 - 使用正确的枚举类型
  std::map<std::string, MvGvspPixelType> pixel_format_map_;
  std::map<std::string, std::string> ros_encoding_map_;

  void* handle_;
  MV_CC_DEVICE_INFO* device_info_copy_; // 用于句柄创建的持久化设备信息副本

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  // 添加：硬件帧率发布器
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frame_rate_pub_;
  
  rclcpp::TimerBase::SharedPtr grab_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  // 添加：硬件帧率定时器
  rclcpp::TimerBase::SharedPtr frame_rate_timer_;
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::atomic<bool> is_grabbing_;

  // 硬件帧率读取函数
  void publish_hardware_frame_rate() {
    if (!handle_) return;
    
    // 使用海康SDK直接读取相机实际帧率
    MVCC_FLOATVALUE frame_rate_value = {0};
    int ret = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &frame_rate_value);
    
    if (ret == MV_OK) {
      // 发布帧率消息
      std_msgs::msg::Float32 fps_msg;
      fps_msg.data = frame_rate_value.fCurValue;
      frame_rate_pub_->publish(fps_msg);
      
      static int log_count = 0;
      if (++log_count % 10 == 0) {  
        RCLCPP_INFO(this->get_logger(), 
                   "硬件帧率: 当前=%.1f FPS, 范围[%.1f~%.1f], 设定=%.1f FPS", 
                   frame_rate_value.fCurValue, frame_rate_value.fMin, 
                   frame_rate_value.fMax, frame_rate_);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "读取硬件帧率失败: %d", ret);
    }
  }

  // 根据SDK像素类型计算每像素字节数
  int bytes_per_pixel_for_sdk(MvGvspPixelType t) {
    if (t == PixelType_Gvsp_Mono8) return 1;
    if (t == PixelType_Gvsp_Mono16) return 2;
    if (t == PixelType_Gvsp_RGB8_Packed) return 3;
    if (t == PixelType_Gvsp_BGR8_Packed) return 3;
    return 3;
  }

  // 根据SDK像素类型映射到ROS编码（用于回退原始发布）
  std::string ros_encoding_for_sdk(MvGvspPixelType t) {
    if (t == PixelType_Gvsp_Mono8) return "mono8";
    if (t == PixelType_Gvsp_Mono16) return "mono16";
    if (t == PixelType_Gvsp_RGB8_Packed) return "rgb8";
    if (t == PixelType_Gvsp_BGR8_Packed) return "bgr8";
    return ""; 
  }

  // 参数回调函数
  rcl_interfaces::msg::SetParametersResult on_parameter_changed(
      const std::vector<rclcpp::Parameter> &parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    bool need_restart_timer = false;

    for (const auto &param : parameters) {
      const std::string &name = param.get_name();
      
      if (name == "exposure_time") {
        double new_exposure = param.as_double();
        exposure_time_ = new_exposure;
        if (handle_) {
          if (!set_exposure_time(new_exposure)) {
            result.successful = false;
            result.reason = "设置曝光时间失败";
            return result;
          }
        }
        RCLCPP_INFO(this->get_logger(), "曝光时间设置为: %.1f", new_exposure);
      } 
      else if (name == "auto_exposure") {
        bool new_auto_exposure = param.as_bool();
        auto_exposure_ = new_auto_exposure;
        if (handle_) {
          if (!set_auto_exposure(new_auto_exposure)) {
            result.successful = false;
            result.reason = "设置自动曝光失败";
            return result;
          }
        }
        RCLCPP_INFO(this->get_logger(), "自动曝光设置为: %s", new_auto_exposure ? "true" : "false");
      }
      else if (name == "gain") {
        double new_gain = param.as_double();
        gain_ = new_gain;
        if (handle_) {
          if (!set_gain(new_gain)) {
            result.successful = false;
            result.reason = "设置增益失败";
            return result;
          }
        }
        RCLCPP_INFO(this->get_logger(), "增益设置为: %.1f", new_gain);
      }
      else if (name == "auto_gain") {
        bool new_auto_gain = param.as_bool();
        auto_gain_ = new_auto_gain;
        if (handle_) {
          if (!set_auto_gain(new_auto_gain)) {
            result.successful = false;
            result.reason = "设置自动增益失败";
            return result;
          }
        }
        RCLCPP_INFO(this->get_logger(), "自动增益设置为: %s", new_auto_gain ? "true" : "false");
      }
      else if (name == "frame_rate") {
        double new_frame_rate = param.as_double();
        frame_rate_ = new_frame_rate;
        if (handle_) {
          if (!set_frame_rate(new_frame_rate)) {
            RCLCPP_WARN(this->get_logger(), "设置帧率到 %.1f 失败（驱动/相机可能限制），仍更新本地参数", new_frame_rate);
          }
        }
        RCLCPP_INFO(this->get_logger(), "帧率设置为: %.1f", new_frame_rate);
        need_restart_timer = true;
      }
      else if (name == "pixel_format") {
        std::string requested = to_upper_norm(param.as_string());
        if (pixel_format_map_.find(requested) == pixel_format_map_.end()) {
          result.successful = false;
          result.reason = "不支持的像素格式";
          RCLCPP_WARN(this->get_logger(), "请求的像素格式不在支持列表: %s", requested.c_str());
          return result;
        }
        pixel_format_ = requested;
        if (handle_) {
          if (!set_pixel_format(pixel_format_)) {
            RCLCPP_WARN(this->get_logger(), "硬件设置像素格式失败，节点将使用软件转换发布: %s", pixel_format_.c_str());
          } else {
            RCLCPP_INFO(this->get_logger(), "硬件像素格式设置成功: %s", pixel_format_.c_str());
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "预设目标像素格式: %s (相机未连接时仅记录)", pixel_format_.c_str());
        }
      }
    }

    if (need_restart_timer) {
      if (grab_timer_) { 
        grab_timer_->cancel(); 
        grab_timer_.reset(); 
      }
      start_grab_timer();
    }

    return result;
  }

  // 设置曝光时间
  bool set_exposure_time(double exposure_time) {
    if (!handle_) return true; 
    int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_time);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置曝光时间失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置自动曝光 - MVS SDK使用不同的API
  bool set_auto_exposure(bool auto_exposure) {
    if (!handle_) return true;
    int ret = MV_CC_SetEnumValue(handle_, "ExposureAuto", auto_exposure ? 2 : 0);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置自动曝光失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置增益
  bool set_gain(double gain) {
    if (!handle_) return true;
    int ret = MV_CC_SetFloatValue(handle_, "Gain", gain);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置增益失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置自动增益 - MVS SDK使用不同的API
  bool set_auto_gain(bool auto_gain) {
    if (!handle_) return true;
    int ret = MV_CC_SetEnumValue(handle_, "GainAuto", auto_gain ? 2 : 0);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "设置自动增益失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置帧率
  bool set_frame_rate(double frame_rate) {
    if (!handle_) return true;
    int ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate);
    if (ret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "设置帧率失败: %d", ret);
      return false;
    }
    return true;
  }

  // 设置图像格式
  bool set_pixel_format(const std::string& pixel_format) {
    if (!handle_) return true;
    
    // 检查格式是否支持
    if (pixel_format_map_.find(pixel_format) == pixel_format_map_.end()) {
      RCLCPP_ERROR(this->get_logger(), "不支持的图像格式: %s", pixel_format.c_str());
      RCLCPP_INFO(this->get_logger(), "支持的格式: BGR8, RGB8, MONO8, MONO16");
      return false;
    }
    
    // 设置图像格式
    int ret = MV_CC_SetEnumValue(handle_, "PixelFormat", static_cast<uint32_t>(pixel_format_map_[pixel_format]));
    if (ret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "设置图像格式失败: %d", ret);
      return false;
    }
    
    return true;
  }

  // 应用相机参数设置
  bool apply_camera_parameters() {
    if (!handle_) return true;
    
    bool success = true;
    
    // 设置曝光相关参数
    if (!set_auto_exposure(auto_exposure_)) success = false;
    if (!auto_exposure_ && !set_exposure_time(exposure_time_)) success = false;
    
    // 设置增益相关参数
    if (!set_auto_gain(auto_gain_)) success = false;
    if (!auto_gain_ && !set_gain(gain_)) success = false;
    
    // 设置帧率
    if (!set_frame_rate(frame_rate_)) success = false;
    
    // 尝试设置硬件像素格式，失败时使用软件转换
    if (!set_pixel_format(pixel_format_)) {
      RCLCPP_WARN(this->get_logger(), "应用硬件像素格式失败（将使用软件转换/回退）");
    }
    
    return success;
  }

  // 辅助函数：停止采集并清理句柄
  void stop_and_cleanup() {
    if (handle_) {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
    if (grab_timer_) { 
      grab_timer_->cancel(); 
      grab_timer_.reset(); 
    }
    is_grabbing_ = false;
  }

  // 启动定期采集定时器（基于帧率动态计算间隔）
  void start_grab_timer() {
    if (grab_timer_) return;
    int interval_ms = static_cast<int>(1000.0 / std::max(frame_rate_, 0.1));
    if (interval_ms < 1) interval_ms = 1;
    grab_timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_ms),
                                          std::bind(&HikvisionCameraNode::grab_image, this));
    RCLCPP_INFO(this->get_logger(), "采集定时器开启: 间隔 %d ms (目标 %.1f FPS)", interval_ms, frame_rate_);
  }

  void health_check() {
    if (!handle_ || !is_grabbing_) {
      // 如果未连接则尝试连接
      if (!handle_) connect_camera();
      return;
    }
    MV_FRAME_OUT frame;
    std::memset(&frame, 0, sizeof(frame));
    int ret = MV_CC_GetImageBuffer(handle_, &frame, 500);
    if (ret == MV_OK) {
      MV_CC_FreeImageBuffer(handle_, &frame);
      // 一切正常
    } else {
      RCLCPP_WARN(this->get_logger(), "健康检查: 获取图像失败 (%d)，尝试重连", ret);
      stop_and_cleanup();
      connect_camera();
    }
  }

  // 根据参数尝试发现并打开相机
  bool connect_camera() {
    stop_and_cleanup();
    if (device_info_copy_) { free(device_info_copy_); device_info_copy_ = nullptr; }

    MV_CC_DEVICE_INFO_LIST dev_list;
    std::memset(&dev_list, 0, sizeof(dev_list));
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "枚举设备失败: %d", ret);
      return false;
    }
    if (dev_list.nDeviceNum == 0) {
      RCLCPP_WARN(this->get_logger(), "未发现任何设备");
      return false;
    }

    MV_CC_DEVICE_INFO* chosen = nullptr;
    // 根据模式选择设备
    if (mode_ == "auto") {
      chosen = dev_list.pDeviceInfo[0];
      RCLCPP_INFO(this->get_logger(), "自动选择第一个设备");
    } else if (mode_ == "by_serial") {
      for (unsigned int i=0;i<dev_list.nDeviceNum;i++) {
        MV_CC_DEVICE_INFO* p = dev_list.pDeviceInfo[i];
        // 先尝试USB序列号
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
      RCLCPP_WARN(this->get_logger(), "未找到匹配的设备 (模式=%s)", mode_.c_str());
      return false;
    }

    // 创建选定设备信息的持久化副本（使内存保持有效）
    device_info_copy_ = (MV_CC_DEVICE_INFO*)malloc(sizeof(MV_CC_DEVICE_INFO));
    if (!device_info_copy_) {
      RCLCPP_ERROR(this->get_logger(), "分配设备信息副本失败");
      return false;
    }
    std::memcpy(device_info_copy_, chosen, sizeof(MV_CC_DEVICE_INFO));

    // 创建句柄并打开设备
    ret = MV_CC_CreateHandle(&handle_, device_info_copy_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "创建设备句柄失败: %d", ret);
      free(device_info_copy_); device_info_copy_ = nullptr;
      handle_ = nullptr;
      return false;
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "打开设备失败: %d", ret);
      MV_CC_DestroyHandle(handle_); handle_ = nullptr;
      free(device_info_copy_); device_info_copy_ = nullptr;
      return false;
    }

    // 应用相机参数设置
    if (!apply_camera_parameters()) {
      RCLCPP_WARN(this->get_logger(), "部分相机参数设置失败");
    }

    // 开始采集
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "开始采集失败: %d", ret);
      MV_CC_CloseDevice(handle_); MV_CC_DestroyHandle(handle_); handle_ = nullptr;
      free(device_info_copy_); device_info_copy_ = nullptr;
      return false;
    }

    is_grabbing_ = true;
    RCLCPP_INFO(this->get_logger(), "相机已打开并开始采集。");
    return true;
  }

  // 采集一帧图像，转换为指定格式并发布
  void grab_image() {
    if (!handle_ || !is_grabbing_) return;

    MV_FRAME_OUT frame;
    std::memset(&frame, 0, sizeof(frame));
    int ret = MV_CC_GetImageBuffer(handle_, &frame, timeout_ms_);
    if (ret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "获取图像缓冲区返回 %d，尝试重连。", ret);
      stop_and_cleanup();
      // 延迟后重试连接 - 使用重连定时器调用connect_camera
      if (!retry_timer_) {
        retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(reconnect_delay_ms_),
                                               [this](){
                                                 if (!handle_ && connect_camera()) {
                                                   // 连接成功 -> 启动采集定时器
                                                   if (!grab_timer_) {
                                                     start_grab_timer();
                                                   }
                                                   // 停止重连定时器
                                                   retry_timer_.reset();
                                                 }
                                               });
      }
      return;
    }

    // 准备转换参数
    MV_CC_PIXEL_CONVERT_PARAM conv;
    std::memset(&conv, 0, sizeof(conv));
    conv.nWidth = frame.stFrameInfo.nWidth;
    conv.nHeight = frame.stFrameInfo.nHeight;
    conv.pSrcData = frame.pBufAddr;
    conv.nSrcDataLen = frame.stFrameInfo.nFrameLen;
    conv.enSrcPixelType = frame.stFrameInfo.enPixelType;

    // 确定目标类型
    MvGvspPixelType dst_type = PixelType_Gvsp_BGR8_Packed;
    auto it = pixel_format_map_.find(pixel_format_);
    if (it != pixel_format_map_.end()) {
      dst_type = it->second;
    }
    conv.enDstPixelType = dst_type;

    // 根据目标bpp分配输出缓冲区大小
    int dst_bpp = bytes_per_pixel_for_sdk(dst_type);
    size_t out_size = static_cast<size_t>(conv.nWidth) * conv.nHeight * dst_bpp;
    std::vector<uint8_t> outbuf(out_size);
    conv.pDstBuffer = outbuf.data();
    conv.nDstBufferSize = static_cast<uint32_t>(out_size);

    int cret = MV_CC_ConvertPixelType(handle_, &conv);
    if (cret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "像素格式转换失败: %d，尝试回退：直接发布原始（若兼容）", cret);
      // 尝试回退：如果源像素类型已经是已知的简单类型，则发布原始数据
      std::string src_encoding = ros_encoding_for_sdk(conv.enSrcPixelType);
      int src_bpp = bytes_per_pixel_for_sdk(conv.enSrcPixelType);
      if (!src_encoding.empty() && frame.stFrameInfo.nFrameLen >= (uint32_t)(conv.nWidth * conv.nHeight * src_bpp)) {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera";
        msg.height = conv.nHeight;
        msg.width = conv.nWidth;
        msg.encoding = src_encoding;
        msg.is_bigendian = 0;
        msg.step = conv.nWidth * src_bpp;
        // 复制原始数据
        msg.data.resize(frame.stFrameInfo.nFrameLen);
        std::memcpy(msg.data.data(), frame.pBufAddr, frame.stFrameInfo.nFrameLen);
        pub_->publish(msg);
        MV_CC_FreeImageBuffer(handle_, &frame);
        return;
      } else {
        // 无法回退 -> 释放并返回
        MV_CC_FreeImageBuffer(handle_, &frame);
        return;
      }
    }

    // 从outbuf构建ROS消息（成功转换）
    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera";
    msg.height = conv.nHeight;
    msg.width = conv.nWidth;
    // 从请求的pixel_format_解析编码
    auto it2 = ros_encoding_map_.find(pixel_format_);
    if (it2 != ros_encoding_map_.end()) {
      msg.encoding = it2->second;
    } else {
      // 回退：使用dst_type的编码
      std::string enc = ros_encoding_for_sdk(conv.enDstPixelType);
      msg.encoding = enc.empty() ? "bgr8" : enc;
    }
    msg.is_bigendian = 0;
    msg.step = conv.nWidth * dst_bpp;
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
