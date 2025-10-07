
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class FrameRateMonitor : public rclcpp::Node {
public:
    FrameRateMonitor() : Node("frame_rate_monitor") {
        // 声明参数 - 从参数服务器获取设定帧率
        this->declare_parameter<double>("target_frame_rate", 30.0);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&FrameRateMonitor::image_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/camera/frame_rate", 10);
        
        // 参数回调，监听设定帧率的变化
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FrameRateMonitor::on_parameter_changed, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(1000ms, std::bind(&FrameRateMonitor::calculate_frame_rate, this));
        
        // 获取初始设定帧率
        target_frame_rate_ = this->get_parameter("target_frame_rate").as_double();
        
        RCLCPP_INFO(this->get_logger(), "帧率监控节点已启动，设定帧率: %.1f FPS", target_frame_rate_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        frame_count_++;
        last_frame_time_ = this->now();
    }

    void calculate_frame_rate() {
        auto current_time = this->now();
        auto elapsed = (current_time - last_calc_time_).seconds();
        
        if (elapsed > 0) {
            float actual_fps = frame_count_ / elapsed;
            
            std_msgs::msg::Float32 fps_msg;
            fps_msg.data = actual_fps;
            publisher_->publish(fps_msg);
            
            // 简化输出：只显示实际帧率和设定帧率，去掉效率计算
            RCLCPP_INFO(this->get_logger(), 
                       "帧率统计: 实际=%.1f FPS, 设定=%.1f FPS", 
                       actual_fps, target_frame_rate_);
            
            frame_count_ = 0;
            last_calc_time_ = current_time;
        }
    }

    // 参数回调函数，监听设定帧率的变化
    rcl_interfaces::msg::SetParametersResult on_parameter_changed(
        const std::vector<rclcpp::Parameter> &parameters) {
        
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto &param : parameters) {
            if (param.get_name() == "target_frame_rate") {
                double new_target_fps = param.as_double();
                target_frame_rate_ = new_target_fps;
                RCLCPP_INFO(this->get_logger(), "监控节点: 设定帧率更新为 %.1f FPS", new_target_fps);
            }
        }
        
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    int frame_count_ = 0;
    rclcpp::Time last_frame_time_;
    rclcpp::Time last_calc_time_ = this->now();
    double target_frame_rate_ = 30.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameRateMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
