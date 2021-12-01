#ifndef LMC_CAMERA_NODE_HPP
#define LMC_CAMERA_NODE_HPP

#include <string.h>
#include <boost/shared_ptr.hpp>
#include <sstream>

#include "Leap.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#define targetWidth 500
#define targetHeight 500
#define cutWidth 280
#define cutHeight 220
#define startX 110
#define startY 140

using namespace Leap;
using namespace std;
using namespace std::chrono_literals;

class CameraListener : public Listener, public rclcpp::Node {
  public:
    CameraListener();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image_left;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _pub_info_left;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image_right;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _pub_info_right;
    image_transport::CameraPublisher _left_image_pub_;
    image_transport::CameraPublisher _right_image_pub_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> info_mgr_right;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_mgr_left;
    
    bool enable_controller_info = false;

    unsigned int seq;
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);  
};


#endif /* LMC_CAMERA_NODE_HPP */