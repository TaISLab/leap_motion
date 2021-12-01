#include <leap_motion/lmc_camera_node.hpp>

CameraListener::CameraListener(): Node("lmc_camera_node")
    {
        //nothing
    }

void CameraListener::onInit(const Controller& controller){
    std::string path;
    std::string default_l_info_filename;
    std::string default_r_info_filename;
    std::string l_info_filename;
    std::string r_info_filename;

    // _pub_image_left = this->create_publisher<sensor_msgs::msg::Image>("image_raw_left", 1);
    // _pub_image_right = this->create_publisher<sensor_msgs::msg::Image>("image_raw_right", 1);

    // _pub_info_left = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_left", 1);
    // _pub_info_right = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_right", 1);

    _left_image_pub_ = image_transport::create_camera_publisher(this, "left/image_rect", rclcpp::QoS{100}.get_rmw_qos_profile());
    _right_image_pub_ = image_transport::create_camera_publisher(this, "right/image_rect", rclcpp::QoS{100}.get_rmw_qos_profile());

    seq = 0;

    // Get default config file
	// ament may throw PackageNotFoundError exception
	path = ament_index_cpp::get_package_share_directory("leap_motion");
        
    default_l_info_filename = path + std::string("/config/camera_info/leap_cal_left.yml");
    default_r_info_filename = path + std::string("/config/camera_info/leap_cal_right.yml");
    
    // See if there is a proper param for it
    this->declare_parameter<std::string>("template_filename_left", default_l_info_filename);
    this->declare_parameter<std::string>("template_filename_right", default_r_info_filename);

    this->get_parameter("template_filename_left", l_info_filename);
    this->get_parameter("template_filename_right", r_info_filename);  
    
    l_info_filename = std::string("file://") + l_info_filename;
    r_info_filename = std::string("file://") + r_info_filename;
    
    info_mgr_left.reset(new camera_info_manager::CameraInfoManager(this, "left", l_info_filename));
    info_mgr_right.reset(new camera_info_manager::CameraInfoManager(this, "right", r_info_filename));

    if(CameraListener::enable_controller_info){  
        RCLCPP_INFO(this->get_logger(), "CameraListener initialized");
    }

    // check for default camera info
    if (!info_mgr_left->isCalibrated()) {
        RCLCPP_WARN(this->get_logger(), "CameraListener left camera not calibrated");
    }
    if (!info_mgr_right->isCalibrated()) {
        RCLCPP_WARN(this->get_logger(), "CameraListener right camera not calibrated");
    }
}

void CameraListener::onConnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        RCLCPP_INFO(this->get_logger(), "CameraListener connected");
    }
}

void CameraListener::onDisconnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {
        RCLCPP_INFO(this->get_logger(), "CameraListener disconnected");
    }
}

void CameraListener::onExit(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        RCLCPP_INFO(this->get_logger(), "CameraListener exited");
    }
}

void CameraListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();  
  // The list of IR images from the Leap Motion cameras. 
  ImageList images = frame.images();
  // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
  sensor_msgs::msg::Image image_msg;

  // no need for seq field
  //image_msg.header.seq = seq++;
  image_msg.header.stamp = this->get_clock()->now();
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  image_msg.encoding = "mono8";
  image_msg.is_bigendian = 0;
  
  for(int camera_num = 0; camera_num < 2; camera_num++){
    Image image = images[camera_num];
    
    // image width, that is, number of columns
    image_msg.width =  cutWidth;
    // image height, that is, number of rows
    image_msg.height = cutHeight;
    // Full row length in bytes
    image_msg.step = cutWidth;
    image_msg.data.resize(cutWidth * cutHeight);

    // The parallel construct forms a team of threads and starts parallel execution.
    // The loop construct specifies that the iterations of loops will be distributed 
    // among and executed by the encountering team of threads.
    #pragma omp parallel for
    
    for(int i = 0; i < cutWidth; i++)
    {
      for(int j = 0; j < cutHeight; j++)
      {
        Vector input = Vector((float)(i + startX)/targetWidth, (float)(j + startY)/targetHeight, 0);
        input.x = (input.x - image.rayOffsetX()) / image.rayScaleX();
        input.y = (input.y - image.rayOffsetY()) / image.rayScaleY();

        Vector pixel = image.warp(input);
        if(pixel.x >= 0 && pixel.x < image.width() && pixel.y >= 0 && pixel.y < image.height()) 
        {
          int data_index = floor(pixel.y) * image.width() + floor(pixel.x);
          image_msg.data[cutWidth*j+i] = image.data()[data_index];
        } 
        else
          image_msg.data[cutWidth*j+i] = 0;
      }
    }

    if(camera_num == 0)
    {
      //sensor_msgs::msg::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_left -> getCameraInfo() ) );
      
      image_msg.header.frame_id = "leap_camera_left"; // = info_msg->header.frame_id 
      // info_msg->width = image_msg.width;
      // info_msg->height = image_msg.height;
      // info_msg->header.stamp = image_msg.header.stamp;
      // info_msg->header.seq = image_msg.header.seq;
      // _pub_image_left.publish(image_msg);
      // _pub_info_left.publish(*info_msg);
      auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(info_mgr_left->getCameraInfo());
      ci->header =  image_msg.header;
      _left_image_pub_.publish(image_msg, *ci);

    }
    else
    {
      //sensor_msgs::msg::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_right->getCameraInfo()));
      image_msg.header.frame_id  = "leap_camera_right"; // = info_msg->header.frame_id
      // info_msg->width = image_msg.width;
      // info_msg->height = image_msg.height;
      // info_msg->header.stamp = image_msg.header.stamp;
      //info_msg->header.seq = image_msg.header.seq;
      //_pub_image_right.publish(image_msg);
      //_pub_info_right.publish(*info_msg);
      auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(info_mgr_right->getCameraInfo());
      ci->header =  image_msg.header;
      _right_image_pub_.publish(image_msg, *ci);

    }
  }
}

void CameraListener::onFocusGained(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        RCLCPP_INFO(this->get_logger(), "CameraListener gained focus");
    }
}

void CameraListener::onFocusLost(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        RCLCPP_INFO(this->get_logger(), "CameraListener lost focus");
    }
}

void CameraListener::onDeviceChange(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        RCLCPP_INFO(this->get_logger(), "CameraListener device changed");
        const DeviceList devices = controller.devices();
        for (int i = 0; i < devices.count(); ++i) {
            RCLCPP_INFO(this->get_logger(),  "id: %s", devices[i].toString().c_str() );
            RCLCPP_INFO(this->get_logger(), "  isStreaming: %s", (devices[i].isStreaming() ? "true" : "false") );
        }
    }
}

void CameraListener::onServiceConnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {
        RCLCPP_INFO(this->get_logger(), "CameraListener service connected");
    }
}

void CameraListener::onServiceDisconnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {
        RCLCPP_INFO(this->get_logger(), "CameraListener service disconnected");
    }
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto listener_ptr = std::make_shared<CameraListener>();

  Controller controller;
  controller.addListener(*listener_ptr);
  controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
  
  rclcpp::spin(listener_ptr);
  controller.removeListener(*listener_ptr);
  rclcpp::shutdown();

  return 0;
}
