#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

class BagRecorderNode : public rclcpp::Node
{
public:
  BagRecorderNode()
  : Node("bag_recorder_node")
  {
    // 自动生成 bag 文件夹名
    std::string bag_name = generateBagName();
    std::filesystem::create_directories("bags");

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_name;
    storage_options.storage_id = "sqlite3";

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options);

    writer_->create_topic({
      0u,
      "/livox/lidar",
      "sensor_msgs/msg/PointCloud2",
      rmw_get_serialization_format(),
      {},
      "",
    });
    writer_->create_topic({
      1u,
      "/img_for_lidar",
      "sensor_msgs/msg/Image",
      rmw_get_serialization_format(),
      {},
      "",
    });

    // 订阅
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&BagRecorderNode::lidarCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/rm_multi_cam/liradar_image", 10,
      std::bind(&BagRecorderNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "BagRecorderNode 已启动，录制 /livox/lidar 和 /img_for_lidar 到: %s",
                bag_name.c_str());
  }

private:
  std::string generateBagName()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local = *std::localtime(&t_c);

    std::ostringstream oss;
    oss << "bags/" << std::put_time(&tm_local, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
  }

  void lidarCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    auto time_stamp = this->now();
    writer_->write(msg,"/livox/lidar","sensor_msgs/msg/PointCloud2", time_stamp);
  }


  void imageCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    auto time_stamp = this->now();
    writer_->write(msg,"/img_for_lidar","sensor_msgs/msg/Image", time_stamp);
  }


  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BagRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
