#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>

class BagRecorderNode : public rclcpp::Node
{
public:
  BagRecorderNode(const rclcpp::NodeOptions& options)
      : Node("bag_recorder_node", options)
  {
    // 自动生成 bag 文件夹名
    std::string bag_name = generateBagName();
    std::filesystem::create_directories("bags");

    // 初始化 rosbag2 writer
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_name;
    storage_options.storage_id = "sqlite3";

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options);

    // 创建 topic
    rosbag2_storage::TopicMetadata lidar_topic;
    lidar_topic.name = "/livox/lidar";
    lidar_topic.type = "sensor_msgs/msg/PointCloud2";
    lidar_topic.serialization_format = "cdr";
    writer_->create_topic(lidar_topic);

    rosbag2_storage::TopicMetadata image_topic;
    image_topic.name = "/image_for_radar";
    image_topic.type = "sensor_msgs/msg/Image";
    image_topic.serialization_format = "cdr";
    writer_->create_topic(image_topic);

    // 订阅选项，启用 intra-process
    rclcpp::SubscriptionOptions options_sub;
    options_sub.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    // 订阅 LiDAR
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar",
        rclcpp::SensorDataQoS(),
        std::bind(&BagRecorderNode::lidarCallback, this, std::placeholders::_1),
        options_sub);

    // 订阅 Image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_for_radar",
        rclcpp::SensorDataQoS(),
        std::bind(&BagRecorderNode::imageCallback, this, std::placeholders::_1),
        options_sub);

      rclcpp::on_shutdown([this]() {
        this->stopRecording();
    });

    RCLCPP_INFO(this->get_logger(),
                "BagRecorderNode 已启动，录制 /livox/lidar 和 /image_for_radar 到: %s",
                bag_name.c_str());
  }

  ~BagRecorderNode(){
    if(writer_) {
        writer_->close(); // 显式关闭，确保 metadata.yaml 生成
        writer_.reset();
    }
  }

  void stopRecording()
{
    if (writer_) {
        RCLCPP_INFO(this->get_logger(), "Closing bag writer...");
        writer_->close();  // 确保 flush + 生成 metadata.yaml
        writer_.reset();
    }
}



private:
  // 生成 bag 文件夹名
  std::string generateBagName()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local = *std::localtime(&t_c);

    std::ostringstream oss;
    oss << "bags/" << std::put_time(&tm_local, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
  }

  // LiDAR 回调
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    serializer.serialize_message(msg.get(), &serialized);

    writer_->write(serialized, "/livox/lidar", "sensor_msgs/msg/PointCloud2", now());
    RCLCPP_INFO(this->get_logger(), "已录制一帧 LiDAR 数据");
  }

  // Image 回调
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    rclcpp::SerializedMessage serialized;
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    serializer.serialize_message(msg.get(), &serialized);

    writer_->write(serialized, "/image_for_radar", "sensor_msgs/msg/Image", now());
    RCLCPP_INFO(this->get_logger(), "已录制一帧 Image 数据");
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

// 注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BagRecorderNode)
