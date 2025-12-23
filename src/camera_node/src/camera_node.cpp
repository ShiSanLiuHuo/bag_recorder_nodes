#include "camera/camera_node.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>

namespace sensor {

CameraNode::CameraNode(const rclcpp::NodeOptions& options):
    Node("camera_node", options),
    frame_(std::make_shared<cv::Mat>()),
    failed_count(0) {
    RCLCPP_INFO(this->get_logger(), "camera_node start");

    // 是否使用视频流标志位
    videoflag = this->declare_parameter("videoflag", false);
    video_path = this->declare_parameter("video_path", "/home/phoenix/zk/save_stuff/20.mp4"); //默认路径
    rosbag_flag = this->declare_parameter("rosbag_flag", false);
    inner_shot_flag = this->declare_parameter("inner_shot_flag", false);
    exposure_time = this->declare_parameter("exposure_time", 30000);
    gain = this->declare_parameter("gain", 64);
    rosbag_flag = this->get_parameter("rosbag_flag").as_bool();

    RCLCPP_INFO(this->get_logger(), "inner_shot flag %d", inner_shot_flag);

    mindvision_ = std::make_shared<MindVision>(
        ament_index_cpp::get_package_share_directory("camera") + "../config/mindvision.config",
        this->declare_parameter("sn", "").c_str()
    );
    if (rosbag_flag) {
        RCLCPP_INFO(this->get_logger(), "rosbag");
        return;
    }
    if (videoflag) {
        capture.open(video_path);
        if (!capture.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "video open failed");
            exit(-1);
        }
        RCLCPP_INFO(this->get_logger(), "use video");
    } else if (!mindvision_->GetCameraStatus()) {
        RCLCPP_ERROR(this->get_logger(), "mindvision failed");
        exit(-1);
    }

    mindvision_->SetExposureTime(exposure_time);

    if (inner_shot_flag) {
        thread_for_inner_shot_ = std::thread(std::bind(&CameraNode::InnerShot, this));
    }

    img_pub_for_radar_ =
        this->create_publisher<sensor_msgs::msg::Image>("/image_for_radar", rclcpp::SensorDataQoS().keep_last(10));

    // if (!videoflag) {
    //     CameraNode::SetExposureTimeWithTrack();
    // }
    // if (!videoflag) {
    //     CameraNode::TuneExposure();
    // }

    double exposure_time_value;
    mindvision_->GetExposureTime(exposure_time_value);
    RCLCPP_DEBUG(this->get_logger(), "exposure_time: %f", exposure_time_value);

    thread_for_publish_ = std::thread(std::bind(&CameraNode::LoopForPublish, this));

    std::thread timerTread(std::bind(&CameraNode::timer, this));
    timerTread.detach();
}

void CameraNode::InnerShot() {
    auto inner_shot = std::make_shared<InnerShotNode>();
    RCLCPP_INFO(this->get_logger(), "inner_shot start !.............. ");
    rclcpp::spin(inner_shot);
}

void CameraNode::GetImg() {
    if (videoflag) {
        capture >> *frame_;
        // usleep(100000);

        // 循环播放
        if ((*frame_).empty()) {
            RCLCPP_INFO(this->get_logger(), "video end");
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            capture >> *frame_;
        }
    } else {
        if (!mindvision_->GetFrame(frame_)) {
            failed_count++;
            RCLCPP_ERROR(this->get_logger(), "mindvision get image failed");
        } else {
            frame_count_++;
            // RCLCPP_DEBUG(this->get_logger(), "mindvision get image success. Size: %d
            // x %d", frame_->cols, frame_->rows);
            failed_count = 0;
        }
    }

    if (failed_count > 10) {
        exit(-1);
    }
}

// void CameraNode::ExposureCallback(){
//     // if papam changed, save to file
//     if (this->has_parameter("exposure_time")) {
//         int exposure_time = this->get_parameter("exposure_time").as_int();
//         mindvision_->SetExposureTime(exposure_time);
//     }
//     if (this->has_parameter("gain")) {
//         int gain = this->get_parameter("gain").as_int();
//         mindvision_->SetGain(gain);
//     }
// }

void CameraNode::TuneExposure() {
    CameraNode::GetImg();
    exposure_time = this->get_parameter("exposure_time").as_int();
    gain = this->get_parameter("gain").as_int();

    // Setup the tune window and trackbars
    cv::namedWindow("tune", cv::WINDOW_NORMAL);

    cv::createTrackbar(
        "gain",
        "tune",
        NULL,
        128,
        [](int value, void* ptr) {
            auto mindvision = static_cast<MindVision*>(ptr);
            mindvision->SetGain(value);
        },
        mindvision_.get()
    );

    while (cv::waitKey(1) != 'q') {
        CameraNode::GetImg();
        cv::resize(*frame_, *frame_, cv::Size(640, 480));
        cv::imshow("tune", *frame_);
    }
    cv::destroyAllWindows();
}

void CameraNode::SetExposureTimeWithTrack() {
    // Setup the tune window and trackbars
    cv::namedWindow("exposure_tune", cv::WINDOW_NORMAL);

    double exposure_time_value;
    mindvision_->GetExposureTime(exposure_time_value);

    int exposure_time_int = static_cast<int>(exposure_time_value);

    // 设备允许的曝光时间在0.008~10284.8ms之间，代码中单位为us
    cv::createTrackbar(
        "exposure_time",
        "exposure_tune",
        &exposure_time_int,
        100000,
        [](int value, void* ptr) {
            auto mindvision = static_cast<MindVision*>(ptr);
            mindvision->SetExposureTime(value);
            std::cout << "Set exposure_time: " << value << std::endl;
        },
        mindvision_.get()
    );

    while (cv::waitKey(1) != 'q') {
        CameraNode::GetImg();
        cv::resize(*frame_, *frame_, cv::Size(640, 480));
        cv::imshow("exposure_tune", *frame_);
    }

    cv::destroyAllWindows();

    RCLCPP_INFO(this->get_logger(), "Set initial exposure_time: %d", exposure_time_int);
}

void CameraNode::LoopForPublish() {
    while (rclcpp::ok()) {
        this->GetImg();

        if (frame_->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame, skip publishing.");
            continue;
        }

        // 发布全图
        {
            if (debug_exposure) {
                double exposure_time_value;
                mindvision_->GetExposureTime(exposure_time_value);
                RCLCPP_DEBUG(this->get_logger(), "exposure_time: %f", exposure_time_value);
            }
            auto full_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *frame_).toImageMsg();
            full_msg->header.stamp = this->now();
            full_msg->header.frame_id = this->enemy_color_flag_;
            img_pub_for_radar_->publish(*full_msg);
        }
    }
}

void CameraNode::timer() {
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int count = frame_count_.exchange(0);
        std::cout << "Camera FPS: " << count << std::endl;
    }
}

} // namespace sensor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor::CameraNode)
