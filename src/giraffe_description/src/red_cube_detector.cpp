#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cmath>

class RedCubeDetector : public rclcpp::Node
{
public:
    RedCubeDetector() : Node("red_cube_detector"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        // Subscribe to camera info
        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/overhead_camera/camera_info",
            10,
            std::bind(&RedCubeDetector::camera_info_callback, this, std::placeholders::_1));

        // Subscribe to overhead camera image
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/overhead_camera/image",
            10,
            std::bind(&RedCubeDetector::image_callback, this, std::placeholders::_1));

        // Subscribe to depth image
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/overhead_camera/depth_image",
            10,
            std::bind(&RedCubeDetector::depth_callback, this, std::placeholders::_1));

        // Publisher for cube position in base_link frame
        position_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/overhead_camera/red_cube_position",
            10);

        // Publisher for moveit compatible coordinates
        moveit_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/red_cube/moveit_target",
            10);

        RCLCPP_INFO(this->get_logger(), "Red Cube Detector started with depth integration");
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Store camera intrinsics
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        fx_ = msg->k[0];
        fy_ = msg->k[4];
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert depth image to CV Mat (depth is typically float32)
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            latest_depth_ = depth_ptr->image.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to convert depth image: %s", e.what());
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            // Convert BGR to HSV for better red detection
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            // Define range for red color in HSV
            cv::Mat mask1, mask2, mask;

            // Lower red (0-10)
            cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);

            // Upper red (170-180)
            cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);

            // Combine masks
            mask = mask1 | mask2;

            // Morphological operations to reduce noise
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask.clone(), contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                // Find the largest contour (likely the red cube)
                int largest_contour_idx = 0;
                double largest_area = 0;

                for (size_t i = 0; i < contours.size(); ++i)
                {
                    double area = cv::contourArea(contours[i]);
                    if (area > largest_area)
                    {
                        largest_area = area;
                        largest_contour_idx = i;
                    }
                }

                // Only process if contour is large enough
                if (largest_area > 100 && fx_ > 0 && fy_ > 0)
                {
                    // Calculate centroid
                    cv::Moments moments = cv::moments(contours[largest_contour_idx], true);
                    double pixel_x = moments.m10 / moments.m00;
                    double pixel_y = moments.m01 / moments.m00;

                    // Get depth at centroid
                    double depth_value = 0.0;
                    if (!latest_depth_.empty() &&
                        pixel_x >= 0 && pixel_x < latest_depth_.cols &&
                        pixel_y >= 0 && pixel_y < latest_depth_.rows)
                    {
                        depth_value = latest_depth_.at<float>(static_cast<int>(pixel_y), static_cast<int>(pixel_x));
                    }

                    // If depth is invalid (0 or NaN), estimate based on area
                    if (depth_value <= 0.01 || std::isnan(depth_value))
                    {
                        depth_value = 0.25; // Default estimated depth
                        RCLCPP_WARN(this->get_logger(), "Using default depth: %.3f", depth_value);
                    }

                    // Convert pixel + depth to 3D camera frame coordinates
                    double camera_x = (pixel_x - cx_) * depth_value / fx_;
                    double camera_y = (pixel_y - cy_) * depth_value / fy_;
                    double camera_z = depth_value;

                    // Create point in camera frame
                    geometry_msgs::msg::PointStamped camera_point;
                    camera_point.header.stamp = msg->header.stamp;
                    camera_point.header.frame_id = "overhead_camera_link";
                    camera_point.point.x = camera_x;
                    camera_point.point.y = camera_y;
                    camera_point.point.z = camera_z;

                    // Transform to base_link frame
                    geometry_msgs::msg::PointStamped base_link_point;
                    try
                    {
                        base_link_point = tf_buffer_.transform(camera_point, "base_link", std::chrono::milliseconds(100));
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                        return;
                    }

                    // Publish both representations
                    position_publisher_->publish(camera_point);
                    moveit_publisher_->publish(base_link_point);

                    RCLCPP_INFO(this->get_logger(),
                                "Red cube - pixel: (%.1f, %.1f), depth: %.4f m, "
                                "camera frame: (%.4f, %.4f, %.4f), "
                                "base_link: (%.4f, %.4f, %.4f)",
                                pixel_x, pixel_y, depth_value,
                                camera_x, camera_y, camera_z,
                                base_link_point.point.x, base_link_point.point.y, base_link_point.point.z);
                }
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr moveit_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Camera intrinsics
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    cv::Mat latest_depth_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedCubeDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
