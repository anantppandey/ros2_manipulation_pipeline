#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class RGBDCubeDetector : public rclcpp::Node
{
public:
    RGBDCubeDetector()
    : Node("rgbd_cube_detector")
    {
        // Create tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Subscribe to camera info to get intrinsics
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/overhead_camera/camera_info",
            10,
            std::bind(&RGBDCubeDetector::cameraInfoCallback, this, std::placeholders::_1));
        
        // Subscribers for RGB and depth
        rgb_sub_.subscribe(this, "/overhead_camera/image");
        depth_sub_.subscribe(this, "/overhead_camera/depth_image");

        // Publisher for detected cube pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_pose",10);
        
        // Sync RGB and depth
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&RGBDCubeDetector::rgbdCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));
        
        cv::namedWindow("Overhead Camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("Overhead Camera", 800, 600);
        
        RCLCPP_INFO(this->get_logger(), "RGBD Cube Detector Started");
        RCLCPP_INFO(this->get_logger(), "Camera frame: overhead_camera_link");
        RCLCPP_INFO(this->get_logger(), "Waiting for camera data...");
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Get camera intrinsics from topic
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        
        RCLCPP_INFO_ONCE(this->get_logger(), 
                         "Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                         fx_, fy_, cx_, cy_);
        
        // Unsubscribe after getting once
        camera_info_sub_.reset();
    }
    
    void pixelToCamera(float u, float v, float& x_cam, float& y_cam)
    {
        // Image center (320x240)
        const float CX = 160.0;
        const float CY = 120.0;
        const float NORM = 120.0;  // Use height for normalization
        
        // Normalize with center at (0,0), y flipped
        float nx = (u - CX) / NORM;
        float ny = -(v - CY) / NORM;
        
        // Recalibrated corners from two known points:
        // Point 1: (197, 63) -> (0.15, 0.40)
        // Point 2: (235, 100) -> (0.05, 0.30)
        const float X_MIN = 0.5632;
        const float X_MAX = -0.0684;
        const float Y_MIN = -0.0784;
        const float Y_MAX = 0.5703;
        
        // Map normalized to camera coordinates
        x_cam = X_MIN + ((nx + 1.0) / 2.0) * (X_MAX - X_MIN);
        y_cam = Y_MIN + ((ny + 1.0) / 2.0) * (Y_MAX - Y_MIN);
    }
    
    // ------------------------------------------------------------------
    // NEW: shared helper for the blue/yellow/box detections below.
    // Takes a detected pixel, applies the SAME pixelToCamera() + world
    // transform logic already used for the red cube, and just prints
    // the resulting values (no publishing yet, per your request).
    // The red cube block further down is untouched and still publishes
    // to /cube_pose on its own.
    // ------------------------------------------------------------------
    void reportDetection(const std::string& label,
                          const cv::Point2f& pixel,
                          const cv::Mat& depth,
                          cv::Mat& frame,
                          const cv::Scalar& draw_color,
                          const std::string& emoji_tag)
    {
        if (pixel.x <= 0 || pixel.y <= 0) {
            return;
        }

        int u = (int)pixel.x;
        int v = (int)pixel.y;

        if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) {
            return;
        }

        float depth_value = depth.at<float>(v, u);
        if (depth_value <= 0.05 || depth_value >= 5.0) {
            return;
        }

        // Same pixel -> camera -> world mapping as the red cube
        float x_cam, y_cam;
        pixelToCamera(u, v, x_cam, y_cam);
        float z_cam = depth_value;

        float world_x = 0.25 - x_cam;
        float world_y = y_cam;
        float world_z = 0.75 - z_cam + 0.04;

        RCLCPP_INFO(this->get_logger(), "Pixel: (%d, %d), Depth: %.3fm", u, v, depth_value);
        RCLCPP_INFO(this->get_logger(), "%s in CAMERA frame: (%.3f, %.3f, %.3f) meters",
                   label.c_str(), x_cam, y_cam, z_cam);
        RCLCPP_INFO(this->get_logger(), "%s %s in WORLD frame: (%.3f, %.3f, %.3f) meters",
                   emoji_tag.c_str(), label.c_str(), world_x, world_y, world_z);

        // Crosshair at the exact detected pixel so you can see where the
        // code thinks the cube/marker is, not just the printed values
        cv::drawMarker(frame, pixel, draw_color, cv::MARKER_CROSS, 16, 2);
        cv::putText(frame, label,
                   cv::Point(pixel.x - 40, pixel.y - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2);
    }
    
    void rgbdCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        try {
            // Convert RGB image
            cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8");
            cv::Mat frame = cv_rgb->image;
            
            // Convert depth image
            cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg);
            cv::Mat depth = cv_depth->image;
            
            // Detect red cube center in RGB
            cv::Point2f cube_pixel = detectRedCube(frame);
            
            if (cube_pixel.x > 0 && cube_pixel.y > 0) {
                // Get depth at cube center
                int u = (int)cube_pixel.x;
                int v = (int)cube_pixel.y;
                
                if (u >= 0 && u < depth.cols && v >= 0 && v < depth.rows) {
                    float depth_value = depth.at<float>(v, u);
                    
                    if (depth_value > 0.05 && depth_value < 5.0) {
                        // Get camera frame position using recalibrated mapping
                        float x_cam, y_cam;
                        pixelToCamera(u, v, x_cam, y_cam);
                        float z_cam = depth_value;
                        
                        RCLCPP_INFO(this->get_logger(), "Pixel: (%d, %d), Depth: %.3fm", u, v, depth_value);
                        RCLCPP_INFO(this->get_logger(), "Cube in CAMERA frame: (%.3f, %.3f, %.3f) meters", 
                                   x_cam, y_cam, z_cam);
                        
                        // === SIMPLE TRANSFORM: Camera at (0.25, 0, 0.75) ===
                        // world_x = 0.25 - x_cam  (camera looks in -X direction)
                        // world_y = y_cam + 0.0045  (y offset correction)
                        // world_z = 0.75 - z_cam  (camera looks down)
                        
                        float world_x = 0.25 - x_cam;
                        float world_y = y_cam ;
                        float world_z = 0.75 - z_cam + 0.04;  // small offset for cube height

                        geometry_msgs::msg::PoseStamped pose;

                        pose.header.stamp = this->now();
                        pose.header.frame_id = "world";

                        pose.pose.position.x = world_x;
                        pose.pose.position.y = world_y;
                        pose.pose.position.z = world_z;
                        // Orientation 
                        pose.pose.orientation.x = -0.59;
                        pose.pose.orientation.y = 0.81;
                        pose.pose.orientation.z = -0.02;
                        pose.pose.orientation.w = -0.02;

                        //     target.orientation.x = -0.59;
                        //     target.orientation.y = 0.81;
                        //     target.orientation.z = -0.02;
                        //     target.orientation.w = -0.02;

                        pose_pub_->publish(pose);
                        
                        RCLCPP_INFO(this->get_logger(), "🎯 Cube in WORLD frame: (%.3f, %.3f, %.3f) meters", 
                                   world_x, world_y, world_z);
                        
                        // Print transform values for MoveIt
                        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                        RCLCPP_INFO(this->get_logger(), "📦 TRANSFORM TO MOVEIT:");
                        RCLCPP_INFO(this->get_logger(), "  Position:");
                        RCLCPP_INFO(this->get_logger(), "    x: %.3f", world_x);
                        RCLCPP_INFO(this->get_logger(), "    y: %.3f", world_y);
                        RCLCPP_INFO(this->get_logger(), "    z: %.3f", world_z);
                        RCLCPP_INFO(this->get_logger(), "  Orientation (for grasping straight down):");
                        RCLCPP_INFO(this->get_logger(), "    x: 0.000");
                        RCLCPP_INFO(this->get_logger(), "    y: 0.000");
                        RCLCPP_INFO(this->get_logger(), "    z: 0.000");
                        RCLCPP_INFO(this->get_logger(), "    w: 1.000");
                        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                        
                        // Draw detection on frame - crosshair + name
                        cv::drawMarker(frame, cube_pixel, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 16, 2);
                        cv::putText(frame, "RED CUBE", 
                                   cv::Point(cube_pixel.x - 40, cube_pixel.y - 15),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }

            // ================================================================
            // Blue / yellow cube + collection box detection - same
            // pixel -> depth -> pixelToCamera -> world logic as the red
            // cube above, routed through reportDetection() so each one
            // prints its values and draws a crosshair + name.
            // ================================================================

            cv::Point2f blue_pixel = detectBlueCube(frame);
            reportDetection("BLUE CUBE", blue_pixel, depth, frame, cv::Scalar(255, 0, 0), "🔵");

            cv::Point2f yellow_pixel = detectYellowCube(frame);
            reportDetection("YELLOW CUBE", yellow_pixel, depth, frame, cv::Scalar(0, 255, 255), "🟡");

            cv::Point2f box_pixel = detectCollectionBox(frame);
            reportDetection("COLLECTION BOX", box_pixel, depth, frame, cv::Scalar(255, 0, 255), "❌");
            
            cv::imshow("Overhead Camera", frame);
            cv::waitKey(1);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }
    
    cv::Point2f detectRedCube(cv::Mat& frame)
    {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // Red color range
        cv::Scalar lower_red1(0, 100, 100);
        cv::Scalar upper_red1(10, 255, 255);
        cv::Scalar lower_red2(160, 100, 100);
        cv::Scalar upper_red2(179, 255, 255);
        
        cv::Mat mask1, mask2;
        cv::inRange(hsv, lower_red1, upper_red1, mask1);
        cv::inRange(hsv, lower_red2, upper_red2, mask2);
        
        cv::Mat red_mask = mask1 | mask2;
        
        // Morphological cleanup
        cv::erode(red_mask, red_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::dilate(red_mask, red_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (!contours.empty()) {
            int largest_idx = 0;
            double largest_area = cv::contourArea(contours[0]);
            
            for (size_t i = 1; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > largest_area) {
                    largest_area = area;
                    largest_idx = i;
                }
            }
            
            if (largest_area > 100) {  // Minimum area threshold
                cv::Moments m = cv::moments(contours[largest_idx]);
                if (m.m00 != 0) {
                    return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                }
            }
        }
        
        return cv::Point2f(-1, -1);
    }

    // ------------------------------------------------------------------
    // NEW: blue cube.
    // Gazebo material ambient/diffuse (0.1, 0.25, 0.8) -> ~RGB(26,64,204)
    // -> ~HSV(114, 223, 204) in OpenCV's 0-179/0-255/0-255 scale.
    // Range below is padded around that estimate; tune if the overhead
    // camera's lighting shifts the actual rendered color.
    // ------------------------------------------------------------------
    cv::Point2f detectBlueCube(cv::Mat& frame)
    {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_blue(100, 100, 60);
        cv::Scalar upper_blue(130, 255, 255);

        cv::Mat blue_mask;
        cv::inRange(hsv, lower_blue, upper_blue, blue_mask);

        cv::erode(blue_mask, blue_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::dilate(blue_mask, blue_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            int largest_idx = 0;
            double largest_area = cv::contourArea(contours[0]);

            for (size_t i = 1; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > largest_area) {
                    largest_area = area;
                    largest_idx = i;
                }
            }

            if (largest_area > 100) {
                cv::Moments m = cv::moments(contours[largest_idx]);
                if (m.m00 != 0) {
                    return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                }
            }
        }

        return cv::Point2f(-1, -1);
    }

    // ------------------------------------------------------------------
    // NEW: yellow cube.
    // Gazebo material ambient/diffuse (0.85, 0.72, 0.18) -> ~RGB(217,184,46)
    // -> ~HSV(24, 201, 217). Range below is padded around that estimate.
    // ------------------------------------------------------------------
    cv::Point2f detectYellowCube(cv::Mat& frame)
    {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower_yellow(15, 100, 100);
        cv::Scalar upper_yellow(35, 255, 255);

        cv::Mat yellow_mask;
        cv::inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

        cv::erode(yellow_mask, yellow_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::dilate(yellow_mask, yellow_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            int largest_idx = 0;
            double largest_area = cv::contourArea(contours[0]);

            for (size_t i = 1; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > largest_area) {
                    largest_area = area;
                    largest_idx = i;
                }
            }

            if (largest_area > 100) {
                cv::Moments m = cv::moments(contours[largest_idx]);
                if (m.m00 != 0) {
                    return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                }
            }
        }

        return cv::Point2f(-1, -1);
    }

    // ------------------------------------------------------------------
    // Collection box - two-stage approach.
    // Stage 1: threshold black to find roughly where the box is. The
    // box's bottom plate and its walls can show up as separate contours
    // (and a wall's visible face can out-area the flat top plate), so
    // taking "the largest black contour" was landing on a wall instead
    // of the box center. Fixed by taking the UNION of every sizeable
    // dark contour's bounding box instead of just the single largest one
    // - that reliably covers the whole box regardless of which part
    // happens to be biggest.
    // Stage 2: within that region only, look for the white X marker and
    // use its centroid as the final point - this also keeps the white
    // arm (which sits elsewhere in the frame) from being mistaken for
    // the marker, since we only search inside the box's black region.
    // Falls back to the black region's center if no white X is visible
    // in it (e.g. arm briefly occluding the marker).
    // ------------------------------------------------------------------
    cv::Point2f detectCollectionBox(cv::Mat& frame)
    {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // --- Stage 1: locate the box region via black ---
        cv::Scalar lower_black(0, 0, 0);
        cv::Scalar upper_black(179, 255, 50);

        cv::Mat black_mask;
        cv::inRange(hsv, lower_black, upper_black, black_mask);

        cv::erode(black_mask, black_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::dilate(black_mask, black_mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        std::vector<std::vector<cv::Point>> black_contours;
        cv::findContours(black_mask, black_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool found_black = false;
        cv::Rect box_roi;

        for (const auto& c : black_contours) {
            if (cv::contourArea(c) > 100) {
                cv::Rect r = cv::boundingRect(c);
                box_roi = found_black ? (box_roi | r) : r;
                found_black = true;
            }
        }

        if (!found_black) {
            return cv::Point2f(-1, -1);
        }

        // --- Stage 2: find the white X marker, but only inside box_roi ---
        cv::Scalar lower_white(0, 0, 200);
        cv::Scalar upper_white(179, 40, 255);

        cv::Mat white_mask;
        cv::inRange(hsv, lower_white, upper_white, white_mask);

        cv::Mat roi_mask = cv::Mat::zeros(white_mask.size(), CV_8UC1);
        white_mask(box_roi).copyTo(roi_mask(box_roi));

        std::vector<std::vector<cv::Point>> white_contours;
        cv::findContours(roi_mask, white_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!white_contours.empty()) {
            int largest_idx = 0;
            double largest_area = cv::contourArea(white_contours[0]);

            for (size_t i = 1; i < white_contours.size(); i++) {
                double area = cv::contourArea(white_contours[i]);
                if (area > largest_area) {
                    largest_area = area;
                    largest_idx = i;
                }
            }

            if (largest_area > 20) {
                cv::Moments m = cv::moments(white_contours[largest_idx]);
                if (m.m00 != 0) {
                    return cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                }
            }
        }

        // Fallback: no white X visible inside the box region right now -
        // use the black region's center instead of failing outright.
        return cv::Point2f(box_roi.x + box_roi.width / 2.0f,
                            box_roi.y + box_roi.height / 2.0f);
    }
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    // Camera intrinsics
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDCubeDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}