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

// NOTE the "3_d" instead of "3d" - this is how rosidl names the generated
// header for anything with "3D" in it (Detection3D -> detection3_d.hpp).
#include "vision_msgs/msg/detection3_d_array.hpp"

// ==========================================
// OBJECT DETECTOR (multi-object version)
//
// Same camera subscriptions, same pixelToCamera() mapping, and the exact
// same four detectXxxCube()/detectCollectionBox() functions as
// red_cube_detector.cpp - none of that changed. What's different:
//
//   - Instead of printing values (or, for red, publishing one hardcoded
//     PoseStamped on /cube_pose), every object detected THIS FRAME is
//     packed into one vision_msgs/Detection3DArray and published on
//     /object_detections. Each entry carries a class_id string
//     ("red_cube", "blue_cube", "yellow_cube", "collection_box") plus its
//     world-frame pose - that class_id is what the orchestrator's
//     target_object parameter gets matched against.
//   - /cube_pose is gone. If anything else still subscribes to it
//     directly, it needs to move over to /object_detections.
//   - The old red-cube-only "TRANSFORM TO MOVEIT" console block is gone
//     too - it was printing exactly what's now in the published message,
//     just less usefully (as log lines instead of structured data).
//   - Detection orientation is left as identity (0,0,0,1) for all four
//     objects. mtc_node computes its own grasp orientation from scratch
//     via IK regardless of what's in pick_pose.orientation, so this
//     isn't a gap - there's just nothing meaningful to put there yet.
// ==========================================
class ObjectDetector : public rclcpp::Node
{
public:
    ObjectDetector()
    : Node("object_detector")
    {
        // Create tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to camera info to get intrinsics
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/overhead_camera/camera_info",
            10,
            std::bind(&ObjectDetector::cameraInfoCallback, this, std::placeholders::_1));

        // Subscribers for RGB and depth
        rgb_sub_.subscribe(this, "/overhead_camera/image");
        depth_sub_.subscribe(this, "/overhead_camera/depth_image");

        // Publisher for ALL detected objects this frame
        detections_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "/object_detections", 10);

        // Sync RGB and depth
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&ObjectDetector::rgbdCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        cv::namedWindow("Overhead Camera", cv::WINDOW_NORMAL);
        cv::resizeWindow("Overhead Camera", 800, 600);

        RCLCPP_INFO(this->get_logger(), "Object Detector Started");
        RCLCPP_INFO(this->get_logger(), "Camera frame: overhead_camera_link");
        RCLCPP_INFO(this->get_logger(), "Publishing detections on /object_detections");
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
    // Pixel -> world position, using the exact same depth-lookup + camera
    // + world-frame math every detector in this file has always used.
    // Returns valid=false if the pixel/depth reading doesn't check out.
    // ------------------------------------------------------------------
    struct WorldPoint {
        bool valid = false;
        float x = 0.0f, y = 0.0f, z = 0.0f;
    };

    WorldPoint computeWorldPoint(const cv::Point2f& pixel, const cv::Mat& depth)
    {
        WorldPoint result;

        if (pixel.x <= 0 || pixel.y <= 0) return result;

        int u = (int)pixel.x;
        int v = (int)pixel.y;
        if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) return result;

        float depth_value = depth.at<float>(v, u);
        if (depth_value <= 0.05 || depth_value >= 5.0) return result;

        float x_cam, y_cam;
        pixelToCamera(u, v, x_cam, y_cam);
        float z_cam = depth_value;

        result.x = 0.25f - x_cam;
        result.y = y_cam;
        result.z = 0.75f - z_cam + 0.04f;
        result.valid = true;
        return result;
    }

    // ------------------------------------------------------------------
    // If the object was detected this frame, append it to the
    // Detection3DArray being built for this frame and draw its
    // crosshair + label. Does nothing if it wasn't found.
    // ------------------------------------------------------------------
    void addDetection(vision_msgs::msg::Detection3DArray& array,
                       const std::string& class_id,
                       const std::string& display_label,
                       const cv::Point2f& pixel,
                       const cv::Mat& depth,
                       cv::Mat& frame,
                       const cv::Scalar& draw_color)
    {
        WorldPoint p = computeWorldPoint(pixel, depth);
        if (!p.valid) return;

        vision_msgs::msg::Detection3D det;
        det.header.stamp = this->now();
        det.header.frame_id = "world";
        det.id = class_id;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = class_id;
        hyp.hypothesis.score = 1.0;
        hyp.pose.pose.position.x = p.x;
        hyp.pose.pose.position.y = p.y;
        hyp.pose.pose.position.z = p.z;
        hyp.pose.pose.orientation.w = 1.0;
        det.results.push_back(hyp);

        det.bbox.center = hyp.pose.pose;
        det.bbox.size.x = 0.03;
        det.bbox.size.y = 0.03;
        det.bbox.size.z = 0.03;

        array.detections.push_back(det);

        cv::drawMarker(frame, pixel, draw_color, cv::MARKER_CROSS, 16, 2);
        // cv::putText(frame, display_label,
        //            cv::Point(pixel.x - 40, pixel.y - 15),
        //            cv::FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2);
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

            vision_msgs::msg::Detection3DArray detections;
            detections.header.stamp = this->now();
            detections.header.frame_id = "world";

            addDetection(detections, "red_cube", "RED CUBE",
                         detectRedCube(frame), depth, frame, cv::Scalar(0, 255, 0));
            addDetection(detections, "blue_cube", "BLUE CUBE",
                         detectBlueCube(frame), depth, frame, cv::Scalar(255, 0, 0));
            addDetection(detections, "yellow_cube", "YELLOW CUBE",
                         detectYellowCube(frame), depth, frame, cv::Scalar(0, 255, 255));
            addDetection(detections, "collection_box", "COLLECTION BOX",
                         detectCollectionBox(frame), depth, frame, cv::Scalar(255, 0, 255));

            detections_pub_->publish(detections);

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
        cv::Scalar lower_red1(0, 150, 100);
        cv::Scalar upper_red1(8, 255, 255);
        cv::Scalar lower_red2(172, 150, 100);
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
    // Blue cube.
    // Gazebo material ambient/diffuse (0.1, 0.25, 0.8) -> ~RGB(26,64,204)
    // -> ~HSV(114, 223, 204) in OpenCV's 0-179/0-255/0-255 scale.
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
    // Yellow cube.
    // Gazebo material ambient/diffuse (0.85, 0.72, 0.18) -> ~RGB(217,184,46)
    // -> ~HSV(24, 201, 217).
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
    // Collection box - two-stage approach (black region to locate it,
    // white X marker inside that region to pinpoint its center).
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
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;

    // Camera intrinsics
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
