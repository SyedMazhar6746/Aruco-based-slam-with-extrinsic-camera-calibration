#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

class CameraImage : public rclcpp::Node {
public:
    CameraImage() : Node("camera_image"), i(1) {
        image_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/aruco_position", 10);
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/kobuki/sensors/realsense/color/image_color", 10, std::bind(&CameraImage::imageCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraImage::timerCallback, this));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;

        // Set marker ID and length
        int marker_id = 1;
        double marker_length = 0.16;

        // Load the ArUco dictionary
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

        cv::Mat camera_matrix = (cv::Mat1d(3, 3) << 1396.8086675255468, 0.0, 960.0,
                                                    0.0, 1396.8086675255468, 540.0,
                                                    0.0, 0.0, 1.0);

        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        // Detect ArUco markers in the frame
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;
        cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids);

        if (!marker_ids.empty()) {
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                cv::Vec3d rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
                
                cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

                cv::Vec3d rotation_vector = rvecs;
                cv::Vec3d translation_vector = tvecs;

                cv::aruco::drawAxis(frame, camera_matrix, dist_coeffs, rotation_vector, translation_vector, marker_length * 0.5f);

                // Display the X, Y, and Z coordinates of the marker on the frame
                double x = tvecs[0];
                double y = tvecs[1];
                double z = tvecs[2];

                cv::putText(frame, cv::format("X: %.2f", x), cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                cv::putText(frame, cv::format("Y: %.2f", y), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                cv::putText(frame, cv::format("Z: %.2f", z), cv::Point(10, 85), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

                // Create and publish the Float64MultiArray message
                auto point_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
                point_msg->data = {x, y, z, static_cast<double>(marker_ids[i])};
                image_pub->publish(std::move(point_msg));
            }
        }

        // Display the frame
        cv::imshow("Camera", frame);
        cv::waitKey(3);
    }

    void timerCallback() {
        // Optional: Perform any periodic actions here
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr image_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    int i;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraImage>());
    rclcpp::shutdown();
    return 0;
}
