// src/image_edge_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <string>

enum class EdgeMethod {
    CANNY,
    SOBEL
};

class ImageEdgeNode : public rclcpp::Node {
public:
    ImageEdgeNode(const std::string &method_str)
    : Node("image_edge_detection_node") 
    {
        if (method_str == "sobel") {
            method_ = EdgeMethod::SOBEL;
            RCLCPP_INFO(this->get_logger(), "Using Sobel edge detection");
        } else {
            method_ = EdgeMethod::CANNY;
            RCLCPP_INFO(this->get_logger(), "Using Canny edge detection");
        }

        sub_ = image_transport::create_subscription(
            this, "/image_blurred",
            std::bind(&ImageEdgeNode::image_callback, this, std::placeholders::_1),
            "raw"
        );

        pub_ = image_transport::create_publisher(this, "/image_edges");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv::Mat gray = cv_bridge::toCvShare(msg, "mono8")->image;

        cv::Mat edges;

        if (method_ == EdgeMethod::CANNY) {
            const double low_threshold = 50;
            const double high_threshold = 150;
            cv::Canny(gray, edges, low_threshold, high_threshold);
        } else if (method_ == EdgeMethod::SOBEL) {
            cv::Mat grad_x, grad_y;
            cv::Mat abs_grad_x, abs_grad_y;

            cv::Sobel(gray, grad_x, CV_16S, 1, 0, 3);
            cv::Sobel(gray, grad_y, CV_16S, 0, 1, 3);

            cv::convertScaleAbs(grad_x, abs_grad_x);
            cv::convertScaleAbs(grad_y, abs_grad_y);

            cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);
        }

        auto edge_msg = cv_bridge::CvImage(msg->header, "mono8", edges).toImageMsg();
        pub_.publish(*edge_msg);
    }

    EdgeMethod method_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::string method = "canny";  // default
    if (argc > 1) {
        method = argv[1];
    }

    auto node = std::make_shared<ImageEdgeNode>(method);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
