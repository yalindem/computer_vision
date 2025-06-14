// src/image_edge_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class ImageEdgeNode : public rclcpp::Node {
public:
    ImageEdgeNode() : Node("image_edge_detection_node") {
        sub_ = image_transport::create_subscription(
            this, "/image_blurred",
            std::bind(&ImageEdgeNode::image_callback, this, std::placeholders::_1),
            "raw"
        );

        pub_ = image_transport::create_publisher(this, "/image_edges");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat edges;
        const double low_threshold = 50;
        const double high_threshold = 150;
        cv::Canny(gray, edges, low_threshold, high_threshold);

        auto edge_msg = cv_bridge::CvImage(msg->header, "mono8", edges).toImageMsg();
        pub_.publish(*edge_msg);
    }

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageEdgeNode>());
    rclcpp::shutdown();
    return 0;
}
