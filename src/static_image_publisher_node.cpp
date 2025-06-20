#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class StaticImagePublisher : public rclcpp::Node {
public:
    StaticImagePublisher()
    : Node("static_image_publisher")
    {
        pub_ = image_transport::create_publisher(this, "image_raw");

        std::string image_path = "/home/yalin/Downloads/test_image.png";
        cv::Mat color_image = cv::imread(image_path, cv::IMREAD_COLOR);

        if (color_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Image cannot be read: %s", image_path.c_str());
            //rclcpp::shutdown();
            return;
        }

        // Görüntüyü griye çevir
        cv::cvtColor(color_image, image_, cv::COLOR_BGR2GRAY);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&StaticImagePublisher::publish_image, this)
        );
    }

private:
    void publish_image() {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_).toImageMsg();
        pub_.publish(msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "The grayscale image is being published: /image_raw");
    }

    cv::Mat image_;
    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
