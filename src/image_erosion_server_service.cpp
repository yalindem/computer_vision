// erosion_service.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "computer_vision/srv/erosion.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class ImageErosionServerService : public rclcpp::Node{
    public: 
        ImageErosionServerService() : Node("image_erosion_server_service")
        {
            service_ = this->create_service<computer_vision::srv::Erosion>(
                "erosion", 
                std::bind(&ImageErosionServerService::handle_service, this, _1, _2));
        }

    private: 


        void handle_service(const std::shared_ptr<computer_vision::srv::Erosion::Request> request,
                            std::shared_ptr<computer_vision::srv::Erosion::Response> response)
        {
            cv_bridge::CvImagePtr cv_ptr;
            std::string encoding;
            try
            {
                encoding = request->image.encoding;
                cv_ptr = cv_bridge::toCvCopy(request->image, encoding);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat input_image = cv_ptr->image;
            cv::Mat gray;

            // If the image is colored, convert it to grayscale
            if (input_image.channels() == 3)
            {
                RCLCPP_INFO(this->get_logger(), "Color image received. Converting to grayscale...");
                cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
            }
            else if (input_image.channels() == 1)
            {
                RCLCPP_INFO(this->get_logger(), "Grayscale image received.");
                gray = input_image;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Unexpected number of channels: %d", input_image.channels());
                return;
            }

            // Perform erosion
            int erosion_size = request->erosion_size;
            cv::Mat eroded;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                    cv::Point(erosion_size, erosion_size));
            cv::erode(gray, eroded, kernel);

            // Convert back to ROS Image message with mono8 encoding
            cv_bridge::CvImage out_msg;
            out_msg.header = cv_ptr->header;
            out_msg.encoding = "mono8";  // Erosion result is a grayscale image
            out_msg.image = eroded;

            response->eroded_image = *out_msg.toImageMsg();
        }

        rclcpp::Service<computer_vision::srv::Erosion>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageErosionServerService>());
    rclcpp::shutdown();
    return 0;
}
