// src/image_blur_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>


/*

1. Noise Reduction
    it smooths out small variations and random noise in images, helping downstream algorithms work better.

2. Preprocessing for Edge Detection
    Real images often have noise that causes false edges.
    Applying Gaussian blur before edge detection (like Canny edge detector) reduces false positives and improves edge accuracy.
    Example: Detecting object boundaries in a photo for object recognition.

3. Image Downsampling (Pyramids)
    When building image pyramids (multi-scale representations), Gaussian blur is used before downsampling to prevent aliasing.
    Example: In face detection algorithms that scan images at multiple scales.

4. Background Blur / Bokeh Effect in Photography
    Simulating shallow depth-of-field by blurring the background while keeping the subject sharp.
    Example: Portrait photography apps or phone cameras with portrait mode.

5. Smoothing in Computer Graphics and Animation
    Used to soften textures or transitions for more natural visuals.
    Example: Blur shadows or lighting effects to make them look more realistic.

6. Reducing Detail for Feature Extraction
    Some algorithms extract global features rather than fine details. Blurring helps by removing irrelevant small details.
    Example: Preparing satellite images for land cover classification.

7. Anti-Aliasing
    Gaussian blur can be part of anti-aliasing techniques to smooth jagged edges in digital images or rendered graphics.

*/

class ImageBlurNode : public rclcpp::Node {
public:
    ImageBlurNode() : Node("image_blur_node") {
        using std::placeholders::_1;
        sub_ = image_transport::create_subscription(this, "/image_raw", std::bind(&ImageBlurNode::image_callback, this, _1), "raw");
        pub_ = image_transport::create_publisher(this, "/image_blurred");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;
        cv::Mat blurred;
        cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
        pub_.publish(cv_bridge::CvImage(msg->header, "mono8", blurred).toImageMsg());
    }

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageBlurNode>());
    rclcpp::shutdown();
    return 0;
}