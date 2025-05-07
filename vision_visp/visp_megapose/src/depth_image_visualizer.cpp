#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <opencv2/imgproc/imgproc.hpp>

class DepthImageVisualizer
{
public:
    explicit DepthImageVisualizer(ros::NodeHandle* nh, ros::NodeHandle* priv_nh);
    ~DepthImageVisualizer();
    void spin();
private:
    ros::NodeHandle* nh_;
    ros::NodeHandle* priv_nh_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber rgb_image_sub_;
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool got_depth_image_ = false;
    bool got_rgb_image_ = false;
    vpImage<vpRGBa> depth_image_;
    vpImage<vpRGBa> rgb_image_;
    vpDisplayX* depth_display_ = nullptr;
    vpDisplayX* rgb_display_ = nullptr;
    std::string depth_topic_;
    std::string rgb_topic_;
};

DepthImageVisualizer::~DepthImageVisualizer()
{
    ROS_INFO("Shutting down DepthImageVisualizer");
    ros::shutdown();
}

DepthImageVisualizer::DepthImageVisualizer(ros::NodeHandle* nh, ros::NodeHandle* priv_nh)
    : nh_(nh), priv_nh_(priv_nh)
{
    // 获取参数
    priv_nh_->param<std::string>("depth_topic", depth_topic_, "/camera/aligned_depth_to_color/image_raw");
    priv_nh_->param<std::string>("rgb_topic", rgb_topic_, "/camera/color/image_raw");

    // 订阅深度图像主题
    depth_image_sub_ = nh_->subscribe(depth_topic_, 1, &DepthImageVisualizer::depthImageCallback, this);
    // 订阅RGB图像主题
    rgb_image_sub_ = nh_->subscribe(rgb_topic_, 1, &DepthImageVisualizer::rgbImageCallback, this);

    ROS_INFO("DepthImageVisualizer initialized.");
}

void DepthImageVisualizer::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_image = cv_ptr->image;

    // 将深度图像缩放到 0~255（假设最大深度值为 10000 毫米）
    cv::Mat depth_image_8u;
    depth_image.convertTo(depth_image_8u, CV_8U, 255.0 / 10000.0);

    // 应用色彩映射
    cv::Mat depth_image_colormap;
    cv::applyColorMap(depth_image_8u, depth_image_colormap, cv::COLORMAP_JET);

    // 将 OpenCV 的 cv::Mat 转换为 ViSP 的 vpImage<vpRGBa>
    depth_image_.resize(depth_image_colormap.rows, depth_image_colormap.cols);
    for (int i = 0; i < depth_image_colormap.rows; i++)
    {
        for (int j = 0; j < depth_image_colormap.cols; j++)
        {
            cv::Vec3b color = depth_image_colormap.at<cv::Vec3b>(i, j);
            depth_image_[i][j] = vpRGBa(color[2], color[1], color[0]);
        }
    }
    got_depth_image_ = true;
}

void DepthImageVisualizer::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat rgb_image = cv_ptr->image;

    // 将 OpenCV 的 cv::Mat 转换为 ViSP 的 vpImage<vpRGBa>
    rgb_image_.resize(rgb_image.rows, rgb_image.cols);
    for (int i = 0; i < rgb_image.rows; i++)
    {
        for (int j = 0; j < rgb_image.cols; j++)
        {
            cv::Vec3b color = rgb_image.at<cv::Vec3b>(i, j);
            rgb_image_[i][j] = vpRGBa(color[2], color[1], color[0]);
        }
    }
    got_rgb_image_ = true;
}

void DepthImageVisualizer::spin()
{
    while (ros::ok())
    {
        if (got_depth_image_)
        {
            // 初始化深度图像显示器（仅在第一次调用时）
            if (!depth_display_)
            {
                depth_display_ = new vpDisplayX(depth_image_, 100, 100, "Depth Image");
            }

            // 显示深度图像
            vpDisplay::display(depth_image_);
            vpDisplay::flush(depth_image_);
        }

        if (got_rgb_image_)
        {
            // 初始化RGB图像显示器（仅在第一次调用时）
            if (!rgb_display_)
            {
                rgb_display_ = new vpDisplayX(rgb_image_, 100, 500, "RGB Image");
            }

            // 显示RGB图像
            vpDisplay::display(rgb_image_);
            vpDisplay::flush(rgb_image_);
        }

        // 检查深度图像和RGB图像的尺寸是否相同
        if (got_depth_image_ && got_rgb_image_)
        {
            if (depth_image_.getWidth() == rgb_image_.getWidth() &&
                depth_image_.getHeight() == rgb_image_.getHeight())
            {
                // ROS_INFO("Depth image and RGB image have the same dimensions.");
            }
            else
            {
                ROS_WARN("Depth image and RGB image have different dimensions.");
            }
        }

        ros::spinOnce();
        ros::Rate(30).sleep(); // 控制循环频率
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_image_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    DepthImageVisualizer node(&nh, &priv_nh);
    node.spin();
    return 0;
}
