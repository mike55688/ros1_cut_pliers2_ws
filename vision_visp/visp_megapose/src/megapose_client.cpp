#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <numeric>

#include <ros/ros.h>
#include <deque>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>

// ViSP includes
// #include <visp3/core/vpTime.h>
// #include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorDNNOpenCV.h>

#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>

// OpenCV/ViSP bridge includes
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <cv_bridge/cv_bridge.h>

// ROS 1 includes
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <forklift_server/Detection.h>

// ROS 1 message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS 1 custom messages and services
#include <visp_megapose/Confidence.h>
#include <visp_megapose/Init.h>
#include <visp_megapose/Track.h>
#include <visp_megapose/Render.h>

using namespace std::chrono_literals; // For using time literals like 1s

enum DetectionMethod
{
  UNKNOWN,
  CLICK,
  DNN
};

struct DetectionAllowed
{
  bool detection_allowed;
  float layer;
};

std::map<std::string, DetectionMethod> stringToDetectionMethod = {
  {"UNKNOWN", UNKNOWN},
  {"CLICK", CLICK},
  {"DNN", DNN}};

bool fileExists(const std::string &path)
{
  std::ifstream file(path);
  return file.good();
}

class MegaPoseClient
{
private:
  ros::NodeHandle* nh_;
  ros::NodeHandle* priv_nh_;
  ros::Subscriber detection_allowed_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber depth_info_sub_;

  DetectionAllowed detection_allowed_;

  std::string image_topic;
  std::string camera_info_topic;
  std::string depth_topic;
  std::string depth_info_topic;
  bool use_depth;
  std::string camera_tf;
  std::string detectorMethod;
  std::string detectorModelPath;
  std::string objectName;
  bool bounding_box;
  bool renderEnable;
  bool UIEnable;
  std::string detectionMode;
  // std::string detection_allowed_topic;
  int buffer_size;

  double reinitThreshold_,refilterThreshold_;
  double confidence_;
  sensor_msgs::CameraInfoConstPtr roscam_info_;
  vpImage<vpRGBa> vpI_;                          // Image used for debug display
  boost::shared_ptr<const sensor_msgs::Image> rosI_; // ROS Image
  boost::shared_ptr<const sensor_msgs::Image> rosD_; // ROS Depth Image
  std::deque<double> buffer_x, buffer_y, buffer_z,buffer_qw, buffer_qx, buffer_qy, buffer_qz;
  double filt_x = 0.0, filt_y = 0.0, filt_z = 0.0, filt_qw = 0.0, filt_qx = 0.0, filt_qy = 0.0, filt_qz = 0.0;

  geometry_msgs::Transform transform_,filter_transform_;
  unsigned width_ = 0, height_ = 0, widthD_ = 0, heightD_ = 0;

  void initial_pose_service_response_callback(const visp_megapose::Init::Response& future);
  bool initialized_;
  // bool init_request_done_;
  void track_pose_service_response_callback(const visp_megapose::Track::Response& future);
  // bool track_request_done_;
  void render_service_response_callback(const visp_megapose::Render::Response& future);
  // bool render_request_done_;
  float confidence_score;
  bool overlayModel_;
  
  void init_parameter();
  void waitForImage();
  void waitForDepth();
  void detectionAllowedCallback(const forklift_server::Detection &msg);
  void imageCallback(const sensor_msgs::ImageConstPtr &image);
  void infoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info);
  void depthCallback(const sensor_msgs::ImageConstPtr &depth);
  void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr &depth_info);
  void overlayRender(const vpImage<vpRGBa> &overlay);
  DetectionMethod getDetectionMethodFromString(const std::string &str);
  void broadcastTransformAndPose(const geometry_msgs::Transform &transform, const std::string &objectName, const std::string &camera_tf);
  void broadcastTransformAndPose_filter(const geometry_msgs::Transform &origpose, const std::string &objectName);
  double calculateMovingAverage(const std::deque<double>& buffer);
  void broadcastConfidenceScore(const std::string &child_frame_id, float confidence_score, bool initialized_);
  vpColor interpolate(const vpColor &low, const vpColor &high, const float f);
  void displayScore(float);
  void transformToVispHomogeneousMatrix(const geometry_msgs::Transform& transform, vpHomogeneousMatrix &M);
  std::optional<vpRect> detectObjectForInitMegaposeClick();
  std::optional<vpRect> detectObjectForInitMegaposeDnn(const std::string &detectionLabel, double confidenceThreshold);
  void displayBoundingBoxOnVispWindow(const std::string &detectionLabel, const std::optional<vpRect> &bboxOpt);
  vpImage<vpRGBa> overlay_img_;
  vpCameraParameters vpcam_info_;
  vpDetectorDNNOpenCV dnn_;
  int check_wait_time = 0;
public:
  explicit MegaPoseClient(ros::NodeHandle* nh, ros::NodeHandle* priv_nh);
  ~MegaPoseClient();
  void spin();
};

MegaPoseClient::MegaPoseClient(ros::NodeHandle* nh, ros::NodeHandle* priv_nh)
    : nh_(nh), priv_nh_(priv_nh)
{
  init_parameter();
  reinitThreshold_ = 0.2;
  refilterThreshold_ = 0.5;
  initialized_ = false;
  // init_request_done_ = true;
  // track_request_done_ = true;
  // render_request_done_ = true;
  overlayModel_ = true;
  detection_allowed_sub_ = nh_->subscribe(objectName + "_detection", 1, &MegaPoseClient::detectionAllowedCallback, this);

  image_sub_ = nh_->subscribe(image_topic, 1, &MegaPoseClient::imageCallback, this);
  camera_info_sub_ = nh_->subscribe(camera_info_topic, 1, &MegaPoseClient::infoCallback, this);
  if (use_depth)
  {
    depth_sub_ = nh_->subscribe(depth_topic, 1, &MegaPoseClient::depthCallback, this);
    depth_info_sub_ = nh_->subscribe(depth_info_topic, 1, &MegaPoseClient::depthInfoCallback, this);
  }

  ROS_INFO("MegaPoseClient initialized.");
}

MegaPoseClient::~MegaPoseClient()
{
  ROS_INFO("Shutting down MegaPoseClient");
  // data_file_.close();
  ros::shutdown();
}

DetectionMethod MegaPoseClient::getDetectionMethodFromString(const std::string &str)
{
  if (stringToDetectionMethod.find(str) != stringToDetectionMethod.end())
  {
    return stringToDetectionMethod[str];
  }
  return UNKNOWN; // Default case if string is not found
};

void MegaPoseClient::init_parameter()
{
  priv_nh_->param<std::string>("image_topic", image_topic, "/camera/image_raw");
  priv_nh_->param<std::string>("camera_info_topic", camera_info_topic, "/camera/camera_info");
  priv_nh_->param<bool>("use_depth", use_depth, true);
  priv_nh_->param<std::string>("depth_topic", depth_topic, "/camera/depth/image_rect_raw");
  priv_nh_->param<std::string>("depth_info_topic", depth_info_topic, "/camera/depth/camera_info");
  priv_nh_->param<std::string>("camera_tf", camera_tf, "camera_color_optical_frame");
  priv_nh_->param<std::string>("detector_method", detectorMethod, "DNN");
  priv_nh_->param<std::string>("detector_model_path", detectorModelPath, "none");
  priv_nh_->param<std::string>("object_name", objectName, "cube");
  priv_nh_->param<bool>("render_enable", renderEnable, true);
  priv_nh_->param<bool>("UI_enable", UIEnable, true);
  priv_nh_->param<std::string>("detection_mode", detectionMode, "Auto");
  // priv_nh_->param<std::string>("detection_allowed_topic", detection_allowed_topic, "/shelf_detection");
  priv_nh_->param<int>("buffer_size", buffer_size, 5);
  // 設定是否使用 bounding box
  priv_nh_->param<bool>("bounding_box", bounding_box, true);

  ROS_INFO("=== Parameters Loaded ===");
  ROS_INFO("Image topic: %s", image_topic.c_str());
  ROS_INFO("Camera info topic: %s", camera_info_topic.c_str());
  ROS_INFO("Depth topic: %s", depth_topic.c_str());
  ROS_INFO("Depth info topic: %s", depth_info_topic.c_str());
  ROS_INFO("Use Depth: %s", use_depth ? "True" : "False");
  ROS_INFO("Camera TF: %s", camera_tf.c_str());
  ROS_INFO("Detector method: %s", detectorMethod.c_str());
  ROS_INFO("Detector model path: %s", detectorModelPath.c_str());
  ROS_INFO("Object name: %s", objectName.c_str());
  ROS_INFO("Render enable: %s", renderEnable ? "True" : "False");
  ROS_INFO("UI enable: %s", UIEnable ? "True" : "False");
  ROS_INFO("Detection mode: %s", detectionMode.c_str());
  // ROS_INFO("Detection allowed topic: %s", detection_allowed_topic.c_str());
  ROS_INFO("Buffer size: %d", buffer_size);
  // 設定是否使用 bounding box
  ROS_INFO("Bounding box: %s", bounding_box ? "True" : "False");
}

void MegaPoseClient::waitForImage()
{
  ros::Rate loop_rate(10);
  ROS_INFO("Waiting for a rectified image...");
  while (ros::ok())
  {
    if (width_ > 0 && height_ > 0)
    {
      ROS_INFO("Got image!");
      return;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void MegaPoseClient::waitForDepth()
{
  ros::Rate loop_rate(10);
  ROS_INFO("Waiting for a rectified depth...");
  while (ros::ok())
  {
    if (widthD_ > 0 && heightD_ > 0)
    {
      ROS_INFO("Got image!");
      return;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void MegaPoseClient::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
  rosI_ = image;
  width_ = image->width;
  height_ = image->height;
  if(UIEnable)
  {
    vpI_ = visp_bridge::toVispImageRGBa(*image);
  }
}

void MegaPoseClient::infoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info)
{
  roscam_info_ = cam_info;
  if(UIEnable)
  {
    vpcam_info_ = visp_bridge::toVispCameraParameters(*cam_info);
  }
}

void MegaPoseClient::depthCallback(const sensor_msgs::ImageConstPtr &depth)
{
  rosD_ = depth;
  widthD_ = depth->width;
  heightD_ = depth->height;
  if (width_ == 0 || height_ == 0)
  {
    // ROS_ERROR("Image size is 0. Waiting for image.");
    return;
  }
  else if (width_ != widthD_ || height_ != heightD_)
  {
    ROS_ERROR("Image and depth image sizes do not match.");
    return;
  }
  
  // ROS_INFO ("Image width: %d, height: %d", width_, height_);
  // ROS_INFO ("Depth width: %d, height: %d", widthD_, heightD_);
}

void MegaPoseClient::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr &depth_info)
{
  if (width_ == 0 || height_ == 0)
  {
    // ROS_ERROR("Image size is 0. Waiting for image.");
    return;
  }
  else if (width_ != widthD_ || height_ != heightD_)
  {
    ROS_ERROR("Image and depth image sizes do not match.");
    return;
  }
}

void MegaPoseClient::detectionAllowedCallback(const forklift_server::Detection &msg)
{
  detection_allowed_.detection_allowed = msg.detection_allowed;
  detection_allowed_.layer = msg.layer;
}

void MegaPoseClient::broadcastTransformAndPose(const geometry_msgs::Transform &transform, const std::string &objectName, const std::string &camera_tf)
{
  // broadcast transform

  static geometry_msgs::TransformStamped transformStamped;
  static tf::TransformBroadcaster tf_broadcaster_;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = camera_tf;
  transformStamped.child_frame_id = objectName;
  transformStamped.transform = transform;
  tf_broadcaster_.sendTransform(transformStamped);
  // publish target pose
  static ros::Publisher pub_pose_ = nh_->advertise<geometry_msgs::Pose>(objectName, 1, true);
  // static auto pub_pose_ = this->create_publisher<geometry_msgs::Pose>(objectName, 1);
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation.x = transform.rotation.x;
  pose.orientation.y = transform.rotation.y;
  pose.orientation.z = transform.rotation.z;
  pose.orientation.w = transform.rotation.w;
  pub_pose_.publish(pose);
}

void MegaPoseClient::broadcastTransformAndPose_filter(const geometry_msgs::Transform &origpose, const std::string &objectName)
{
  if(confidence_ > refilterThreshold_)
  {
    if (boost::numeric_cast<int>(buffer_x.size()) >= buffer_size)
    {
      buffer_x.pop_front();
      buffer_y.pop_front();
      buffer_z.pop_front();
      buffer_qw.pop_front();
      buffer_qx.pop_front();
      buffer_qy.pop_front();
      buffer_qz.pop_front();
    }
    buffer_x.push_back(origpose.translation.x);
    buffer_y.push_back(origpose.translation.y);
    buffer_z.push_back(origpose.translation.z);
    buffer_qw.push_back(origpose.rotation.w);
    buffer_qx.push_back(origpose.rotation.x);
    buffer_qy.push_back(origpose.rotation.y);
    buffer_qz.push_back(origpose.rotation.z);

    filter_transform_.translation.x = calculateMovingAverage(buffer_x);
    filter_transform_.translation.y = calculateMovingAverage(buffer_y);
    filter_transform_.translation.z = calculateMovingAverage(buffer_z);
    filter_transform_.rotation.w = calculateMovingAverage(buffer_qw);
    filter_transform_.rotation.x = calculateMovingAverage(buffer_qx);
    filter_transform_.rotation.y = calculateMovingAverage(buffer_qy);
    filter_transform_.rotation.z = calculateMovingAverage(buffer_qz);

    static ros::Publisher pub_filter_ = nh_->advertise<geometry_msgs::Pose>(objectName + "_filter", 1, true);
    // static auto pub_filter_ = this->create_publisher<geometry_msgs::Pose>(objectName + "_filter", 1);
    geometry_msgs::Pose pose;
    pose.position.x = filter_transform_.translation.x;
    pose.position.y = filter_transform_.translation.y;
    pose.position.z = filter_transform_.translation.z;
    pose.orientation.x = filter_transform_.rotation.x;
    pose.orientation.y = filter_transform_.rotation.y;
    pose.orientation.z = filter_transform_.rotation.z;
    pose.orientation.w = filter_transform_.rotation.w;
    pub_filter_.publish(pose);
  }
  // data_file_ << std::fixed << std::setprecision(6)
  //   << origpose.position.x << ", " << origpose.position.y << ", " << origpose.position.z << ", "
  //   << origpose.orientation.w << ", " << origpose.orientation.x << ", " << origpose.orientation.y << ", " << origpose.orientation.z << ", "
  //   << filt_x << ", " << filt_y << ", " << filt_z << ", "
  //   << filt_qw << ", " << filt_qx << ", " << filt_qy << ", " << filt_qz << "\n";
}

double MegaPoseClient::calculateMovingAverage(const std::deque<double>& buffer)
{
  if (buffer.size() < 1) return 0.0;  // Avoid division by zero
  return std::accumulate(buffer.begin(), buffer.end(), 0.0) / buffer.size();
}

void MegaPoseClient::broadcastConfidenceScore(const std::string &objectName, float confidence_score, bool detection)
{
  // publish confidence score
  static ros::Publisher pub_confidence_ = nh_->advertise<visp_megapose::Confidence>(objectName + "_confidence", 1, true);
  // static auto pub_confidence_ = this->create_publisher<visp_megapose::msg::Confidence>(objectName + "_confidence", 1);
  visp_megapose::Confidence confidence_msg;
  confidence_msg.object_confidence = confidence_score;
  confidence_msg.model_detection = detection;
  pub_confidence_.publish(confidence_msg);
}

void MegaPoseClient::spin()
{
  if (getDetectionMethodFromString(detectorMethod) == UNKNOWN)
  {
    ROS_ERROR("Unknown detection method. Exiting.");
    ros::shutdown();
  }

  if (!fileExists(detectorModelPath))
  {
    ROS_ERROR("Detector model path does not exist. Exiting.");
    ros::shutdown();
  }

  std::string detectorConfig = "none";
  std::string detectorFramework = "onnx", detectorTypeString = "yolov7";
  std::vector<std::string> labels = {objectName};

  waitForImage();
  if (use_depth)
  {
    waitForDepth();
  }
  // Initialize DNN detector if detectorMethod is DNN
  if (getDetectionMethodFromString(detectorMethod) == DNN)
  {
    float detectorMeanR = 0.f, detectorMeanG = 0.f, detectorMeanB = 0.f;
    float detectorConfidenceThreshold = 0.65f, detectorNmsThreshold = 0.5f, detectorFilterThreshold = -0.25f;
    float detectorScaleFactor = 0.0039f;
    bool detectorSwapRB = false;

    vpDetectorDNNOpenCV::DNNResultsParsingType detectorType = vpDetectorDNNOpenCV::dnnResultsParsingTypeFromString(detectorTypeString);
    vpDetectorDNNOpenCV::NetConfig netConfig(detectorConfidenceThreshold, detectorNmsThreshold, labels, cv::Size(640, 640), detectorFilterThreshold);
    // vpDetectorDNNOpenCV dnn(netConfig, detectorType);  // I don't know why this doesn't work. If I use this
    // it will cause the error "Cuda and/or GPU driver might not be correctly installed. Setting preferable backend to CPU and trying again.
    // terminate called after throwing an instance of 'cv::Exception"
    dnn_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    dnn_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    dnn_.setNetConfig(netConfig);
    dnn_.setParsingMethod(detectorType);
    dnn_.readNet(detectorModelPath, detectorConfig, detectorFramework);
    dnn_.setMean(detectorMeanR, detectorMeanG, detectorMeanB);
    dnn_.setScaleFactor(detectorScaleFactor);
    dnn_.setSwapRB(detectorSwapRB);
  }

  vpDisplayX *d = NULL;
  d = new vpDisplayX();
  // ros::spinOnce();
  if(UIEnable)
  {
    d->init(vpI_); // also init display   //顯示MegaPose可視化界面可以關閉減低效能
    vpDisplay::setTitle(vpI_, "MegaPose debug " + objectName);    //顯示MegaPose可視化界面可以關閉減低效能
  }

  ros::ServiceClient initial_pose_client = nh_->serviceClient<visp_megapose::Init>("initial_pose");
  ros::ServiceClient track_pose_client = nh_->serviceClient<visp_megapose::Track>("track_pose");
  ros::ServiceClient render_client = nh_->serviceClient<visp_megapose::Render>("render_object");
  while (!ros::service::waitForService("initial_pose", ros::Duration(1.0)) &&
         !ros::service::waitForService("track_pose", ros::Duration(1.0)) &&
         !ros::service::waitForService("render_object", ros::Duration(1.0)))
  {
    if (!ros::ok())
    {
      ROS_ERROR("Interrupted while waiting for the service. Exiting.");
      return;
    }
    ROS_INFO("initial_pose service not available, waiting again...");
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    vpDisplay::display(vpI_);
    ros::spinOnce();
    if(detection_allowed_.detection_allowed)
      vpDisplay::displayText(vpI_, 40, 20, "Detection allowed state: True", vpColor::red);
    else
      vpDisplay::displayText(vpI_, 40, 20, "Detection allowed state: False", vpColor::red);

    // ROS_INFO("initialized_ %d", initialized_ );
    // ROS_INFO("init_request_done_ %d", init_request_done_ );
    // ROS_INFO("detection_allowed_.detection_allowed %d", detection_allowed_.detection_allowed );
    if (!initialized_)
    {
      std::optional<vpRect> detection = std::nullopt;

      if (getDetectionMethodFromString(detectorMethod) == CLICK)
      {
        detection = detectObjectForInitMegaposeClick();
      }
      else if (getDetectionMethodFromString(detectorMethod) == DNN && detectionMode == "Auto")
      {
        detection_allowed_.detection_allowed = true;
        detection = detectObjectForInitMegaposeDnn(objectName, 0.5);
      }
      else if (getDetectionMethodFromString(detectorMethod) == DNN && detectionMode == "Manual" && detection_allowed_.detection_allowed == true)
      {
        detection = detectObjectForInitMegaposeDnn(objectName, 0.5);
      }

      if (detection)
      {
        visp_megapose::Init initial_pose_request;
        visp_megapose::Init::Response initial_pose_response;
        initial_pose_request.request.object_name = objectName;
        initial_pose_request.request.topleft_i = detection->getTopLeft().get_i();
        initial_pose_request.request.topleft_j = detection->getTopLeft().get_j();
        initial_pose_request.request.bottomright_i = detection->getBottomRight().get_i();
        initial_pose_request.request.bottomright_j = detection->getBottomRight().get_j();
        initial_pose_request.request.image = *rosI_;
        if (use_depth)
        {
          initial_pose_request.request.depth = *rosD_;
        }
        else
        {
          initial_pose_request.request.depth = sensor_msgs::Image();
        }
        initial_pose_request.request.camera_info = *roscam_info_;

        if (initial_pose_client.call(initial_pose_request))
        {
          // ROS_INFO("Initial pose service called successfully.");
          initial_pose_service_response_callback(initial_pose_request.response);
          // init_request_done_ = false;
        } 
        else 
        {
          ROS_ERROR("Failed to call initial pose service.");
        }
      }
    }
    else if (initialized_)
    {
      visp_megapose::Track track_pose_request;
      // if (track_request_done_)
      // {
        track_pose_request.request.object_name = objectName;
        track_pose_request.request.init_pose = transform_;
        track_pose_request.request.refiner_iterations = 1;
        track_pose_request.request.image = *rosI_;
        if (use_depth)
        {
          track_pose_request.request.depth = *rosD_;
        }
        else
        {
          track_pose_request.request.depth = sensor_msgs::Image();
        }
        track_pose_request.request.camera_info = *roscam_info_;

        if (track_pose_client.call(track_pose_request)) 
        {
          // ROS_INFO("Track pose service called successfully.");
          track_pose_service_response_callback(track_pose_request.response);
          // track_request_done_ = false;
        } 
        else 
        {
          ROS_ERROR("Failed to call track pose service.");
        }
      // }
      visp_megapose::Render render_request;
      if (overlayModel_ && renderEnable)
      {
        render_request.request.object_name = objectName;
        render_request.request.pose = transform_;
        render_request.request.camera_info = *roscam_info_;
        if (render_client.call(render_request)) 
        {
          // ROS_INFO("Render service called successfully.");
          render_service_response_callback(render_request.response);
          // render_request_done_ = false;
        } 
        else 
        {
          ROS_ERROR("Failed to call render service.");
        }
      }

      std::string keyboardEvent;
      const bool keyPressed = vpDisplay::getKeyboardEvent(vpI_, keyboardEvent, false);
      if (keyPressed)
      {
        if (keyboardEvent == "t")
          overlayModel_ = !overlayModel_;
      }
      if (overlay_img_.getSize() > 0 && overlayModel_)
        overlayRender(overlay_img_);
      vpDisplay::displayText(vpI_, 20, 20, "Right click to quit", vpColor::red);
      vpDisplay::displayText(vpI_, 30, 20, "Press t: Toggle overlay", vpColor::red);
      static vpHomogeneousMatrix M_original, M_filter;
      
      // ROS_INFO("confidence_: %f", confidence_);
      // ROS_INFO("transform_ translation: x=%f, y=%f, z=%f", transform_.translation.x, transform_.translation.y, transform_.translation.z);
      // ROS_INFO("transform_ rotation: x=%f, y=%f, z=%f, w=%f", transform_.rotation.x, transform_.rotation.y, transform_.rotation.z, transform_.rotation.w);
      
      M_original = visp_bridge::toVispHomogeneousMatrix(transform_);
      // transformToVispHomogeneousMatrix(transform_, M_original);
      vpDisplay::displayFrame(vpI_, M_original, vpcam_info_, 0.05, vpColor::none, 3);
      displayScore(confidence_);
      broadcastTransformAndPose(transform_, objectName, camera_tf);
      broadcastTransformAndPose_filter(transform_, objectName);
      M_filter = visp_bridge::toVispHomogeneousMatrix(filter_transform_);
      // vpDisplay::displayFrame(vpI_, M_filter, vpcam_info_, 0.05, vpColor::none, 3);
      // init_request_done_ = true;
      // track_request_done_ = true;
      // render_request_done_ = true;
    }
    broadcastConfidenceScore(objectName,confidence_,initialized_);

    vpDisplay::flush(vpI_);
    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(vpI_, button, false))
    {
      if (button == vpMouseButton::button3)
      {
        break; // Right click to stop
      }
    }
    // loop_rate.sleep();
  }
  delete d;
}

void MegaPoseClient::transformToVispHomogeneousMatrix(const geometry_msgs::Transform& transform, vpHomogeneousMatrix &M) 
{
  ROS_ERROR("1");
  M.eye();  // 設為單位矩陣
  ROS_ERROR("M matrix");
  ROS_ERROR("M matrix after eye() initialization: \n[%f, %f, %f, %f; %f, %f, %f, %f; %f, %f, %f, %f]", 
    M[0][0], M[0][1], M[0][2], M[0][3], 
    M[1][0], M[1][1], M[1][2], M[1][3], 
    M[2][0], M[2][1], M[2][2], M[2][3]);

  // 檢查 translation 是否包含無效值
  if (std::isnan(transform.translation.x) || std::isnan(transform.translation.y) || std::isnan(transform.translation.z)) {
    ROS_ERROR("Invalid transform: translation contains NaN.");
    return;
  }
  ROS_ERROR("2");

  // 驗證 M 的大小和結構
  ROS_ERROR("M matrix dimensions: 3x4");

  // 提取 translation 和 rotation
  ROS_ERROR("Transform translation: x=%f, y=%f, z=%f", transform.translation.x, transform.translation.y, transform.translation.z);
  M[0][3] = transform.translation.x;
  M[1][3] = transform.translation.y;
  M[2][3] = transform.translation.z;
  ROS_ERROR("3");

  // 用 quaternion 計算旋轉矩陣
  tf::Quaternion quat(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  tf::Matrix3x3 R(quat);
  ROS_ERROR("4");

  // 設定旋轉矩陣
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      M[i][j] = R[i][j];
    }
  }

  // 打印矩陣內容
  ROS_ERROR("M matrix after initialization: \n[%f, %f, %f, %f; %f, %f, %f, %f; %f, %f, %f, %f]", 
    M[0][0], M[0][1], M[0][2], M[0][3], 
    M[1][0], M[1][1], M[1][2], M[1][3], 
    M[2][0], M[2][1], M[2][2], M[2][3]);

  ROS_ERROR("5");
}

void MegaPoseClient::initial_pose_service_response_callback(const visp_megapose::Init::Response& future)
{
  // init_request_done_ = true;
  transform_ = future.pose;
  confidence_ = future.confidence;
  if (confidence_ <= refilterThreshold_)
  {
    buffer_x.clear();
    buffer_y.clear();
    buffer_z.clear();
    buffer_qw.clear();
    buffer_qx.clear();
    buffer_qy.clear();
    buffer_qz.clear();
  }
  
  if (confidence_ < reinitThreshold_)
  {
    ROS_INFO("Initial pose not reliable, reinitializing...");
  }
  else
  {
    initialized_ = true;
    ROS_INFO("Initialized successfully!");
  }
}

void MegaPoseClient::track_pose_service_response_callback(const visp_megapose::Track::Response& future)
{
  // track_request_done_ = true;
  transform_ = future.pose;
  confidence_ = future.confidence;
  if (detectionMode == "Manual" && detection_allowed_.detection_allowed == false)
  {
    initialized_ = false;
    ROS_INFO("No tracking allowed, waiting for tracking permission...");
  }
  else if (confidence_ < reinitThreshold_)
  {
    initialized_ = false;
    ROS_INFO("Tracking lost, reinitializing...");
  }
}

void MegaPoseClient::render_service_response_callback(const visp_megapose::Render::Response& future)
{
  // render_request_done_ = true;
  overlay_img_ = visp_bridge::toVispImageRGBa(future.image);
}

void MegaPoseClient::displayScore(float confidence)
{
  const unsigned top = static_cast<unsigned>(vpI_.getHeight() * 0.85f);
  const unsigned height = static_cast<unsigned>(vpI_.getHeight() * 0.1f);
  const unsigned left = static_cast<unsigned>(vpI_.getWidth() * 0.05f);
  const unsigned width = static_cast<unsigned>(vpI_.getWidth() * 0.5f);
  vpRect full(left, top, width, height);
  vpRect scoreRect(left, top, width * confidence, height);
  const vpColor low = vpColor::red;
  const vpColor high = vpColor::green;
  const vpColor c = interpolate(low, high, confidence);

  vpDisplay::displayRectangle(vpI_, full, c, false, 5);
  vpDisplay::displayRectangle(vpI_, scoreRect, c, true, 1);
}

vpColor MegaPoseClient::interpolate(const vpColor &low, const vpColor &high, const float f)
{
  const float r = ((float)high.R - (float)low.R) * f;
  const float g = ((float)high.G - (float)low.G) * f;
  const float b = ((float)high.B - (float)low.B) * f;
  return vpColor((unsigned char)r, (unsigned char)g, (unsigned char)b);
}

std::optional<vpRect> MegaPoseClient::detectObjectForInitMegaposeDnn(const std::string &detectionLabel, double confidenceThreshold)
{
  cv::Mat I = cv_bridge::toCvCopy(rosI_, rosI_->encoding)->image;
  std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> detections_vec;
  dnn_.detect(I, detections_vec);

  std::vector<vpDetectorDNNOpenCV::DetectedFeatures2D> matchingDetections;
  for (const auto &detection : detections_vec)
  {
    std::optional<std::string> classnameOpt = detection.getClassName();
    if (classnameOpt && *classnameOpt == detectionLabel)
    {
      if (detection.getConfidenceScore() > confidenceThreshold)
      {
        matchingDetections.push_back(detection);
      }
    }
  }

  if (matchingDetections.empty()) 
  {
    check_wait_time = 0;
    vpDisplay::displayText(vpI_, 20, 20, "No object detected", vpColor::red);
    return std::nullopt;
  }
  
  check_wait_time ++;
  if (check_wait_time <= 20)
  {
    vpDisplay::displayText(vpI_, 20, 20, "No object detected", vpColor::red);
    return std::nullopt;
  }
  
  if(matchingDetections.size() == 1)
  {
    check_wait_time = 0;
    // 設定 bounding box
    if (bounding_box)
    {
      displayBoundingBoxOnVispWindow(detectionLabel, matchingDetections[0].getBoundingBox());
    }
    return matchingDetections[0].getBoundingBox();
  }

  // 有多個目標
  for (const auto &detection : matchingDetections)
  {
    // 設定 bounding box
    if (bounding_box)
    {
      displayBoundingBoxOnVispWindow(detectionLabel, detection.getBoundingBox());
    }
  }
  auto bestDetection = std::max_element(
    matchingDetections.begin(),
    matchingDetections.end(),
    [this](const vpDetectorDNNOpenCV::DetectedFeatures2D &a, const vpDetectorDNNOpenCV::DetectedFeatures2D &b) {
      const vpRect bboxA = a.getBoundingBox();
      const vpRect bboxB = b.getBoundingBox();
      double bottomA = bboxA.getTop() + bboxA.getHeight();
      double bottomB = bboxB.getTop() + bboxB.getHeight();
      if (detection_allowed_.layer == 2.0) {
        return bottomA > bottomB;
      }
      return bottomA < bottomB;
    });
  check_wait_time = 0;
  return bestDetection->getBoundingBox();
}

void MegaPoseClient::displayBoundingBoxOnVispWindow(const std::string &detectionLabel, const std::optional<vpRect> &bboxOpt)
{
  vpRect bbox = bboxOpt.value();
  vpImagePoint ip1(static_cast<int>(bbox.getTop()), static_cast<int>(bbox.getLeft()));    // 左上角
  vpImagePoint ip2(static_cast<int>(bbox.getTop()), static_cast<int>(bbox.getRight()));   // 右上角
  vpImagePoint ip3(static_cast<int>(bbox.getBottom()), static_cast<int>(bbox.getRight()));  // 右下角
  vpImagePoint ip4(static_cast<int>(bbox.getBottom()), static_cast<int>(bbox.getLeft()));   // 左下角
  
  vpDisplay::displayLine(vpI_, ip1, ip2, vpColor::red, 2, true);  // 線寬為 2，紅色
  vpDisplay::displayLine(vpI_, ip2, ip3, vpColor::red, 2, true);
  vpDisplay::displayLine(vpI_, ip3, ip4, vpColor::red, 2, true);
  vpDisplay::displayLine(vpI_, ip4, ip1, vpColor::red, 2, true);
  
  // 在矩形左上方顯示 detectionLabel
  vpDisplay::displayText(vpI_, static_cast<int>(bbox.getTop()) - 10, static_cast<int>(bbox.getLeft()), detectionLabel, vpColor::red);
  vpDisplay::flush(vpI_); // 刷新顯示結果
}

std::optional<vpRect> MegaPoseClient::detectObjectForInitMegaposeClick()
{
  const bool startLabelling = vpDisplay::getClick(vpI_, false);

  const vpImagePoint textPosition(10.0, 20.0);

  if (startLabelling)
  {
    vpImagePoint topLeft, bottomRight;
    vpDisplay::displayText(vpI_, textPosition, "Click the upper left corner of the bounding box", vpColor::red);
    vpDisplay::flush(vpI_);
    vpDisplay::getClick(vpI_, topLeft, true);
    vpDisplay::display(vpI_);
    vpDisplay::displayCross(vpI_, topLeft, 5, vpColor::red, 2);
    vpDisplay::displayText(vpI_, textPosition, "Click the bottom right corner of the bounding box", vpColor::red);
    vpDisplay::flush(vpI_);
    vpDisplay::getClick(vpI_, bottomRight, true);
    vpRect bb(topLeft, bottomRight);
    return bb;
  }
  else
  {
    vpDisplay::display(vpI_);
    vpDisplay::displayText(vpI_, textPosition,
                           "Click when the object is visible and static to start reinitializing megapose.",
                           vpColor::red);
    vpDisplay::flush(vpI_);
    return std::nullopt;
  }
}

void MegaPoseClient::overlayRender(const vpImage<vpRGBa> &overlay)
{
  vpRGBa black(0, 0, 0);  // 初始化黑色
  for (unsigned int i = 0; i < height_; ++i)
  {
    for (unsigned int j = 0; j < width_; ++j)
    {
      if (const_cast<vpRGBa&>(overlay[i][j]) != black)  // 使用 const_cast 去掉 const
      {
        vpI_[i][j] = overlay[i][j];
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "megapose_client");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  MegaPoseClient node(&nh, &priv_nh);
  node.spin();

  return 0;
}
