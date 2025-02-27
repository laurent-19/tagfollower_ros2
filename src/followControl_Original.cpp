#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include "tagfollower_ros2/msg/navdata.hpp"

// for img processing 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

// for Kalman filter
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class TagFollowerNode : public rclcpp::Node
{
public:
  TagFollowerNode() : Node("tag_follower")
  {
    // Initialize the Kalman filter
    initKF();
    
    // Create publishers
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Create subscribers
    navdata_subscription_ = this->create_subscription<tagfollower_ros2::msg::Navdata>(
      "/ardrone/navdata", 10,
      std::bind(&TagFollowerNode::navCallback, this, std::placeholders::_1));
    
    // Image transport in ROS2
    image_subscription_ = image_transport::create_subscription(
      this, "/ardrone/image_raw",
      std::bind(&TagFollowerNode::imageCallback, this, std::placeholders::_1),
      "raw");
      
    // Timer for the main loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz
      std::bind(&TagFollowerNode::timerCallback, this));
  }

private:
  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<tagfollower_ros2::msg::Navdata>::SharedPtr navdata_subscription_;
  image_transport::Subscriber image_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  geometry_msgs::msg::Twist command;
  cv::Point position1, position2;

  // Kalman filter variables
  KalmanFilter KF{4, 2, 0, CV_32F}; //=CreateKalman(4, 2, 0);
  Mat_<float> state{4, 1}; /* (x, y, Vx, Vy) */
  Mat processNoise{4, 1, CV_32F};
  Mat_<float> measurmentVP{2, 1};
  vector<Point> kalmanVec;
  int nrNotDetect = 0;
  Point observation, estimation;
  double lastTimeStamp = 0.0;

  void initKF()
  {
    measurmentVP.setTo(Scalar(0.0));
    KF.statePre.at<float>(0) = 16;
    KF.statePre.at<float>(1) = 11;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    // 4x4 for the constant velocity model
    KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
    // proces noise setup, faster response
    // can be finetuned these values for the actual config
    KF.processNoiseCov.at<float>(10) = pow(10, -2);
    KF.processNoiseCov.at<float>(15) = pow(10, -2);
    KF.processNoiseCov.at<float>(0) = pow(10, -2.5);
    KF.processNoiseCov.at<float>(5) = pow(10, -2.5);
    // measurment noise setup - look for the actual values from the experiments
    setIdentity(KF.measurementNoiseCov, Scalar::all(2e-2));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
    kalmanVec.clear();
  }

  Point KFestimation(Point at)
  {
    //estimation
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
    measurmentVP(0) = at.x;
    measurmentVP(1) = at.y;
    // measurement += KF.measurementMatrix*state;
    Mat estimated = KF.correct(measurmentVP);
    Point statePt(estimated.at<float>(0), estimated.at<float>(1));
    kalmanVec.push_back(statePt);
    return statePt;
  }

  // function processing the navigation data from diverse sensors and observations. TAG detection included.
  void navCallback(const tagfollower_ros2::msg::Navdata::SharedPtr usm)
  {
    // Convert stamp to seconds
    double stamp_sec = static_cast<double>(usm->header.stamp.sec) + 
                        static_cast<double>(usm->header.stamp.nanosec) / 1e9;
    [[maybe_unused]] double deltT = (stamp_sec - lastTimeStamp) * 10;
                
    // Check if any tag was detected
    if (usm->tags_count == 1) {
      // scale it down and take the middle part
      position1.x = usm->tags_xc[0] * 64/120;
      position1.y = usm->tags_yc[0] * 36/120;
      position2.x = position1.x + usm->tags_width[0] * 64/120; // -360/2;
      position2.y = position1.y + usm->tags_height[0] * 36/120; // -640/2;        
      
      estimation = KFestimation(Point(position1.x, position1.y));
    } else {
      // no tag detected, go for blind prediction
      position1.x = -1;
      position1.y = -1;
      position2.x = -1;
      position2.y = -1;        
      Mat prediction = KF.predict();        
      estimation = Point(prediction.at<float>(0), prediction.at<float>(1));
    }    
    lastTimeStamp = stamp_sec;
  }

  // image processing and visualization function
  void imageCallback(const std::shared_ptr<const sensor_msgs::msg::Image>& msg_ptr)
  {
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(msg_ptr, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  
    Scalar red = CV_RGB(220, 0, 0);
    cv::rectangle(cv_image->image, estimation, Point(estimation.x+70, estimation.y+25), Scalar(220,0,0), 3);
    cv::rectangle(cv_image->image, position1, position2, red, 3);
    cv::circle(cv_image->image, position1, 3, Scalar(220,0,0), -1, 8, 0);
    cv::circle(cv_image->image, position2, 3, Scalar(120,0,0), -1, 8, 0);
  
    cv::imshow("Image window", cv_image->image);
    cv::waitKey(3);
  }
  
  // Timer callback replaces the original main loop
  void timerCallback()
  {
    // Main loop
    // Calculate the control command
    // Publish the control command
    //cmd_vel_publisher_->publish(command);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TagFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}