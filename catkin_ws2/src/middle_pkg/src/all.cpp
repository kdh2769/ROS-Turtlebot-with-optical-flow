#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>


#define MAX_LIN_VEL 0.26
#define MAX_ANG_VEL 1.82
#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1

#define NOTHING -1
#define KEYCODE_UP 0
#define KEYCODE_DOWN 1
#define KEYCODE_LEFT 2
#define KEYCODE_RIGHT 3

class ImageConvAndTeleopKey{
private:
  ros::NodeHandle nh_;
  ros::Publisher teleop_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::NodeHandle nt_;
  double target_linear_vel_;
  double target_angular_vel_;
  double control_linear_vel_;
  double control_angular_vel_;

  cv::Mat frame1_;
  cv::Mat prvs_;

  int first_check_;

public:
  ImageConvAndTeleopKey();
  ~ImageConvAndTeleopKey();
  void img_cb_calc_of(const sensor_msgs::ImageConstPtr& msg);
  double angular_ok(double input);
  double linear_ok(double input);
  void cmd_vel_pub (int optical_flow_direction);
  double makeSimpleProfile(double output, double input, double slop);
};
ImageConvAndTeleopKey::ImageConvAndTeleopKey() : it_(nh_)
{
  image_sub_ = it_.subscribe("/image_raw", 1, &ImageConvAndTeleopKey::img_cb_calc_of, this);
  teleop_pub_ = nt_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  first_check_ = 1;

  target_linear_vel_ = 0.0;
  target_angular_vel_= 0.0;
  control_linear_vel_= 0.0;
  control_angular_vel_= 0.0;

  cv::namedWindow("mission #1");
  cv::namedWindow("mission #2");
}

ImageConvAndTeleopKey::~ImageConvAndTeleopKey()
{
  cv::destroyWindow("mission #1");
  cv::destroyWindow("mission #2");
}

void ImageConvAndTeleopKey::img_cb_calc_of(const sensor_msgs::ImageConstPtr& msg)
{
  /* make first prvious frame */
  if (first_check_){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    frame1_ = cv_ptr->image;
    cv::cvtColor(frame1_, prvs_, cv::COLOR_BGR2GRAY);
    first_check_ = 0;
    cv::imshow("mission #1",frame1_);
    return ;
  }

  /* calculate optical flow */
  cv_bridge::CvImagePtr cv_ptr2;
  try
  {
    cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  /* mission #1*/
  cv::Mat frame2 = cv_ptr2->image;
  cv::imshow("mission #1",frame2);

  cv::Mat next;
  cvtColor(frame2, next, cv::COLOR_BGR2GRAY);
  cv::Mat flow(prvs_.size(), CV_32FC2);
  calcOpticalFlowFarneback(prvs_, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  cv::Mat flow_parts[2];
  split(flow, flow_parts);
  cv::Mat magnitude, angle, magn_norm;
  cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
  normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
  cv::Mat oang = angle * 3.141592 / 180.0;
  angle *= ((1.f / 360.f) * (180.f / 255.f));
  cv::Mat _hsv[3], hsv, hsv8, bgr;
  _hsv[0] = angle;
  _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
  _hsv[2] = magn_norm;
  merge(_hsv, 3, hsv);
  hsv.convertTo(hsv8, CV_8U, 255.0);
  cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

  /* cal 4 direction */
  int up = 0, down = 0, left = 0, right = 0;

  /* optical flow step */
  int step = 10; // 20
  cv::Mat img_vec = frame2;

  /* correct noise */
  int move = 0;

  for (int r = 0; r < angle.rows; r += step) {
    for (int c = 0; c < angle.cols; c += step) {
      float ang = oang.at<float>(r, c);

      /* correct noise */
      float threshold = 0.15;
      if (magn_norm.at<float>(r, c) < threshold
          and magn_norm.at<float>(r, c) > -threshold){
        magn_norm.at<float>(r, c) = 0;
      }
      else {
        move ++;
        /* optical flow direction line scale */
        float m = magn_norm.at<float>(r, c) * 50;
        cv::Point pt1 = cv::Point(c, r);
        cv::Point pt2 = cv::Point(c + m * cos(ang), r + m * sin(ang));

        if (magn_norm.at<float>(r,c ) != 0)
          line(img_vec, pt1, pt2, cv::Scalar(180, 105, 255), 1, 8, 0);

        float horizontal = m * cos(ang);
        float vertical = m * sin(ang);

        /* calculate direction */
        int threshold_direction = 0;
        if (vertical > threshold_direction) {
          if (horizontal > threshold_direction) {
            if (vertical > horizontal)
              down++;
            else
              left++;
          }
          else if (horizontal < -threshold_direction){
            if (vertical > -horizontal)
              down++;
            else
              right++;
          }
        }

        else if (vertical < -threshold_direction){
          if (horizontal > threshold_direction) {
            if (-vertical > horizontal)
              up++;
            else
              left++;
          }
          else if (horizontal <-threshold_direction){
            if (vertical < horizontal)
              up++;
            else
              right++;
          }
        }
      }
    }
  }
  /* ROS_INFO direction & publish "cmd_vel" */
  int max_id = -1;
  int background_threshold = 1200;

  if (move > 500){
    int direction_count[4] = { up, down, left, right };
    int max_index = 0;
    int max = direction_count[0];

    for (int i = 1; i < 4; i++) {
      if (direction_count[i] > max) {
        max_index = i;
        max = direction_count[i];
      }
    }

    if (max_index == 0){
      if (move > background_threshold){
        ROS_INFO("DOWN");
        max_id = 1;
      }
      else{
        ROS_INFO("UP");
        max_id = 0;
      }
    }else if (max_index == 1) {
      if (move > background_threshold){
        ROS_INFO("UP");
        max_id = 0;
      }
      else{
        ROS_INFO("DOWN");
        max_id = 1;
      }
    }else if (max_index == 2) {
      ROS_INFO("LEFT");
      max_id = max_index;
    } else if(max_index == 3) {
      ROS_INFO("RIGHT");
      max_id = max_index;
    }
    /* background move */
    // ROS_INFO("direction : %d", move);
    cmd_vel_pub(max_id);
  }
  imshow("mission #2", img_vec);
  cv::waitKey(3);
  prvs_ = next;
}
double ImageConvAndTeleopKey::angular_ok(double input){
   if(input > MAX_ANG_VEL)
     input = MAX_ANG_VEL;
   else if (input < -MAX_ANG_VEL)
     input = -MAX_ANG_VEL;
   return input;
}
double ImageConvAndTeleopKey::linear_ok(double input){
  if (input > MAX_LIN_VEL)
    input = MAX_LIN_VEL;
  else if (input < -MAX_LIN_VEL)
    input = -MAX_LIN_VEL;
  return input;
}

double ImageConvAndTeleopKey::makeSimpleProfile(double output, double input, double slop){
  if (input > output){
    if (input < output + slop)
      output = input;
    else
      output = output + slop;
  }
  else if (input < output){
    if ( input > output - slop)
      output = input;
    else
      output =  output - slop;
  }
  else
    output = input;

  return output;
}

void ImageConvAndTeleopKey::cmd_vel_pub (int optical_flow_direction){
  int direction = optical_flow_direction;
  geometry_msgs::Twist vel;

  if (direction != NOTHING){
    if (direction == KEYCODE_UP){
      target_linear_vel_ = linear_ok(target_linear_vel_ + LIN_VEL_STEP_SIZE);
    } else if (direction == KEYCODE_DOWN) {
      target_linear_vel_ = linear_ok(target_linear_vel_ - LIN_VEL_STEP_SIZE);

    } else if (direction == KEYCODE_LEFT) {
      target_angular_vel_ = angular_ok(target_angular_vel_ + ANG_VEL_STEP_SIZE);

    } else if (direction == KEYCODE_RIGHT){
      target_angular_vel_ = angular_ok(target_angular_vel_ - ANG_VEL_STEP_SIZE);
    }

    control_linear_vel_ = makeSimpleProfile(control_linear_vel_, target_linear_vel_, LIN_VEL_STEP_SIZE/2.0);
    vel.linear.x = control_linear_vel_;

    control_angular_vel_ = makeSimpleProfile(control_angular_vel_, target_angular_vel_, ANG_VEL_STEP_SIZE/2.0);
    vel.angular.z = control_angular_vel_;

    ROS_INFO("linear x : %f", control_linear_vel_ );
    ROS_INFO("angular z : %f", control_angular_vel_);


    teleop_pub_.publish(vel);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConvAndTeleopKey ic;
  ros::spin();
  return 0;
}
