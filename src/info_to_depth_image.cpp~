#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void InfoToDepthImage::onInit()
  {
    pnh_->param("frame_height", frame_height, 2.0);
    pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

void InfoToDepthImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &InfoToDepthImage::convert, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

void InfoToDepthImage::convert(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    cv::Mat depth_mat(info_msg->height,
                      info_msg->width,
                      CV_32FC1);
    depth_mat.setTo(frame_height);
                                      
    pub_depth_.publish(cv_bridge::CvImage(info_msg->header,
                                          sensor_msgs::image_encodings::TYPE_32FC1,
                                          depth_mat).toImageMsg());
  }
