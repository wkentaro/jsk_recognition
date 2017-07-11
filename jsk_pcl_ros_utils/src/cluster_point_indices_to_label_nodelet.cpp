/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Kentaro Wada and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of Kentaro Wada and JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <boost/assign.hpp>
#include <map>

#include "jsk_topic_tools/log_utils.h"
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "jsk_pcl_ros_utils/cluster_point_indices_to_label.h"

namespace jsk_pcl_ros_utils
{

void ClusterPointIndicesToLabel::onInit()
{
  DiagnosticNodelet::onInit();
  pub_label_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  onInitPostProcess();
}

void ClusterPointIndicesToLabel::subscribe()
{
  sub_info_ = pnh_->subscribe("input/camera_info", 1,
                              &ClusterPointIndicesToLabel::cameraInfoCb,
                              this);
  sub_indices_ = pnh_->subscribe("input/cluster_indices", 1,
                                 &ClusterPointIndicesToLabel::convert,
                                 this);
}

void ClusterPointIndicesToLabel::cameraInfoCb(
  const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  height_ = caminfo_msg->height;
  width_ = caminfo_msg->width;
}

void ClusterPointIndicesToLabel::unsubscribe()
{
  sub_info_.shutdown();
  sub_indices_.shutdown();
}

void ClusterPointIndicesToLabel::convert(
  const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_indices_msg)
{
  vital_checker_->poke();

  if (height_ < 0 || width_ < 0)
  {
    NODELET_ERROR_THROTTLE(10, "Height or width is not initialized from camera info.");
    return;
  }
  int height, width;
  {
    boost::mutex::scoped_lock lock(mutex_);
    height = height_;
    width = width_;
  }

  cv::Mat label = cv::Mat::zeros(height, width, CV_32SC1);
  label.setTo(-1);

  for (size_t i = 0; i < cluster_indices_msg->cluster_indices.size(); i++)
  {
    pcl_msgs::PointIndices indices = cluster_indices_msg->cluster_indices[i];
    for (size_t j = 0; j < indices.indices.size(); j++)
    {
      int index = indices.indices[j];
      int img_x = index % width;
      int img_y = index / width;
      int instance_id = i;
      label.at<int>(img_y, img_x) = instance_id;
    }
  }

  pub_label_.publish(cv_bridge::CvImage(
    cluster_indices_msg->header,
    sensor_msgs::image_encodings::TYPE_32SC1,
    label).toImageMsg());
}

}  // namespace jsk_pcl_ros_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::ClusterPointIndicesToLabel, nodelet::Nodelet);
