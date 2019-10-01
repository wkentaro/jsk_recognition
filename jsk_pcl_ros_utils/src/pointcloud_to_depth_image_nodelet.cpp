// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros_utils/pointcloud_to_depth_image.h"
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_pcl_ros_utils
{
  void PointCloudToDepthImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PointCloudToDepthImage::subscribe()
  {
    sub_cloud_ = pnh_->subscribe<sensor_msgs::PointCloud2>(
        "input", 1, &PointCloudToDepthImage::convert, this);
  }

  void PointCloudToDepthImage::unsubscribe()
  {
    sub_cloud_.shutdown();
  }

  void PointCloudToDepthImage::convert(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pc);

    if (!pc->isOrganized())
    {
      NODELET_FATAL("Input point cloud is not organized.");
      return;
    }

    cv::Mat depth_image = cv::Mat(
      cloud_msg->height,
      cloud_msg->width,
      CV_32FC1,
      std::numeric_limits<double>::quiet_NaN());
    for (size_t index = 0; index < pc->points.size(); index++)
    {
      int width_index = index % cloud_msg->width;
      int height_index = index / cloud_msg->width;
      depth_image.at<float>(height_index, width_index) = pc->points[index].z;
    }
    cv_bridge::CvImage depth_bridge(cloud_msg->header,
                                    sensor_msgs::image_encodings::TYPE_32FC1,
                                    depth_image);
    pub_.publish(depth_bridge.toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::PointCloudToDepthImage, nodelet::Nodelet);
