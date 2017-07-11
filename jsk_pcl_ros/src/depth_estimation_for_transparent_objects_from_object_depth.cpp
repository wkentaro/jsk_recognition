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

#include <limits>
#include <numeric>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>

#include "jsk_pcl_ros/depth_estimation_for_transparent_objects_from_object_depth.h"

namespace jsk_pcl_ros
{
  void DepthEstimationForTransparentObjectsFromObjectDepth::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_depth_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void DepthEstimationForTransparentObjectsFromObjectDepth::subscribe()
  {
    sub_depth_.subscribe(*pnh_, "input/depth", 1);
    sub_indices_.subscribe(*pnh_, "input/cluster_indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_depth_, sub_indices_);
    sync_->registerCallback(
      boost::bind(&DepthEstimationForTransparentObjectsFromObjectDepth::estimate,
                  this, _1, _2));
  }

  void DepthEstimationForTransparentObjectsFromObjectDepth::unsubscribe()
  {
    sub_depth_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void DepthEstimationForTransparentObjectsFromObjectDepth::estimate(
    const sensor_msgs::Image::ConstPtr& depth_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_indices_msg)
  {
    vital_checker_->poke();
    if (depth_msg->header.frame_id != cluster_indices_msg->header.frame_id)
    {
      NODELET_FATAL("Depth and cluster indices have different frame_id: [%s, %s]",
                    depth_msg->header.frame_id.c_str(),
                    cluster_indices_msg->header.frame_id.c_str());
      return;
    }
    if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      NODELET_FATAL("Unsupported encoding. Expected: 32FC1, Actual: %s",
                    depth_msg->encoding.c_str());
      return;
    }
    cv::Mat depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;

    std::map<int, std::vector<float> > instance_depths;
    for (int i = 0; i < cluster_indices_msg->cluster_indices.size(); i++)
    {
      pcl_msgs::PointIndices indices = cluster_indices_msg->cluster_indices[i];
      for (size_t j = 0; j < indices.indices.size(); j++)
      {
        int index = indices.indices[j];
        int img_y = index / depth.cols;
        int img_x = index % depth.cols;
        float depth_value = depth.at<float>(img_y, img_x);
        if (!std::isnan(depth_value))
        {
          instance_depths[i].push_back(depth_value);
        }
      }
    }

    cv::Mat depth_estimated = cv::Mat(depth.rows, depth.cols, CV_32FC1);
    depth_estimated.setTo(std::numeric_limits<float>::quiet_NaN());
    for (std::map<int, std::vector<float> >::iterator it = instance_depths.begin();
         it != instance_depths.end(); it++)
    {
      float object_depth_ref = 0.1;  // 10cm
      int i = it->first;
      std::vector<float> depth_values = it->second;

      std::pair<std::vector<float>::const_iterator, std::vector<float>::const_iterator> minmax =
        std::minmax_element(depth_values.begin(), depth_values.end());
      float min_depth = *minmax.first;
      float max_depth = *minmax.second;

      pcl_msgs::PointIndices indices = cluster_indices_msg->cluster_indices[i];
      for (size_t j = 0; j < indices.indices.size(); j++)
      {
        int index = indices.indices[j];
        int img_y = index / depth.cols;
        int img_x = index % depth.cols;

        float depth_value = depth.at<float>(img_y, img_x);
        if (std::isnan(depth_value))
        {
          continue;
        }

        float object_depth = depth_value - min_depth;
        if (object_depth < object_depth_ref)
        {
          // max_depth is surface of other object for transparent object.
          // object_depth_ref is a minimum thickness of object.
          // so estimated surface depth of transparent object is max_depth - object_depth_ref.
          depth_estimated.at<float>(img_y, img_x) = max_depth - object_depth_ref;
        }
      }
    }

    pub_depth_.publish(cv_bridge::CvImage(
      depth_msg->header,
      sensor_msgs::image_encodings::TYPE_32FC1,
      depth_estimated));

  }
}  // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::DepthEstimationForTransparentObjectsFromObjectDepth, nodelet::Nodelet);
