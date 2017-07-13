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

#ifndef JSK_PCL_ROS_POINTCLOUD_XYZLL_DECOMPOSER_H_
#define JSK_PCL_ROS_POINTCLOUD_XYZLL_DECOMPOSER_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"
#include "jsk_recognition_utils/tf_listener_singleton.h"

#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "jsk_recognition_utils/pcl_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_topic_tools/diagnostic_nodelet.h"
#include "jsk_pcl_ros/ClusterPointIndicesDecomposerConfig.h"

#include "jsk_pcl_ros/cluster_point_indices_decomposer.h"

namespace jsk_pcl_ros
{
  struct PointXYZLL
  {
    float x;
    float y;
    float z;
    uint32_t label_class;
    uint32_t label_instance;
  };

  class PointCloudXYZLLDecomposer: public ClusterPointIndicesDecomposer
  {
  public:
    virtual void onInit();
    virtual void inputCb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void subscribe();
    virtual void unsubscribe();
  protected:
    ros::Subscriber sub_cloud_;
  };

}  // namespace jsk_pcl_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
  jsk_pcl_ros::PointXYZLL,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (uint32_t, label_class, label_class)
  (uint32_t, label_instance, label_instance)
)

#endif  // JSK_PCL_ROS_POINTCLOUD_XYZLL_DECOMPOSER_H_
