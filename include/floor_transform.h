/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * floor_transform.h
 *
 *  Created on: 15.05.2013
 *     Authors: Sebastian Puetz <spuetz@uni-osnabrueck.de>
 */

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_datatypes.h>
#include <floor_transform/GetTransform.h>

namespace floor_transform{

   class FloorTransform{

      public:
         FloorTransform(ros::NodeHandle& nh);
         ~FloorTransform();
      private:
         bool getTransformation(
            GetTransform::Request& request,
            GetTransform::Response& response);

         void getPlane(const sensor_msgs::PointCloud2& ros_cloud);
         bool getPlane(
            const sensor_msgs::PointCloud2& ros_cloud,
            geometry_msgs::PoseStamped& transform, bool publish_plane);

         ros::Publisher pub;
         ros::Subscriber sub;
         ros::ServiceServer srv;
         double delta_angle, min_z, max_z;
         std::string input_cloud, output_cloud;
   };

}
