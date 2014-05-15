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
 * floor_transform.cpp
 *
 *  Created on: 15.05.2013
 *     Authors: Sebastian Puetz <spuetz@uni-osnabrueck.de>
 */

#include <slam6d_node.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace floor_transform{

FlootTransform::srvGetTransform(sensor_msgs::PointCloud2& ros_cloud){

   pcl::PCLPointCloud2 pcl_cloud;
   pcl_conversions::toPCL(ros_cloud, pcl_cloud);
   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::fromPCLPointCloud2(pcl_cloud, cloud);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud.makeShared();

   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.01);
   seg.setMaxIterations (1000);

   int i = 0, nr_points = (int) cloud_filtered->points.size ();
   // While 30% of the original cloud is still there
   while (cloud_filtered->points.size () > 0.3 * nr_points)
   {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
         std::cerr << "Could not estimate a planar model for the given dataset."
            << std::endl;
         break;
      }

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);
      std::cerr << "PointCloud representing the planar component: "
         << cloud_p->width * cloud_p->height << " data points." << std::endl;

      std::cerr << "Model coefficients: "
         << coefficients->values[0] << " "
         << coefficients->values[1] << " "
         << coefficients->values[2] << " "
         << coefficients->values[3] << std::endl;

      std::cerr << "Model inliers: "
         << inliers->indices.size () << std::endl;
      for (size_t i = 0; i < inliers->indices.size (); ++i)
      {
         std::cerr << inliers->indices[i] << "    "
            << cloud->points[inliers->indices[i]].x << " "
            << cloud->points[inliers->indices[i]].y << " "
            << cloud->points[inliers->indices[i]].z << std::endl;
      }

      std::stringstream ss;
      ss << "plane_" << i << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);
      i++;
   }

   return (0);
}
}
