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

#include <floor_transform.h>

namespace floor_transform{

   FloorTransform::FloorTransform(ros::NodeHandle& nh)
   {
      nh.param("input_cloud", input_cloud, std::string("/uos_3dscans"));
      nh.param("output_cloud", output_cloud, std::string("/plane"));
      nh.param("min_z", min_z, -0.3);
      nh.param("max_z", max_z, 0.3);
      nh.param("delta_angle", delta_angle, 15.0);
      sub = nh.subscribe<sensor_msgs::PointCloud2>(input_cloud.c_str(), 1, &FloorTransform::getPlane, this);
      pub = nh.advertise<sensor_msgs::PointCloud2>(output_cloud.c_str(), 1);
   }

   FloorTransform::~FloorTransform()
   {

   }

   void FloorTransform::getPlane(const sensor_msgs::PointCloud2 ros_cloud){

      ROS_INFO("Got PointCloud2...");

      pcl::PCLPointCloud2::Ptr
         cloud (new pcl::PCLPointCloud2),
         cloud_crop (new pcl::PCLPointCloud2),
         cloud_voxel (new pcl::PCLPointCloud2);

      pcl::PointCloud<pcl::PointXYZ>::Ptr
         cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
         cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
         cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

      pcl_conversions::toPCL(ros_cloud, *cloud);

      std::cerr << "PointCloud before filtering: "
         << cloud->width * cloud->height
         << " data points." << std::endl;


      Eigen::Vector4f z_min, z_max;

      z_min[0] = -INFINITY;
      z_min[1] = -INFINITY;
      z_min[2] = min_z;

      z_max[0] = INFINITY;
      z_max[1] = INFINITY;
      z_max[2] = max_z;

      // filter PCLPointCloud2 cloud_blob with CropBox in Rang [z_min z_max]
      pcl::CropBox<pcl::PCLPointCloud2> crop;
      crop.setInputCloud (cloud);
      crop.setMin(z_min);
      crop.setMax(z_max);
      crop.filter(*cloud_crop);

      // filter PCLPointCloud2 cloud_blob with VoxelGrid to cloud_filtered_blob
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud_crop);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloud_voxel);


      // convert PCLPointCloud2 to PointCloud<pcl::PointXYZ> cloud_filtered
      pcl::fromPCLPointCloud2(*cloud_voxel, *cloud_filtered);

      std::cerr << "PointCloud after filtering: "
         << cloud_filtered->width * cloud_filtered->height
         << " data points." << std::endl;


      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;

      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory

      seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
      //seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.01);

      Eigen::Vector3f z_axis;
      z_axis[0] = 0.0;
      z_axis[1] = 0.0;
      z_axis[2] = 1.0;

      seg.setAxis(z_axis);
      seg.setEpsAngle(delta_angle * M_PI / 180);

      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
         std::cerr << "Could not estimate a planar model for the given dataset."
            << std::endl;
      }
      else
      {
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

         sensor_msgs::PointCloud2 pub_plane_cloud;
         pcl::toROSMsg<pcl::PointXYZ>(*cloud_p, pub_plane_cloud);
         pub.publish(pub_plane_cloud);
      }
   }
}

int main (int argc, char** argv)
{

   ros::init(argc, argv, "floor_transform");
   ros::NodeHandle nh;
   floor_transform::FloorTransform node(nh);
   ros::Rate loop_rate(10);
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
   return EXIT_SUCCESS;

}


