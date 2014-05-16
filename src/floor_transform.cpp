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
      :  pub(nh.advertise<sensor_msgs::PointCloud2>("plane", 1)),
         sub(nh.subscribe<sensor_msgs::PointCloud2>("/uos_3dscans", 1, &FloorTransform::getPlain, this))
   {

   }

   FloorTransform::~FloorTransform()
   {

   }

   void FloorTransform::getPlain(const sensor_msgs::PointCloud2 ros_cloud){

      pcl::PCLPointCloud2::Ptr
         cloud_blob (new pcl::PCLPointCloud2),
         cloud_filtered_blob (new pcl::PCLPointCloud2);

      pcl::PointCloud<pcl::PointXYZ>::Ptr
         cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
         cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
         cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

      pcl_conversions::toPCL(ros_cloud, *cloud_blob);

      std::cerr << "PointCloud before filtering: "
         << cloud_blob->width * cloud_blob->height
         << " data points." << std::endl;

      // filter PCLPointCloud2 cloud_blob with VoxelGrid to cloud_filtered_blob
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud_blob);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*cloud_filtered_blob);

      // convert PCLPointCloud2 to PointCloud<pcl::PointXYZ> cloud_filtered
      pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

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
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (1000);
      seg.setDistanceThreshold (0.01);

      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      int nr_points = (int) cloud_filtered->points.size ();
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

         sensor_msgs::PointCloud2 pub_plane_cloud;
         pcl::toROSMsg<pcl::PointXYZ>(*cloud_p, pub_plane_cloud);
         pub.publish(pub_plane_cloud);

         // Create the filtering object
         extract.setNegative (true);
         extract.filter (*cloud_f);
         cloud_filtered.swap (cloud_f);
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


