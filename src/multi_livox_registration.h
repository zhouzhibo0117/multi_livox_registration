//
// Created by localization on 6/24/19.
//

#ifndef SRC_MULTI_LIVOX_REGISTRATION_H
#define SRC_MULTI_LIVOX_REGISTRATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define MAX_FRAME_GAP 0.01

class MutilLivoxRegistration {
private:
    // ROS related.
    ros::NodeHandle nh_;
    ros::Subscriber livox_subscriber_;
    ros::Publisher pointcloud_publisher_;

    // Pointcloud vector that saves clouds from single sensors.
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vec_pointcloud_single_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_single_;

    // Timestamp base;
    double timestamp_base_;

    void LivoxHandler(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {
        double timestamp = cloud_in->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_in,*pointcloud_single_);

        if (timestamp - timestamp_base_ > MAX_FRAME_GAP) {

            ROS_DEBUG("Time Gap: %f, Total frame: %d", timestamp - timestamp_base_, vec_pointcloud_frame_.size());

            vec_pointcloud_frame_.clear();

            // Update timestamp base.
            timestamp_base_ = timestamp;
        }

        // old
        vec_pointcloud_frame_.push_back(cloud_in);

        if (vec_pointcloud_frame_.size()==8){
            //
        }


    }


public:
    MutilLivoxRegistration() {

        timestamp_base_ = 0.0;
        pointcloud_single_=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

        livox_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/hub",
                                                                    1,
                                                                    &MutilLivoxRegistration::LivoxHandler,
                                                                    this);

        pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox_hub_points", 1);
    }

    ~MutilLivoxRegistration() = default;
};


#endif //SRC_MULTI_LIVOX_REGISTRATION_H
