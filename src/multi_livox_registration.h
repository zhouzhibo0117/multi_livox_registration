//
// Created by localization on 6/24/19.
//

#ifndef SRC_MULTI_LIVOX_REGISTRATION_H
#define SRC_MULTI_LIVOX_REGISTRATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ctime>

#define MAX_FRAME_GAP 0.01
#define LIVOX_NUM 8

class MutilLivoxRegistration {
private:
    // ROS related.
    ros::NodeHandle nh_;
    ros::Subscriber livox_subscriber_;
    ros::Publisher pointcloud_publisher_;

    // Pointcloud vector that saves clouds from single sensors.
//    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vec_pointcloud_single_; // TODO: 为什么vector不行？
    pcl::PointCloud<pcl::PointXYZI>::Ptr arr_pointcloud_single_[LIVOX_NUM];
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_single_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_all_;

    // Timestamp base;
    double timestamp_base_;

    int livox_count;

    void LivoxHandler(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {

        clock_t startTime = clock();

        double timestamp = cloud_in->header.stamp.toSec();
        pointcloud_single_->clear();
        pcl::fromROSMsg(*cloud_in, *pointcloud_single_);

        if (timestamp - timestamp_base_ > MAX_FRAME_GAP) {
            // Update timestamp base.
            timestamp_base_ = timestamp;
        }

        *arr_pointcloud_single_[livox_count] = *pointcloud_single_;
        livox_count++;

        if (livox_count == 8) {

            pointcloud_all_->clear();

            for (int i = 0; i < LIVOX_NUM; ++i) {
                *pointcloud_all_ += *arr_pointcloud_single_[i];
            }

            ROS_INFO_STREAM(pointcloud_all_->points.size());

            // Publish point cloud.
            sensor_msgs::PointCloud2 cloud_out;
            pcl::toROSMsg(*pointcloud_all_, cloud_out);
            cloud_out.header.stamp = ros::Time().fromSec(timestamp_base_);
            cloud_out.header.frame_id = "/livox_frame";
            pointcloud_publisher_.publish(cloud_out);

            livox_count = 0;

            ROS_INFO_STREAM("Running time difference: "<<double(clock() - startTime)/CLOCKS_PER_SEC<<" s.");

        }
    }


public:
    MutilLivoxRegistration() {

        timestamp_base_ = 0.0;
        pointcloud_single_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pointcloud_all_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        for (int i = 0; i < LIVOX_NUM; ++i) {
            arr_pointcloud_single_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

        livox_count = 0;


        livox_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/hub",
                                                                    1000,
                                                                    &MutilLivoxRegistration::LivoxHandler,
                                                                    this);

        pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox_hub_points", 1000);
    }

    ~MutilLivoxRegistration() {}
};


#endif //SRC_MULTI_LIVOX_REGISTRATION_H
