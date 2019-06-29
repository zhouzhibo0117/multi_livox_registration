#include <cmath>

#include <cmath>

//
// Created by localization on 6/24/19.
//

#ifndef SRC_MULTI_LIVOX_REGISTRATION_H
#define SRC_MULTI_LIVOX_REGISTRATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ctime>

#define MAX_FRAME_GAP 0.01
#define LIVOX_NUM 8
#define LIVOX_NUM_MAX 32
#define LIVOX_FREQUENCY 20
#define FRAME_INTERVAL 2

class MutilLivoxRegistration {
private:
    // ROS related.
    ros::NodeHandle nh_;
    ros::Subscriber livox_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher pointcloud_publisher_;

    // Pointcloud vector that saves clouds from single sensors.
//    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vec_pointcloud_single_; // TODO: 为什么vector不行？
    pcl::PointCloud<pcl::PointXYZI>::Ptr arr_pointcloud_single_[LIVOX_NUM];
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_single_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_all_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_to_publish_;

    // Timestamp base;
    double timestamp_base_;

    int livox_count;
    int frame_count;

    // Calibration config.
    struct CalibrationInfo {
        double x;
        double y;
        double z;
        double rx;
        double ry;
        double rz;
    };
    CalibrationInfo arr_livox_cali_info[LIVOX_NUM_MAX];

    // Odometry and IMU info.
    double linear_velocity_XYplane_;
    double angular_velocity_rz_;

    void LivoxHandler(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {

        clock_t startTime = clock();

        double timestamp = cloud_in->header.stamp.toSec();
        pointcloud_single_->clear();
        pcl::fromROSMsg(*cloud_in, *pointcloud_single_);

        if (fabs(timestamp - timestamp_base_) > MAX_FRAME_GAP) {
            // Update timestamp base.
            timestamp_base_ = timestamp;
        }

        *arr_pointcloud_single_[livox_count] = *pointcloud_single_;
        livox_count++;

        if (livox_count == 8) {

            pointcloud_all_->clear();

            // livox_id: 0       3       4       5       21      24      25      26
//            for (int i = 0; i < LIVOX_NUM; ++i) {
//                float raw_intensity = arr_pointcloud_single_[i]->points[0].intensity;
//                int livox_id = round((raw_intensity - float(int(raw_intensity))) * 1000);
//                std::cout<<livox_id<<'\t';
//            }

            for (int i = 0; i < LIVOX_NUM; ++i) {
                // Find livox id.
//                float raw_intensity = arr_pointcloud_single_[i]->points[0].intensity;
//                int livox_id = round((raw_intensity - float(int(raw_intensity))) * 1000);
//
//                    TransfromToLivoxCoordinate(arr_pointcloud_single_[i],
//                                               arr_livox_cali_info[livox_id].x,
//                                               arr_livox_cali_info[livox_id].y,
//                                               arr_livox_cali_info[livox_id].z,
//                                               arr_livox_cali_info[livox_id].rx,
//                                               arr_livox_cali_info[livox_id].ry,
//                                               arr_livox_cali_info[livox_id].rz);

                // Motion compensation.
                CompensateMotionInFrame(arr_pointcloud_single_[i],
                                        linear_velocity_XYplane_,
                                        angular_velocity_rz_);


                *pointcloud_all_ += *arr_pointcloud_single_[i];

            }


            livox_count = 0;

//            ROS_INFO_STREAM("Running time difference: " << double(clock() - startTime) / CLOCKS_PER_SEC << " s.");


            frame_count++;

            if (frame_count < FRAME_INTERVAL) {
                *pointcloud_to_publish_ += *pointcloud_all_;

                // Transfrom to next frame.
                CompensateMotionAmongFrame(pointcloud_to_publish_,
                                           linear_velocity_XYplane_,
                                           angular_velocity_rz_);
            } else {
                frame_count = 0;

                *pointcloud_to_publish_ += *pointcloud_all_;

                ROS_INFO_STREAM(pointcloud_to_publish_->points.size());

                // Publish point cloud.
                sensor_msgs::PointCloud2 cloud_out;
                pcl::toROSMsg(*pointcloud_to_publish_, cloud_out);
                cloud_out.header.stamp = ros::Time().fromSec(timestamp_base_);
                cloud_out.header.frame_id = "/livox_frame";
                pointcloud_publisher_.publish(cloud_out);

                pointcloud_to_publish_->clear();

                ROS_INFO_STREAM("Publishing running time: " << double(clock() - startTime) / CLOCKS_PER_SEC << " s.");
            }

        }
    }

    void OdomHandler(const nav_msgs::OdometryConstPtr &odom_in) {
        linear_velocity_XYplane_ = sqrt(odom_in->twist.twist.linear.x * odom_in->twist.twist.linear.x +
                                        odom_in->twist.twist.linear.y * odom_in->twist.twist.linear.y);
    }

    void ImuHandler(const sensor_msgs::ImuConstPtr &imu_in) {
        angular_velocity_rz_ = imu_in->angular_velocity.z;
    }

    void TransfromToLivoxCoordinate(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                    double tx, double ty, double tz,
                                    double rx, double ry, double rz) {

        for (int i = 0; i < cloud->points.size(); ++i) {
            cloud->points[i].x -= tx;
            cloud->points[i].y -= ty;
            cloud->points[i].z -= tz;

            double tempX = cloud->points[i].x;
            double tempY = cloud->points[i].y;

            cloud->points[i].x = cos(rz) * tempX + sin(rz) * tempY;
            cloud->points[i].y = -sin(rz) * tempX + cos(rz) * tempY;

        }

    }

    void CompensateMotionInFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                 double linear_velocity_xy,
                                 double angular_velocity_rz) {
        double coefficient = 1 / double(LIVOX_FREQUENCY) / double(cloud->points.size());
        double angle_coefficient = angular_velocity_rz * coefficient;
        double translation_coefficient = linear_velocity_xy * coefficient;
        double angle = 0.0;
        double translation = 0.0;

        for (int i = 0; i < cloud->points.size(); ++i) {

            angle += angle_coefficient;
            translation += translation_coefficient;

            double cos_theta = cos(angle);
            double sin_theta = sin(angle);

            double tempX = cos_theta * cloud->points[i].x - sin_theta * cloud->points[i].y + translation * cos_theta;
            double tempY = sin_theta * cloud->points[i].x + cos_theta * cloud->points[i].y + translation * sin_theta;

            cloud->points[i].x = tempX;
            cloud->points[i].y = tempY;
        }

    }

    void CompensateMotionAmongFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                    double linear_velocity_xy,
                                    double angular_velocity_rz) {

        double angle = angular_velocity_rz / double(LIVOX_FREQUENCY);
        double translation = linear_velocity_xy / double(LIVOX_FREQUENCY);

        double cos_theta = cos(angle);
        double sin_theta = sin(angle);

        for (int i = 0; i < cloud->points.size(); ++i) {

            double tempX = cos_theta * cloud->points[i].x + sin_theta * cloud->points[i].y - translation;
            double tempY = -sin_theta * cloud->points[i].x + cos_theta * cloud->points[i].y;

            cloud->points[i].x = tempX;
            cloud->points[i].y = tempY;
        }

    }


public:
    MutilLivoxRegistration() {

        timestamp_base_ = 0.0;
        pointcloud_single_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pointcloud_all_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pointcloud_to_publish_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

        pointcloud_to_publish_->clear();
        for (int i = 0; i < LIVOX_NUM; ++i) {
            arr_pointcloud_single_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        }

        livox_count = 0;
        frame_count = 0;

        linear_velocity_XYplane_ = 0;
        angular_velocity_rz_ = 0;

        // Calibration init.
        arr_livox_cali_info[0] = {-0.16, -0.16, 0.737 - 0.0737, 0, 0, -0.75 * M_PI};
        arr_livox_cali_info[3] = {0.18, -0.18, 0.397 - 0.0397, 0, 0, -0.2417 * M_PI + 0.167 * M_PI};
        arr_livox_cali_info[4] = {0.18, -0.18, 0.397 - 0.0397, 0, 0, -0.2417 * M_PI};
        arr_livox_cali_info[5] = {0.18, -0.18, 0.397 - 0.0397, 0, 0, -0.2417 * M_PI - 0.167 * M_PI};
        arr_livox_cali_info[21] = {-0.16, 0.16, 0.737 - 0.0737, 0, 0, 0.75 * M_PI};
        arr_livox_cali_info[24] = {0.18, 0.18, 0.397 - 0.0397, 0, 0, 0.25 * M_PI + 0.167 * M_PI};
        arr_livox_cali_info[25] = {0.18, 0.18, 0.397 - 0.0397, 0, 0, 0.25 * M_PI};
        arr_livox_cali_info[26] = {0.18, 0.18, 0.397 - 0.0397, 0, 0, 0.25 * M_PI - 0.167 * M_PI};

        livox_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/hub",
                                                                    1000,
                                                                    &MutilLivoxRegistration::LivoxHandler,
                                                                    this);

        odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/Inertial/gps/odom",
                                                             1000,
                                                             &MutilLivoxRegistration::OdomHandler,
                                                             this);

        imu_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("/Inertial/imu/data",
                                                          1000,
                                                          &MutilLivoxRegistration::ImuHandler,
                                                          this);

        pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox_hub_points", 1000);
    }

    ~MutilLivoxRegistration() {}
};


#endif //SRC_MULTI_LIVOX_REGISTRATION_H
