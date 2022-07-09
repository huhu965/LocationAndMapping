/*
 * @Author: Hu Ziwei 
 * @Description:IMU数据格式
 * @Date: 2021-11-08 15:53:22 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-28 13:09:00
 */
#ifndef LOAM_FRAME_SENSOR_DATA_IMU_DATA_HPP_
#define LOAM_FRAME_SENSOR_DATA_IMU_DATA_HPP_
#include <deque>
#include <list>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


namespace loam_frame {
enum imu_sync_status{
    imu_sync_success = 0,
    not_imu_data_before_cloud,
    not_imu_data_behind_cloud
};

class IMUData {
public:
    IMUData() = default;
    ~IMUData() = default;

    Eigen::Matrix3d GetOrientationMatrix(); // 把四元数转换成旋转矩阵送出去
    static int SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, std::list<double> &sync_time_list);
public:
    Eigen::Quaterniond orientation;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
    double time;
};
}

#endif //LOAM_FRAME_SENSOR_DATA_GNSS_DATA_HPP_