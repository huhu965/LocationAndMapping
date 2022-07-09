/*
 * @Author: Hu Ziwei 
 * @Description:  imu数据
 * @Date: 2021-12-28 12:57:53 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-28 13:08:57
 */
#include <cmath>

#include "sensor_data/imu_data.hpp"

namespace loam_frame{
Eigen::Matrix3d IMUData::GetOrientationMatrix() {
    Eigen::Matrix3d matrix = orientation.matrix().cast<double>();
    return matrix;
}

int IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, std::list<double> &sync_time_list) {
    while(sync_time_list.size()!=0){
        double _sync_time = sync_time_list.front();
        bool _need_interpolation = false;
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        while (UnsyncedData.back().time >= _sync_time && UnsyncedData.size() > 1) {  //imu未同步时间大于待插值时间，才能继续干啊
            //点云获取时间前没有imu数据，雷达数据该抛掉了
            if (UnsyncedData.front().time > _sync_time){
                sync_time_list.pop_front();
                break;
            }
            
            if (UnsyncedData.at(1).time < _sync_time) {
                SyncedData.push_back(UnsyncedData.front());
                UnsyncedData.pop_front();
                continue;
            }

            if(UnsyncedData.front().time<=_sync_time && UnsyncedData.at(1).time >= _sync_time){
                _need_interpolation = true;
                sync_time_list.pop_front();
                break;
            }
        }

        if (UnsyncedData.size() < 2)
            break;

        IMUData front_data = UnsyncedData.at(0);
        IMUData back_data = UnsyncedData.at(1);
        IMUData synced_data;
        SyncedData.push_back(UnsyncedData.front());
        UnsyncedData.pop_front();

        double front_scale = (back_data.time - _sync_time) / (back_data.time - front_data.time);
        double back_scale = (_sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = _sync_time;
        synced_data.linear_acceleration[0] = front_data.linear_acceleration[0] * front_scale + back_data.linear_acceleration[0] * back_scale;
        synced_data.linear_acceleration[1] = front_data.linear_acceleration[1] * front_scale + back_data.linear_acceleration[1] * back_scale;
        synced_data.linear_acceleration[2] = front_data.linear_acceleration[2] * front_scale + back_data.linear_acceleration[2] * back_scale;
        synced_data.angular_velocity[0] = front_data.angular_velocity[0] * front_scale + back_data.angular_velocity[0] * back_scale;
        synced_data.angular_velocity[1] = front_data.angular_velocity[1] * front_scale + back_data.angular_velocity[1] * back_scale;
        synced_data.angular_velocity[2] = front_data.angular_velocity[2] * front_scale + back_data.angular_velocity[2] * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x() = front_data.orientation.x() * front_scale + back_data.orientation.x() * back_scale;
        synced_data.orientation.y() = front_data.orientation.y() * front_scale + back_data.orientation.y() * back_scale;
        synced_data.orientation.z() = front_data.orientation.z() * front_scale + back_data.orientation.z() * back_scale;
        synced_data.orientation.w() = front_data.orientation.w() * front_scale + back_data.orientation.w() * back_scale;
        // 线性插值之后要归一化
        synced_data.orientation.normalize();

        SyncedData.push_back(synced_data);

        return imu_sync_success;
    }
}

}