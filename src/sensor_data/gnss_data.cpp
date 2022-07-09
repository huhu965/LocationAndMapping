/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-23 10:57:15 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2022-04-14 21:08:26
 */
#include "sensor_data/gnss_data.hpp"
#include <iostream>

//静态成员变量必须在类外初始化
double loam_frame::GNSSData::origin_longitude = 0.0;
double loam_frame::GNSSData::origin_latitude = 0.0;
double loam_frame::GNSSData::origin_altitude = 0.0;

bool loam_frame::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian loam_frame::GNSSData::geo_converter;

namespace loam_frame{

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}


void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        std::cout << "GeoConverter has not set origin position"<<std::endl;
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

int GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, std::list<double> &sync_time_list){
    while(sync_time_list.size()!=0){
        double _sync_time = sync_time_list.front();
        bool _need_interpolation = false;
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        while (UnsyncedData.back().time >= _sync_time && UnsyncedData.size() > 1) {  //imu未同步时间大于待插值时间，才能继续干啊
            //点云获取时间前没有gnss数据，雷达数据该抛掉了
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

        GNSSData front_data = UnsyncedData.at(0);
        GNSSData back_data = UnsyncedData.at(1);
        GNSSData synced_data;
        SyncedData.push_back(UnsyncedData.front());
        UnsyncedData.pop_front();

        double front_scale = (back_data.time - _sync_time) / (back_data.time - front_data.time);
        double back_scale = (_sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = _sync_time;
        synced_data.status = back_data.status;
        synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
        synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
        synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
        synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
        synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
        synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

        SyncedData.push_back(synced_data);
        break;
    }
    return 0;
}

}