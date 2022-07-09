/*
 * @Author: Hu Ziwei 
 * @Description:gnss数据格式
 * @Date: 2021-11-08 15:53:22 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-28 12:54:09
 */
#ifndef LOAM_FRAME_SENSOR_DATA_GNSS_DATA_HPP_
#define LOAM_FRAME_SENSOR_DATA_GNSS_DATA_HPP_
#include <vector>
#include <deque>
#include <list>
#include <string>

#include "Geocentric/LocalCartesian.hpp"

using std::vector;
using std::string;

namespace loam_frame {
enum gnss_sync_status{
    gnss_sync_success = 0,
    not_gnss_data_before_cloud,
    not_gnss_data_behind_cloud
};

class GNSSData {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;
    
    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;

  private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

  public: 
    void InitOriginPosition();
    void UpdateXYZ();
    static int SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, std::list<double> &sync_time_list);
};
}

#endif //LOAM_FRAME_SENSOR_DATA_GNSS_DATA_HPP_