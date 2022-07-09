/*
 * @Author: Hu Ziwei 
 * @Description:地图的使用接口
 * @Date: 2021-11-04 14:45:37 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-27 14:36:50
 */
#ifndef LOAM_FRAME_MODULES_MAP_MAP_HANDLE_INTERFACE_HPP_
#define LOAM_FRAME_MODULES_MAP_MAP_HANDLE_INTERFACE_HPP_

#include "modules/map/map_variable.hpp"
#include "modules/map/submap_process.hpp"
#include "modules/map/map_process.hpp"

namespace loam_frame{

class MapHandleInterface:public MapProcess, public SubmapProcess{
public:
    MapHandleInterface(ros::NodeHandle& nh):MapProcess(nh), SubmapProcess(nh), MapVariable(nh){}
};

}
#endif //LOAM_FRAME_MODULES_MAP_MAP_HANDLE_INTERFACE_HPP_