/*
 * @Author: Hu Ziwei 
 * @Date: 2021-10-12 16:27:35 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-23 15:58:32
 */

#include "modules/map/submap_process.hpp"

namespace loam_frame{
SubmapProcess::SubmapProcess(ros::NodeHandle& nh):MapVariable(nh){}

PointCloudData::point_cloud SubmapProcess::ExtractSurroundKeyFramesPose(double _currentTime){
    return ExtractSurroundKeyFramesPose(translationCloudKeyFrames->back(), _currentTime);
}

PointCloudData::point_cloud SubmapProcess::TransformToWorld(const PointCloudData::point_cloud_ptr _cloud_in, Eigen::Isometry3d &_pose){
    PointCloudData::point_cloud _cloud_out;
    for (int i = 0; i < _cloud_in->points.size(); i++){
        PointCloudData::point _point_out;
        Eigen::Vector3d _point(_cloud_in->points[i].x, _cloud_in->points[i].y, _cloud_in->points[i].z);
        Eigen::Vector3d _un_point = _pose * _point;
        _point_out.x = _un_point.x();
        _point_out.y = _un_point.y();
        _point_out.z = _un_point.z();
        _cloud_out.push_back(_point_out);
    }
    return _cloud_out;
}

void SubmapProcess::ClearSurroundCloud(){
    currentTranslationFromSubmapSet.clear();
    laserCornerPointsFromSubmapDSPtr->clear();
    laserSurfPointsFromSubmapDSPtr->clear();
}

PointCloudData::point_cloud SubmapProcess::ExtractSurroundKeyFramesPose(const PointCloudData::point &_currentKeyFrameTranslation, double _currentTime){
    
    PointCloudData::point_cloud_ptr _surroundingKeyPoses(new PointCloudData::point_cloud());
    PointCloudData::point_cloud_ptr _surroundingKeyPosesDS(new PointCloudData::point_cloud());
    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis; //距离平方
    hasLoop = false; //默认没检测到回环检测
    loopSubmapIndex.clear();

    currentLaserCloudTime = _currentTime;
    // kdtree的输入，全局关键帧位姿集合（历史所有关键帧集合）
    kdtreeSurroundingKeyPosesPtr_->setInputCloud(translationCloudKeyFrames); 
    // std::cout<<"kdtree建立通过"<<std::endl;
    // 对最近的一帧关键帧，在半径区域内搜索空间区域上相邻的关键帧集合
    kdtreeSurroundingKeyPosesPtr_->radiusSearch(_currentKeyFrameTranslation, 
                                                surroundingKeyframeSearchRadius_, 
                                                _pointSearchInd, 
                                                _pointSearchSqDis);
    // std::cout<<"kdtree查找通过"<<std::endl;
    // 遍历搜索结果，pointSearchInd存的是结果在cloudKeyFrameTranslation下面的索引
    for (int i = 0; i < (int)_pointSearchInd.size(); ++i)
    {
        int id = _pointSearchInd[i];
        //回环检测
        if (currentLaserCloudTime- poseCloudKeyFrames[id].time > 30.0){
            loopSubmapIndex.push_back(poseCloudKeyFrames[id].index);
        }
        else{
            //加入相邻关键帧位姿集合中
            _surroundingKeyPoses->push_back(translationCloudKeyFrames->points[id]);
        }
    }
    if((int)loopSubmapIndex.size() > 2){ //检测到回环了
        hasLoop = true;
    }

    // 加入时间上相邻的一些关键帧，比如当载体在原地转圈，这些帧加进来是合理的
    int numPoses = translationCloudKeyFrames->size();
    for (int i = numPoses-1; i >= 0; --i)
    {
        if (currentLaserCloudTime- poseCloudKeyFrames[i].time < 5.0)
            _surroundingKeyPoses->push_back(translationCloudKeyFrames->points[i]);
        else
            break;
    }

    // 降采样一下
    downSizeFilterSurroundingKeyPoses_.setInputCloud(_surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses_.filter(*_surroundingKeyPosesDS);

    return *_surroundingKeyPosesDS;
}

void SubmapProcess::ExtractSurroundCloud(const PointCloudData::point_cloud &_surroundKeyFramePoses){
    laserCornerPointsFromSubmapPtr->clear();
    laserSurfPointsFromSubmapPtr->clear();

    // 太大了，清空一下内存
    if ( (int)currentTranslationFromSubmapSet.size() > (2*(int)_surroundKeyFramePoses.size()) ){
        ClearSurroundCloud();
    }
    // ClearSurroundCloud();

    // 遍历当前帧（实际是取最近的一个关键帧来找它相邻的关键帧集合）时空维度上相邻的关键帧集合
    for (int i = 0; i < (int)_surroundKeyFramePoses.size(); ++i)
    {
        // 相邻关键帧索引
        int thisKeyInd = (int)_surroundKeyFramePoses.points[i].intensity;
        if(currentTranslationFromSubmapSet.find(thisKeyInd) != currentTranslationFromSubmapSet.end()){
            //如果子地图中有该点，就不管了
            continue;
        }

        if ( thisKeyInd < translationCloudKeyFrames->size())//如果指针关键帧小于尺寸
        {
            currentTranslationFromSubmapSet.insert(thisKeyInd);
            *laserCornerPointsFromSubmapPtr += this->TransformToWorld(cornerCloudKeyFrames[thisKeyInd], 
                                                                    poseCloudKeyFrames[thisKeyInd].pose);
            *laserSurfPointsFromSubmapPtr   += this->TransformToWorld(surfCloudKeyFrames[thisKeyInd], 
                                                                    poseCloudKeyFrames[thisKeyInd].pose);
        }
        
    }
    //子地图新增的点加上之前子地图的点，作为没有降采样之前的点
    *laserCornerPointsFromSubmapPtr += *laserCornerPointsFromSubmapDSPtr;
    *laserSurfPointsFromSubmapPtr += *laserSurfPointsFromSubmapDSPtr;

    // 降采样局部角点map
    downSizeFilterCorner_.setInputCloud(laserCornerPointsFromSubmapPtr);
    downSizeFilterCorner_.filter(*laserCornerPointsFromSubmapDSPtr);
    // 降采样局部平面点map
    downSizeFilterSurface_.setInputCloud(laserSurfPointsFromSubmapPtr);
    downSizeFilterSurface_.filter(*laserSurfPointsFromSubmapDSPtr);
}

void SubmapProcess::ExtractLoopSurroundKeyFramesPose(const PointCloudData::point &_loopKeyFrameTranslation, double _currentTime){
    
    PointCloudData::point_cloud_ptr _surroundingKeyPoses(new PointCloudData::point_cloud());
    PointCloudData::point_cloud_ptr _surroundingKeyPosesDS(new PointCloudData::point_cloud());
    std::vector<int> _pointSearchInd;
    std::vector<float> _pointSearchSqDis; //距离平方
    loopSubmapIndex.clear();

    // kdtree的输入，全局关键帧位姿集合（历史所有关键帧集合）
    kdtreeSurroundingKeyPosesPtr_->setInputCloud(translationCloudKeyFrames); 
    // std::cout<<"kdtree建立通过"<<std::endl;
    // 对最近的一帧关键帧，在半径区域内搜索空间区域上相邻的关键帧集合
    kdtreeSurroundingKeyPosesPtr_->radiusSearch(_loopKeyFrameTranslation, 
                                                historyKeyFrameSearchRadius_,
                                                _pointSearchInd, 
                                                _pointSearchSqDis);
    // std::cout<<"kdtree查找通过"<<std::endl;
    // 遍历搜索结果，pointSearchInd存的是结果在cloudKeyFrameTranslation下面的索引
    for (int i = 0; i < (int)_pointSearchInd.size(); ++i)
    {
        int id = _pointSearchInd[i];
        //回环检测
        if (_currentTime- poseCloudKeyFrames[id].time > 30.0){
            loopSubmapIndex.push_back(poseCloudKeyFrames[id].index);
        }
        if(loopSubmapIndex.size() > historyKeyFrameFearchNum_){
            break;
        }
    }
}

void SubmapProcess::ExtractLoopSurroundCloud(std::vector<int> &_surroundLoopFramePoses){
    laserCornerPointsFromLoopSubmapPtr->clear();
    laserSurfPointsFromLoopSubmapPtr->clear();

    // 时空维度上相邻的关键帧集合
    for (int i = 0; i < (int)_surroundLoopFramePoses.size(); ++i)
    {
        // 相邻关键帧索引
        int thisKeyInd = (int)_surroundLoopFramePoses[i];
        //向子地图中填加点云
        *laserCornerPointsFromLoopSubmapPtr += this->TransformToWorld(cornerCloudKeyFrames[thisKeyInd], 
                                                                poseCloudKeyFrames[thisKeyInd].pose);
        *laserSurfPointsFromLoopSubmapPtr   += this->TransformToWorld(surfCloudKeyFrames[thisKeyInd], 
                                                                poseCloudKeyFrames[thisKeyInd].pose); 
    }

    // 降采样局部角点map
    downSizeFilterCorner_.setInputCloud(laserCornerPointsFromLoopSubmapPtr);
    downSizeFilterCorner_.filter(*laserCornerPointsFromLoopSubmapDSPtr);
    // 降采样局部平面点map
    downSizeFilterSurface_.setInputCloud(laserSurfPointsFromLoopSubmapPtr);
    downSizeFilterSurface_.filter(*laserSurfPointsFromLoopSubmapDSPtr);

    //返回最近的回环位姿用于回环位姿的判断
    // return poseCloudKeyFrames[(int)_surroundLoopFramePoses[0]];
}

    
}