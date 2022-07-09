/*
 * @Author: Hu Ziwei 
 * @Description:  
 * @Date: 2021-11-15 11:15:35 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-11-15 11:28:21
 */

#ifndef LOAM_FRAME_MODULES_OPTIMIZATION_G2O_G2O_GRAPH_OPTIMIZATION_HPP_
#define LOAM_FRAME_MODULES_OPTIMIZATION_G2O_G2O_GRAPH_OPTIMIZATION_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>
#include <deque>

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "modules/optimization/g2o/edge/edge_se3_priorquat.hpp"
#include "modules/optimization/g2o/edge/edge_se3_priorxyz.hpp"

namespace g2o{
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
} // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace loam_frame {
class G2oGraphOptimizer {
  public:
    G2oGraphOptimizer(const std::string &solver_type = "lm_var");
    // 优化
    bool Optimize();
    // 输出数据
    bool GetOptimizedPose(std::deque<Eigen::Isometry3d>& optimized_pose);
    int GetNodeNum();
    // 添加节点、边、鲁棒核
    void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size);
    /*
    * @Description:添加节点，need_fix为true则设为固定节点，不会被优化
    */
    void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix);
    void AddSe3Edge(int vertex_index1,
                    int vertex_index2,
                    const Eigen::Isometry3d &relative_pose,
                    const Eigen::VectorXd noise);
    void AddSe3PriorXYZEdge(int se3_vertex_index,
                            const Eigen::Vector3d &xyz,
                            Eigen::VectorXd noise);
    void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                   const Eigen::Quaterniond &quat,
                                   Eigen::VectorXd noise);

  private:
    Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
    Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);
    Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
    void AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size);

  private:
    g2o::RobustKernelFactory *robust_kernel_factory_;
    std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

    std::string robust_kernel_name_;
    double robust_kernel_size_;
    bool need_robust_kernel_ = false;

    int max_iterations_num_ = 512;
};
} // namespace lidar_localization
#endif