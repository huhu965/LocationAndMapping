/*
 * @Author: Hu Ziwei 
 * @Description:  imu预积分
 * @Date: 2021-12-22 16:52:58 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-28 13:42:24
 */

#ifndef LOAM_FRAME_DATA_PRETREAT_IMU_PREINTEGRATE_HPP_
#define LOAM_FRAME_DATA_PRETREAT_IMU_PREINTEGRATE_HPP_

#include <vector>
#include <utility>
#include <mutex>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv4/opencv2/core/core.hpp>
namespace loam_frame{
    
#ifndef GRAVITY_VALUE
#define GRAVITY_VALUE 9.81
#endif
//静态偏差
class ImuBias
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & bax;
        ar & bay;
        ar & baz;

        ar & bwx;
        ar & bwy;
        ar & bwz;
    }

public:
    ImuBias():bax(0),bay(0),baz(0),bwx(0),bwy(0),bwz(0){}
    ImuBias(const float &b_acc_x, const float &b_acc_y, const float &b_acc_z,
            const float &b_ang_vel_x, const float &b_ang_vel_y, const float &b_ang_vel_z):
            bax(b_acc_x), bay(b_acc_y), baz(b_acc_z), bwx(b_ang_vel_x), bwy(b_ang_vel_y), bwz(b_ang_vel_z){}
    void CopyFrom(ImuBias &b);
    friend std::ostream& operator<< (std::ostream &out, const ImuBias &b);

public:
    float bax, bay, baz;
    float bwx, bwy, bwz;
};

class ImuCalib
{
    template<class Archive>
    void serializeMatrix(Archive &ar, cv::Mat& mat, const unsigned int version)
    {
        int cols, rows, type;
        bool continuous;

        if (Archive::is_saving::value) {
            cols = mat.cols; rows = mat.rows; type = mat.type();
            continuous = mat.isContinuous();
        }

        ar & cols & rows & type & continuous;
        if (Archive::is_loading::value)
            mat.create(rows, cols, type);

        if (continuous) {
            const unsigned int data_size = rows * cols * mat.elemSize();
            ar & boost::serialization::make_array(mat.ptr(), data_size);
        } else {
            const unsigned int row_size = cols*mat.elemSize();
            for (int i = 0; i < rows; i++) {
                ar & boost::serialization::make_array(mat.ptr(i), row_size);
            }
        }
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        serializeMatrix(ar,Tcb,version);
        serializeMatrix(ar,Tbc,version);
        serializeMatrix(ar,Cov,version);
        serializeMatrix(ar,CovWalk,version);
    }

public:
    ImuCalib(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
    {
        Set(Tbc_,ng,na,ngw,naw);
    }
    ImuCalib(const ImuCalib &calib);
    ImuCalib(){}

    void Set(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw);

public:
    cv::Mat Tcb;
    cv::Mat Tbc;
    cv::Mat Cov, CovWalk;
};

class IntegratedRotation
{
public:
    IntegratedRotation() = default;
    IntegratedRotation(const cv::Point3f &angVel, const ImuBias &imuBias, const float &time);

public:
    float deltaT; //integration time
    cv::Mat deltaR; //integrated rotation
    cv::Mat rightJ; // right jacobian
};

class ImuPreintegrate{

template<class Archive>
void serializeMatrix(Archive &ar, cv::Mat& mat, const unsigned int version){
    int cols, rows, type;
    bool continuous;

    if (Archive::is_saving::value) {
        cols = mat.cols; rows = mat.rows; type = mat.type();
        continuous = mat.isContinuous();
    }

    ar & cols & rows & type & continuous;
    if (Archive::is_loading::value)
        mat.create(rows, cols, type);

    if (continuous) {
        const unsigned int data_size = rows * cols * mat.elemSize();
        ar & boost::serialization::make_array(mat.ptr(), data_size);
    } else {
        const unsigned int row_size = cols*mat.elemSize();
        for (int i = 0; i < rows; i++) {
            ar & boost::serialization::make_array(mat.ptr(i), row_size);
        }
    }
}

friend class boost::serialization::access;
template<class Archive>
void serialize(Archive & ar, const unsigned int version){
    ar & dT;
    serializeMatrix(ar,C,version);
    serializeMatrix(ar,Info,version);
    serializeMatrix(ar,Nga,version);
    serializeMatrix(ar,NgaWalk,version);
    ar & b;
    serializeMatrix(ar,dR,version);
    serializeMatrix(ar,dV,version);
    serializeMatrix(ar,dP,version);
    serializeMatrix(ar,JRg,version);
    serializeMatrix(ar,JVg,version);
    serializeMatrix(ar,JVa,version);
    serializeMatrix(ar,JPg,version);
    serializeMatrix(ar,JPa,version);
    serializeMatrix(ar,avgA,version);
    serializeMatrix(ar,avgW,version);

    ar & bu;
    serializeMatrix(ar,db,version);
    ar & mvMeasurements;
}

public:
    ImuPreintegrate(const ImuBias &b_, const ImuCalib &calib);
    ImuPreintegrate(ImuPreintegrate* pImuPre);
    ImuPreintegrate() = default;
    ~ImuPreintegrate() = default;
    void CopyFrom(ImuPreintegrate* pImuPre);
    void Initialize(const ImuBias &b_, const ImuCalib &calib);
    void Initialize(const ImuBias &b_);
    void IntegrateNewMeasurement(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &dt);
    void Reintegrate();
    void MergePrevious(ImuPreintegrate* pPrev);
    void SetNewBias(const ImuBias &bu_);
    ImuBias GetDeltaBias(const ImuBias &b_);
    cv::Mat GetDeltaRotation(const ImuBias &b_);
    cv::Mat GetDeltaVelocity(const ImuBias &b_);
    cv::Mat GetDeltaPosition(const ImuBias &b_);
    cv::Mat GetUpdatedDeltaRotation();
    cv::Mat GetUpdatedDeltaVelocity();
    cv::Mat GetUpdatedDeltaPosition();
    cv::Mat GetOriginalDeltaRotation();
    cv::Mat GetOriginalDeltaVelocity();
    cv::Mat GetOriginalDeltaPosition();
    Eigen::Matrix<double,15,15> GetInformationMatrix();
    cv::Mat GetDeltaBias();
    ImuBias GetOriginalBias();
    ImuBias GetUpdatedBias();
public:
    float dT;
    cv::Mat C;
    cv::Mat Info;
    cv::Mat Nga, NgaWalk;

    // Values for the original bias (when integration was computed)
    ImuBias b;
    cv::Mat dR, dV, dP; //这段时间内的旋转，速度变化，位置变化量
    cv::Mat JRg, JVg, JVa, JPg, JPa;
    cv::Mat avgA;
    cv::Mat avgW;

private:
    // Updated bias
    ImuBias bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    cv::Mat db;

    struct integrable
    {
        integrable(const cv::Point3f &a_, const cv::Point3f &w_ , const float &t_):a(a_),w(w_),t(t_){}
        cv::Point3f a;
        cv::Point3f w;
        float t;
    };

    std::vector<integrable> mvMeasurements;
    std::mutex mMutex;

};

// Lie Algebra Functions
cv::Mat ExpSO3(const float &x, const float &y, const float &z);
Eigen::Matrix<double,3,3> ExpSO3(const double &x, const double &y, const double &z);
cv::Mat ExpSO3(const cv::Mat &v);
cv::Mat LogSO3(const cv::Mat &R);
cv::Mat RightJacobianSO3(const float &x, const float &y, const float &z);
cv::Mat RightJacobianSO3(const cv::Mat &v);
cv::Mat InverseRightJacobianSO3(const float &x, const float &y, const float &z);
cv::Mat InverseRightJacobianSO3(const cv::Mat &v);
cv::Mat Skew(const cv::Mat &v);
cv::Mat NormalizeRotation(const cv::Mat &R);

}
#endif 