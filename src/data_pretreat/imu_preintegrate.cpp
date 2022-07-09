/*
 * @Author: Hu Ziwei 
 * @Description:  imu预积分，高射炮打蚊子，用不上
 * @Date: 2021-12-23 10:01:56 
 * @Last Modified by: Hu Ziwei
 * @Last Modified time: 2021-12-30 18:07:53
 */
#include <iostream>

#include "data_pretreat/imu_preintegrate.hpp"

namespace loam_frame{
const float eps = 1e-4;

//归一化旋转矩阵
cv::Mat NormalizeRotation(const cv::Mat &R)
{
    cv::Mat U,w,Vt; //求得的V是转置
    cv::SVDecomp(R,w,U,Vt,cv::SVD::FULL_UV);//后面的FULL_UV表示把U和VT补充称单位正交方阵
    return U*Vt;
}

//获取向量的反对称矩阵
cv::Mat Skew(const cv::Mat &v)
{
    const float x = v.at<float>(0);
    const float y = v.at<float>(1);
    const float z = v.at<float>(2);
    return (cv::Mat_<float>(3,3) << 0, -z, y,
            z, 0, -x,
            -y,  x, 0);
}
//罗德里格斯公式，由李代数转到李群，旋转向量转为旋转矩阵，角度用弧度表示
cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

Eigen::Matrix<double,3,3> ExpSO3(const double &x, const double &y, const double &z)
{
    Eigen::Matrix<double,3,3> I = Eigen::MatrixXd::Identity(3,3);
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix<double,3,3> W;
    W(0,0) = 0;
    W(0,1) = -z;
    W(0,2) = y;
    W(1,0) = z;
    W(1,1) = 0;
    W(1,2) = -x;
    W(2,0) = -y;
    W(2,1) = x;
    W(2,2) = 0;

    if(d<eps)
        return (I + W + 0.5*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

//李群转李代数
//利用迹求旋转角
//然后利用sin（旋转角度）a^ = (R-Rt)/2
cv::Mat LogSO3(const cv::Mat &R)
{
    const float tr = R.at<float>(0,0)+R.at<float>(1,1)+R.at<float>(2,2);//求迹
    cv::Mat w = (cv::Mat_<float>(3,1) <<(R.at<float>(2,1)-R.at<float>(1,2))/2,
                                        (R.at<float>(0,2)-R.at<float>(2,0))/2,
                                        (R.at<float>(1,0)-R.at<float>(0,1))/2);
    const float costheta = (tr-1.0f)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const float theta = acos(costheta);
    const float s = sin(theta);
    if(fabs(s)<eps)
        return w;
    else
        return theta*w/s;
}
//右乘雅克比矩阵
cv::Mat RightJacobianSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        return cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

cv::Mat RightJacobianSO3(const cv::Mat &v)
{
    return RightJacobianSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

//右乘雅克比的逆矩阵
cv::Mat InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        return cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

cv::Mat InverseRightJacobianSO3(const cv::Mat &v)
{
    return InverseRightJacobianSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

void ImuBias::CopyFrom(ImuBias &b)
{
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

std::ostream& operator<< (std::ostream &out, const ImuBias &b)
{
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

void ImuCalib::Set(const cv::Mat &Tbc_, const float &ng, const float &na, const float &ngw, const float &naw)
{
    Tbc = Tbc_.clone();
    Tcb = cv::Mat::eye(4,4,CV_32F);
    Tcb.rowRange(0,3).colRange(0,3) = Tbc.rowRange(0,3).colRange(0,3).t();
    Tcb.rowRange(0,3).col(3) = -Tbc.rowRange(0,3).colRange(0,3).t()*Tbc.rowRange(0,3).col(3);
    Cov = cv::Mat::eye(6,6,CV_32F);
    const float ng2 = ng*ng;
    const float na2 = na*na;
    Cov.at<float>(0,0) = ng2;
    Cov.at<float>(1,1) = ng2;
    Cov.at<float>(2,2) = ng2;
    Cov.at<float>(3,3) = na2;
    Cov.at<float>(4,4) = na2;
    Cov.at<float>(5,5) = na2;
    CovWalk = cv::Mat::eye(6,6,CV_32F);
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;
    CovWalk.at<float>(0,0) = ngw2;
    CovWalk.at<float>(1,1) = ngw2;
    CovWalk.at<float>(2,2) = ngw2;
    CovWalk.at<float>(3,3) = naw2;
    CovWalk.at<float>(4,4) = naw2;
    CovWalk.at<float>(5,5) = naw2;
}

ImuCalib::ImuCalib(const ImuCalib &calib)
{
    Tbc = calib.Tbc.clone();
    Tcb = calib.Tcb.clone();
    Cov = calib.Cov.clone();
    CovWalk = calib.CovWalk.clone();
}

IntegratedRotation::IntegratedRotation(const cv::Point3f &angVel, const ImuBias &imuBias, const float &time):deltaT(time)
{
    const float x = (angVel.x-imuBias.bwx)*time;
    const float y = (angVel.y-imuBias.bwy)*time;
    const float z = (angVel.z-imuBias.bwz)*time;

    cv::Mat I = cv::Mat::eye(3,3,CV_32F);

    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
    {
        deltaR = I + W;
        rightJ = cv::Mat::eye(3,3,CV_32F);
    }
    else
    {
        deltaR = I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2; //李代数转李群，计算旋转变化量的矩阵，
        rightJ = I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d); //计算旋转变化量的右乘雅克比
    }
}

ImuPreintegrate::ImuPreintegrate(const ImuBias &b_, const ImuCalib &calib)
{
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
}

// Copy constructor
ImuPreintegrate::ImuPreintegrate(ImuPreintegrate* pImuPre): dT(pImuPre->dT), C(pImuPre->C.clone()), Info(pImuPre->Info.clone()),
    Nga(pImuPre->Nga.clone()), NgaWalk(pImuPre->NgaWalk.clone()), b(pImuPre->b), dR(pImuPre->dR.clone()), dV(pImuPre->dV.clone()),
    dP(pImuPre->dP.clone()), JRg(pImuPre->JRg.clone()), JVg(pImuPre->JVg.clone()), JVa(pImuPre->JVa.clone()), JPg(pImuPre->JPg.clone()),
    JPa(pImuPre->JPa.clone()), avgA(pImuPre->avgA.clone()), avgW(pImuPre->avgW.clone()), bu(pImuPre->bu), db(pImuPre->db.clone()), mvMeasurements(pImuPre->mvMeasurements)
{
    
}

void ImuPreintegrate::CopyFrom(ImuPreintegrate* pImuPre)
{
    std::cout << "ImuPreintegrate: start clone" << std::endl;
    dT = pImuPre->dT;
    C = pImuPre->C.clone();
    Info = pImuPre->Info.clone();
    Nga = pImuPre->Nga.clone();
    NgaWalk = pImuPre->NgaWalk.clone();
    std::cout << "ImuPreintegrate: first clone" << std::endl;
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR.clone();
    dV = pImuPre->dV.clone();
    dP = pImuPre->dP.clone();
    JRg = pImuPre->JRg.clone();
    JVg = pImuPre->JVg.clone();
    JVa = pImuPre->JVa.clone();
    JPg = pImuPre->JPg.clone();
    JPa = pImuPre->JPa.clone();
    avgA = pImuPre->avgA.clone();
    avgW = pImuPre->avgW.clone();
    std::cout << "ImuPreintegrate: second clone" << std::endl;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db.clone();
    std::cout << "ImuPreintegrate: third clone" << std::endl;
    mvMeasurements = pImuPre->mvMeasurements;
    std::cout << "ImuPreintegrate: end clone" << std::endl;
}

void ImuPreintegrate::Initialize(const ImuBias &b_, const ImuCalib &calib){
    Nga = calib.Cov.clone();
    NgaWalk = calib.CovWalk.clone();
    Initialize(b_);
}

void ImuPreintegrate::Initialize(const ImuBias &b_)
{
    dR = cv::Mat::eye(3,3,CV_32F);
    dV = cv::Mat::zeros(3,1,CV_32F);
    dP = cv::Mat::zeros(3,1,CV_32F);
    JRg = cv::Mat::zeros(3,3,CV_32F);
    JVg = cv::Mat::zeros(3,3,CV_32F);
    JVa = cv::Mat::zeros(3,3,CV_32F);
    JPg = cv::Mat::zeros(3,3,CV_32F);
    JPa = cv::Mat::zeros(3,3,CV_32F);
    C = cv::Mat::zeros(15,15,CV_32F);
    Info=cv::Mat();
    db = cv::Mat::zeros(6,1,CV_32F);
    b=b_;
    bu=b_;
    avgA = cv::Mat::zeros(3,1,CV_32F);
    avgW = cv::Mat::zeros(3,1,CV_32F);
    dT=0.0f;
}

void ImuPreintegrate::Reintegrate()
{
    std::unique_lock<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    Initialize(bu);
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}

//给入加速度，角速度，持续时间
void ImuPreintegrate::IntegrateNewMeasurement(const cv::Point3f &acceleration, const cv::Point3f &angVel, const float &dt)
{
    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    //Matrices to compute covariance
    // cv::Mat A = cv::Mat::eye(9,9,CV_32F);
    // cv::Mat B = cv::Mat::zeros(9,6,CV_32F);
    //去掉零偏，得到角加速度和运动加速度
    cv::Mat acc = (cv::Mat_<float>(3,1) << acceleration.x-b.bax,acceleration.y-b.bay, acceleration.z-b.baz);
    cv::Mat accW = (cv::Mat_<float>(3,1) << angVel.x-b.bwx, angVel.y-b.bwy, angVel.z-b.bwz);

    //求平均加速度
    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    //求平均角加速度
    avgW = (dT*avgW + accW*dt)/(dT+dt);

    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt; //把两次数据之间看做匀加速运动，来模拟曲线，世界坐标系下
    dV = dV + dR*acc*dt; //加速度做积分，获取当前的速度方向，世界坐标系下

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    // 加速度的反对称矩阵
    cv::Mat Wacc = (cv::Mat_<float>(3,3) << 0, -acc.at<float>(2), acc.at<float>(1),
                                                   acc.at<float>(2), 0, -acc.at<float>(0),
                                                   -acc.at<float>(1), acc.at<float>(0), 0);

    // A.rowRange(3,6).colRange(0,3) = -dR*dt*Wacc;
    // A.rowRange(6,9).colRange(0,3) = -0.5f*dR*dt*dt*Wacc;
    // A.rowRange(6,9).colRange(3,6) = cv::Mat::eye(3,3,CV_32F)*dt;
    
    // B.rowRange(3,6).colRange(3,6) = dR*dt;
    // B.rowRange(6,9).colRange(3,6) = 0.5f*dR*dt*dt;

    // Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // Update delta rotation
    //一次imu数据的旋转量
    IntegratedRotation dRi(angVel,b,dt);
    dR = NormalizeRotation(dR*dRi.deltaR);

    // // Compute rotation parts of matrices A and B
    // A.rowRange(0,3).colRange(0,3) = dRi.deltaR.t(); //矩阵的转置
    // B.rowRange(0,3).colRange(0,3) = dRi.rightJ*dt;

    // // Update covariance
    // C.rowRange(0,9).colRange(0,9) = A*C.rowRange(0,9).colRange(0,9)*A.t() + B*Nga*B.t();
    // C.rowRange(9,15).colRange(9,15) = C.rowRange(9,15).colRange(9,15) + NgaWalk;

    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.t()*JRg - dRi.rightJ*dt;

    // Total integrated time
    dT += dt;
}

void ImuPreintegrate::MergePrevious(ImuPreintegrate* pPrev)
{
    if (pPrev==this)
        return;

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    ImuBias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

void ImuPreintegrate::SetNewBias(const ImuBias &bu_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    bu = bu_;

    db.at<float>(0) = bu_.bwx-b.bwx;
    db.at<float>(1) = bu_.bwy-b.bwy;
    db.at<float>(2) = bu_.bwz-b.bwz;
    db.at<float>(3) = bu_.bax-b.bax;
    db.at<float>(4) = bu_.bay-b.bay;
    db.at<float>(5) = bu_.baz-b.baz;
}

ImuBias ImuPreintegrate::GetDeltaBias(const ImuBias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    return ImuBias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}

cv::Mat ImuPreintegrate::GetDeltaRotation(const ImuBias &b_)
{
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    // return NormalizeRotation(dR*ExpSO3(JRg*dbg));
    return NormalizeRotation(dR);
}

cv::Mat ImuPreintegrate::GetDeltaVelocity(const ImuBias &b_)
{
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    cv::Mat dba = (cv::Mat_<float>(3,1) << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz);
    return dV + JVg*dbg + JVa*dba;
}

cv::Mat ImuPreintegrate::GetDeltaPosition(const ImuBias &b_)
{
    cv::Mat dbg = (cv::Mat_<float>(3,1) << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
    cv::Mat dba = (cv::Mat_<float>(3,1) << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz);
    // return dP + JPg*dbg + JPa*dba;
    return dP;
}

cv::Mat ImuPreintegrate::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return NormalizeRotation(dR*ExpSO3(JRg*db.rowRange(0,3)));
}

cv::Mat ImuPreintegrate::GetUpdatedDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV + JVg*db.rowRange(0,3) + JVa*db.rowRange(3,6);
}

cv::Mat ImuPreintegrate::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP + JPg*db.rowRange(0,3) + JPa*db.rowRange(3,6);
}

cv::Mat ImuPreintegrate::GetOriginalDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dR.clone();
}

cv::Mat ImuPreintegrate::GetOriginalDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV.clone();
}

cv::Mat ImuPreintegrate::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP.clone();
}

ImuBias ImuPreintegrate::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return b;
}

ImuBias ImuPreintegrate::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

cv::Mat ImuPreintegrate::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db.clone();
}

Eigen::Matrix<double,15,15> ImuPreintegrate::GetInformationMatrix()
{
    std::unique_lock<std::mutex> lock(mMutex);
    if(Info.empty())
    {
        Info = cv::Mat::zeros(15,15,CV_32F);
        Info.rowRange(0,9).colRange(0,9)=C.rowRange(0,9).colRange(0,9).inv(cv::DECOMP_SVD);
        for(int i=9;i<15;i++)
            Info.at<float>(i,i)=1.0f/C.at<float>(i,i);
    }

    Eigen::Matrix<double,15,15> EI;
    for(int i=0;i<15;i++)
        for(int j=0;j<15;j++)
            EI(i,j)=Info.at<float>(i,j);
    return EI;
}
  
}