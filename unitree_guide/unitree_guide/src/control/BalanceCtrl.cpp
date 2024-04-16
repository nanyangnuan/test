/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "control/BalanceCtrl.h"
#include "common/mathTools.h"
#include "thirdParty/quadProgpp/Array.hh"
#include "common/timeMarker.h"
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/crba.hpp>
 
#define E3 Eigen::MatrixXd::Identity(3, 3)
#define E30 Eigen::MatrixXd::Identity(30, 30)

// BalanceCtrl::BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta)
//             : _mass(mass), _Ib(Ib), _S(S), _alpha(alpha), _beta(beta){
//     _Fprev.setZero();
//     _g << 0, 0, -9.81;
//     _fricRatio = 0.3;
//     double _fricMatMax,_fricMatMin;   //这两个数记得填；

//     _fricMatlimit  <<  0,
//                        0,
//                        0,
//                        0,
//                        0;          
// }

BalanceCtrl::BalanceCtrl(QuadrupedRobot *robModel){

    double _fricMatMax,_fricMatMin;   //这两个数记得填；
    Eigen::Matrix<double,30,1> w;
    _fricRatio = 0.3;
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
    _fricMatlimit  <<  0,
                       0,
                       0,
                       0,
                       0;  
    _alpha = 0.00001;
    taumax<<  50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50; 
    taumin<< -50,-50,-50,-50,-50,-50,-50,-50,-50,-50,-50,-50;
    hmin=0.3;
    hmax=0.5;
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
}

BalanceCtrl::BalanceCtrl(QuadrupedRobot *robModel,pinocchio::Data *data,pinocchio::Model *model){

    _data = data;
    _model = model;

    double _fricMatMax,_fricMatMin;   //这两个数记得填；
    Eigen::Matrix<double,30,1> w;
    _fricRatio = 0.3;
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
    _fricMatlimit  <<  0,
                       0,
                       0,
                       0,
                       0;  
    _alpha = 0.00001;
    taumax<<  50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50; 
    taumin<< -50,-50,-50,-50,-50,-50,-50,-50,-50,-50,-50,-50;
    hmin=0.3;
    hmax=0.5;
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
}

void BalanceCtrl::Modeldata(VecInt4 contact, Vec19 q, Vec18 u,Vec34 accel)
{ 

    long long _startTime;
  _startTime = getSystemTime();
  using namespace pinocchio; 
  Eigen::VectorXd _q(_model->nq),_v(_model->nv);
    //输入顺序互换
  _q.segment(0,7)=q.segment(0,7);
  _q.segment(7,3)=q.segment(10,3);
  _q.segment(10,3)=q.segment(7,3);
  _q.segment(13,3)=q.segment(16,3);
  _q.segment(16,3)=q.segment(13,3);
  _v.segment(0,6)=u.segment(0,6); 
  _v.segment(6,3)=u.segment(9,3);
  _v.segment(9,3)=u.segment(6,3);
  _v.segment(12,3)=u.segment(15,3);
  _v.segment(15,3)=u.segment(12,3);
  _u=_v;
  _contact(0)=contact(1);
  _contact(1)=contact(0);
  _contact(2)=contact(3);
  _contact(3)=contact(2);
  _accel.col(0)=accel.col(1);
  _accel.col(1)=accel.col(0);
  _accel.col(2)=accel.col(3);
  _accel.col(3)=accel.col(2);
  Eigen::Matrix<double,6,18> _J;
  Eigen::Matrix<double,6,18> _dJ;
  contactLegNum = 0;
    for(int i(0); i<4; ++i){
        if(_contact(i) == 1){
            contactLegNum += 1;
        }
    }
  Js.resize(contactLegNum*3,18);
  dJs.resize(contactLegNum*3,18);
  Js.setZero();
  dJs.setZero();
  _J.setZero();
  _dJ.setZero();
  J.setZero();
  pinocchio::forwardKinematics(*_model,*_data,_q,_v);
  pinocchio::computeJointJacobians(*_model, *_data);
  pinocchio::updateFramePlacements(*_model, *_data);
  pinocchio::nonLinearEffects(*_model, *_data,_q,_v);
  pinocchio::crba(*_model, *_data, _q);      //计算质量矩阵的,算出来的储存在data.M
  pinocchio::computeJointJacobiansTimeVariation(*_model, *_data,_q,_v);
  int j=0;
  VecInt4 joint_id;
  joint_id(0)=_model->getFrameId("FL_foot");
  joint_id(1)=_model->getFrameId("FR_foot");
  joint_id(2)=_model->getFrameId("RL_foot");
  joint_id(3)=_model->getFrameId("RR_foot");
  for(int i = 0; i < 4; i++)
  {
    _J.setZero();
    _dJ.setZero();
    pinocchio::getFrameJacobian(*_model, *_data, joint_id(i), LOCAL_WORLD_ALIGNED, _J);
    pinocchio::getFrameJacobianTimeVariation(*_model, *_data,joint_id(i), LOCAL_WORLD_ALIGNED,_dJ);
    J.block(3*i, 0, 3, 18) = _J.block(0,0,3,18);                             
    dJ.block(3*i, 0, 3, 18) = _dJ.block(0,0,3,18);
    if(_contact[i]){

        Js.block(3*j, 0, 3, 18) = J.block(3*i, 0, 3, 18);
        dJs.block(3*j, 0, 3, 18) = dJ.block(3*i, 0, 3, 18);
        qcontact.block(3*j, 0, 3, 1) = q.block(3*j, 0, 3, 1);
        j ++;
    }
  } 
    Eigen::MatrixXd _Js=Js.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr = _Js.householderQr();
    Eigen::MatrixXd _Q = qr.householderQ();
    Eigen::MatrixXd _R = qr.matrixQR().triangularView<Eigen::Upper>();
    Qc.resize(18,3*contactLegNum);
    Qu.resize(18,18-3*contactLegNum);
    Qc.setZero();
    Qu.setZero();
    Qc=_Q.block(0,0,18,3*contactLegNum);
    Qu=_Q.block(0,3*contactLegNum,18,18-3*contactLegNum);
    R.resize(3*contactLegNum,3*contactLegNum);
    R=_R.block(0,0,3*contactLegNum,3*contactLegNum);

  h=_data->nle;
  _M=_data->M;
  S.setZero();
  S.block(0,6,12,12)=I12;
  MatM.block(0,0,18,18)=_M;
  MatM.block(0,18,18,12)=-S.transpose();

  

}

Vec12 BalanceCtrl::calF(VecInt4 contact,Vec19 q,Vec18 u,Vec3 _ddPcd,Vec3 _dWbd,Vec34 accel,Mat3 RotCToI)
{      
    Modeldata(contact,q,u,accel);
    // 初始化_C,_d矩阵
    _C=E30;
    _w.setZero();

    task0(_u,_contact);
    task1(_ddPcd,_dWbd,RotCToI,q,_u,_contact);
    task2(_contact);
    Vec12 tau;
    tau.segment(0,3)=_w.segment(21,3);
    tau.segment(3,3)=_w.segment(18,3);
    tau.segment(6,3)=_w.segment(27,3);
    tau.segment(9,3)=_w.segment(24,3);
    return tau;
}

void BalanceCtrl::task0(Vec18 u,VecInt4 contact)
{   Eigen::MatrixXd a, b, d, f;
    EquationOfMotion();
    a=A;b=B;

    NoContactMotion(u);
    a=concatenateMatrices(a,A);b=concatenateMatrices(b,B);

    TorqueLimit();
    d=D;f=F;

    FrictionCone(contact);
    d=concatenateMatrices(d,D);f=concatenateMatrices(f,F);

    solution(a,b,d,f);

}


void BalanceCtrl::task1(Vec3 _ddPcd,Vec3 _dWbd,Mat3 RotCToI,Vec19 q,Vec18 u,VecInt4 contact)
{   Eigen::FullPivLU<MatX> lu(Aisfull);
    if (lu.rank() >= Aisfull.cols())// if A_bar full rank
    {
        return; 
    }
    _C = _C * Aisfull.fullPivLu().kernel();

    Eigen::MatrixXd a, b, d, f;
    DesiredTorsoPosition(_ddPcd,RotCToI,u);
    a=A;b=B;

    SwingFootMotionTracking(u,contact);
    a=concatenateMatrices(a,A);b=concatenateMatrices(b,B);

    DesiredTorsoYaw(_dWbd);
    a=concatenateMatrices(a,A);b=concatenateMatrices(b,B);

    DesiredTorsoRoll_Pitch(_dWbd);
    a=concatenateMatrices(a,A);b=concatenateMatrices(b,B);

    // FrictionCone(contact);
    // // a=concatenateMatrices(a,A);b=concatenateMatrices(b,B);
    // d=concatenateMatrices(d,D);f=concatenateMatrices(f,F);

    // Limbkinematiclimits(q,u);
    // d=D;f=F;

    solution(a,b,d,f);
}

void BalanceCtrl::task2(VecInt4 contact)
{   
    Eigen::FullPivLU<MatX> lu(Aisfull);
    if (lu.rank() >= Aisfull.cols())// if A_bar full rank
    {
        return; 
    }
    _C = _C * Aisfull.fullPivLu().kernel();

    Eigen::MatrixXd a, b, d, f;

    ContactForceMin();
    a=A;b=B;
    solution(a,b,d,f);

}

Eigen::MatrixXd BalanceCtrl::concatenateMatrices(Eigen::MatrixXd m1,Eigen::MatrixXd m2)
{
    if (m1.cols() <= 0) {
      return m2;
    } else if (m2.cols() <= 0) {
      return m1;
    }
    assert(m1.cols() == m2.cols());
    Eigen::MatrixXd res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;
    return res;
}

void BalanceCtrl::EquationOfMotion()
{
    A.resize(18-3*contactLegNum, 30);
    B.resize(18-3*contactLegNum, 1);
    A.setZero();
    B.setZero();
    A=Qu.transpose()*MatM;
    B=-Qu.transpose()*h;
}

void BalanceCtrl::NoContactMotion(Vec18 u){   //反正雅可比矩阵和u都在一个坐标系下，不怕
    A.resize(3*contactLegNum, 30);
    B.resize(3*contactLegNum, 1);
    A.setZero();
    B.setZero();
    A.block(0,0,3*contactLegNum,18)=Js;
    B=-dJs*u;
}

void BalanceCtrl::TorqueLimit()  
{
    D.resize(24, 30);
    F.resize(24);
    D.setZero();
    F.setZero();
    D.block(0,18,12,12)=-I12;
    D.block(12,18,12,12)=I12;
    F.segment(0,12)=taumax;
    F.segment(12,12)=taumax;
}

void BalanceCtrl::FrictionCone(VecInt4 contact)  
{
    D.resize(5*contactLegNum, 30);
    F.resize(5*contactLegNum);
    D.setZero();
    F.setZero();
    Eigen::MatrixXd fricMat;
    fricMat.resize(5*contactLegNum,3*contactLegNum);
    fricMat.setZero();
    int j=0;
    for(int i=0;i<contactLegNum;i++){       
           fricMat.block(5*i,3*i,5,3)=_fricMat;
    }
    Eigen::MatrixXd FRQc;
    FRQc=fricMat*pseudoInverse(R)*(Qc.transpose());
    D=FRQc*MatM;
    F=FRQc*h;

}


void BalanceCtrl::DesiredTorsoPosition(Vec3 _ddPcd,Mat3 RotCToI,Vec18 u)
{
    A.resize(3, 30);
    B.resize(3, 1);
    A.setZero();
    B.setZero();
    A.block(0,0,3,3)=E3;
    B=_ddPcd;             
}

void BalanceCtrl::DesiredTorsoYaw(Vec3 _dWbd)
{
    A.resize(1, 30);
    B.resize(1, 1);
    A.setZero();
    B.setZero();
    A(5)=1;
    B(0)=_dWbd(2);             
}

void BalanceCtrl::DesiredTorsoRoll_Pitch(Vec3 _dWbd)
{
    A.resize(2, 30);
    B.resize(2, 1);
    A.setZero();
    B.setZero();
    A.block(0,3,2,2)=Eigen::MatrixXd::Identity(2, 2);
    B=_dWbd.segment(0,2);             
}

void BalanceCtrl::Limbkinematiclimits(Vec19 q,Vec18 u)//这个我感觉论文还得再看看，我感觉我没理解透
{   Vec12 w_h;
    w_h<< 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1;
    Eigen::MatrixXd hh,hz;
    double h_error,hmax,hmin;
    hz=w_h.transpose()*qcontact/4;
    h_error=q(2)-hz(0);
    double dt=0.002;
    D.resize(2, 30);
    F.resize(2);
    D.setZero();
    F.setZero();
    D.block(0,0,1,18)=-0.000002*J.block(5,0,1,18);
    D.block(1,0,1,18)=0.000002*J.block(5,0,1,18); 
    hh=(J.block(5,0,1,18)+0.5*dt*dJ.block(5,0,1,18))*u;  //这儿怎么写啊
    F(0)=hmax-h_error-dt*hh(0);
    F(1)=h_error-hmin+dt*hh(0);
}



void BalanceCtrl::ContactForceMin()
{
    A.resize(3*contactLegNum, 30); 
    B.resize(3*contactLegNum, 1);
    A.setZero(); 
    B.setZero();
    A=pseudoInverse(R)*Qc.transpose()*MatM;//M是惯性矩阵
    B=pseudoInverse(R)*Qc.transpose()*h;
}

void BalanceCtrl::SwingFootMotionTracking(Vec18 u,VecInt4 contact)
{
    A.resize(12-3*contactLegNum, 30);
    B.resize(12-3*contactLegNum, 1);
    A.setZero();
    B.setZero();
    int j=0;
    for (int i = 0; i < 4; i++) {
    if (!contact(i)) {
      A.block(3 * j, 0, 3, 18) = J.block(3 * i, 0, 3, 18);
      B.block(3 * j,0, 3,1) = _accel.col(i) - dJ.block(3 * i, 0, 3, 18) * u;
      j++;
    }
  }
}

// void BalanceCtrl::MainBodyLimit()
// {   Vec12 w_h;
//     w_h<< 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1;
//     Eigen::MatrixXd hh,hz;
//     double h_error,hmax,hmin;
//     hz=w_h.transpose()*qcontact/4;
//     A.resize(30, 30);
//     A=E30;
//     B.resize(30, 1);
//     B.setZero();
//     A=A*_C.transpose();
//     B=B-A*_d;
//     _G.setZero();
//     _g0T.setZero();
//     _G = 2*A.transpose()*A+_alpha*_W;
//     _g0T = -2*B.transpose()*A;
//     _CI.resize(2, 30);
//     _ci0.resize(2);
//     _CI.setZero();
//     _ci0.setZero();
//     _CI.block(0,2,2,1)<<-1,
//                          1;
//     _ci0(0)=hmax+hz(0);
//     _ci0(0)=-hmin+hz(0);
//     solveQP();
//     d=_F;
//     _d=_d+_C*d;
//     BalanceCtrl::nullspace(_A);
//     _C=null_space;

// }


Eigen::MatrixXd BalanceCtrl::pseudoInverse(const Eigen::MatrixXd &A, double epsilon) const{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double tolerance = epsilon * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs().maxCoeff();
    
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(A.cols(), A.rows());
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tolerance) {
            S_inv(i, i) = 1.0 / svd.singularValues()(i);
        }
    }
    
    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}

void BalanceCtrl::solution(Eigen::MatrixXd a,Eigen::MatrixXd b,Eigen::MatrixXd d,Eigen::MatrixXd f)  
{   Eigen::MatrixXd _A,_B,_D,_F;
    int numDecisionVars=_C.cols();
    _A=a*_C;
    _B=b-a*_w;
    if(d.rows()>0){
     _D=d*_C;
    _F=d*_w+f;
    }else{
        _D=Eigen::MatrixXd::Zero(1, numDecisionVars);
        _F=Eigen::MatrixXd::Zero(1,1);
    }
    _G.resize(numDecisionVars,numDecisionVars);
    _g0T.resize(1,numDecisionVars);
    _W=Eigen::MatrixXd::Identity(numDecisionVars,numDecisionVars);
    _G.setZero();
    _g0T.setZero();
    _G = _A.transpose()*_A+_alpha*_W;  
    _g0T = -_A.transpose()*_B;
    _CE.setZero();
    _ce0.setZero();
    _CI=_D;
    _ci0=_F;
    w.resize(_C.cols());
    solveQP();
    _w=_w+_C*w;
    Aisfull=a*_C;
}

void BalanceCtrl::solveQP(){
    int n = w.size();
    int m = _ce0.size();
    int p = _ci0.size();

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i,j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] =  (_CI.transpose())(i,j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T(i);
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    for (int i = 0; i < n; ++i) {
        w[i] = x[i];
    }  
}