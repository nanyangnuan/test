/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <pinocchio/fwd.hpp> 
#include "pinocchio/parsers/urdf.hpp" 
#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include "common/mathTypes.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/unitreeRobot.h"

#ifdef COMPILE_DEBUG
    #include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

class BalanceCtrl{
public:
    BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta);
    BalanceCtrl(QuadrupedRobot *robModel, pinocchio::Data *data,pinocchio::Model *model);
    BalanceCtrl(QuadrupedRobot *robModel);
    Vec12 calF(VecInt4 contact,Vec19 q,Vec18 u,Vec3 _ddPcd,Vec3 _dWbd,Vec34 accel,Mat3 RotCToI);
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A, double epsilon = 1e-6) const;
    pinocchio::Data *_data;
    pinocchio::Model *_model;
    Vec12 _q_contact;

#ifdef COMPILE_DEBUG
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG
private:
    void Modeldata(VecInt4 contact,Vec19 q,Vec18 u,Vec34 accel);
    void EquationOfMotion();
    void NoContactMotion(Vec18 u);
    void TorqueLimit();
    void FrictionCone(VecInt4 contact);
    void DesiredTorsoPosition(Vec3 _ddPcd,Mat3 RotCToI,Vec18 u);
    void DesiredTorsoYaw(Vec3 _dWbd);
    void DesiredTorsoRoll_Pitch(Vec3 _dWbd);
    void SwingFootMotionTracking(Vec18 u,VecInt4 contact);
    void Limbkinematiclimits(Vec19 q,Vec18 u);
    // void MainBodyLimit();
    void ContactForceMin();
    void task0(Vec18 u,VecInt4 contact);
    void task1(Vec3 _ddPcd,Vec3 _dWbd,Mat3 RotCToI,Vec19 q,Vec18 u,VecInt4 contact);
    void task2(VecInt4 contact);
    Eigen::MatrixXd concatenateMatrices(Eigen::MatrixXd m1,Eigen::MatrixXd m2);
    void solution(Eigen::MatrixXd a,Eigen::MatrixXd b,Eigen::MatrixXd d,Eigen::MatrixXd f); 
    void solveQP();


    VecInt4 _contact;
    Mat6 _S;
    Mat3 _Ib;
    Vec6 _bd;
    Vec3 _g;
    Vec3 _pcb;
    Vec12  _Fprev,taumax,taumin;
    double _mass, _alpha, _beta, _fricRatio;

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
    //Eigen::Matrix<double, 5 , 3 > _fricMat;
    Mat3 _KpSwing,_KdSwing;
    Vec34 _accel;
    //更改部分
    double hmin,hmax;
    Eigen::Matrix<double, 5 , 3 > _fricMat;
    Eigen::Matrix<double, 5 , 1 > _fricMatlimit;
    int contactLegNum;
    Eigen::MatrixXd _CE, _CI,A,B,Qc,Qu,R,Js,dJs,QP_Qc,QP_Qu,QP_R,D,_C,_G,_W,_g0T;
    Eigen::VectorXd _ce0, _ci0,F,w;
    // Eigen::Matrix<double,30,30> _W; 
    // Eigen::Matrix<double,1,30> _g0T;
    Eigen::Matrix<double,30,1> _w;
    Eigen::Matrix<double,18,18> _M;//这里是质量阵
    Eigen::Matrix<double,12,18> S;//这里是力矩选取阵
    Eigen::Matrix<double,12,1> f,qcontact,FeetDesV;//这里是支撑力阵,但是这里根本就不知道支撑力多少，所以先当零么？
    Vec18 h,_u;
    Eigen::Matrix<double,18,30> MatM;
    Eigen::Matrix<double,12,18> J,dJ;//按理说应该不包括质心的，

//    // 但是如果身体坐标系的原点没有建立在质心上呢？或者说实际上，我动用的世界原点是采样器的位置，身体坐标系在质心上。这样j就是对采样器的，这样就接得上了。
    Eigen::MatrixXd Aisfull;
#ifdef COMPILE_DEBUG
    PyPlot *_testPlot;
#endif  // COMPILE_DEBUG
};

class Robot_model 
{
    public:
        pinocchio::Model model;
        Robot_model()
        {
            const std::string urdf_filename = "/home/nan/unitree/src/unitree_ros/robots/a1_description/urdf/a1.urdf";                     
            pinocchio::JointModelFreeFlyer root_joint; 
            pinocchio::urdf::buildModel(urdf_filename,root_joint,model);
        }


    private:
};


#endif  // BALANCECTRL_H