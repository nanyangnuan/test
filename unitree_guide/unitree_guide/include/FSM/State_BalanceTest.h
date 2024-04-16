/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef BALANCETEST_H
#define BALANCETEST_H

#include "FSM/FSMState.h"

class State_BalanceTest : public FSMState{
public:
    State_BalanceTest(CtrlComponents *ctrlComp);
    ~State_BalanceTest(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void calcTau();

    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    VecInt4 *_contact;

    RotMat _Rd, _RdInit;
    Vec3 _pcd, _pcdInit;
    double _kpw,_dYaw;
    Mat3 _Kpp, _Kdp, _Kdw;
    Vec3 _ddPcd, _dWbd;

    Vec12 _q, _tau;
    Vec3 _posBody, _velBody;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec34 _posFeet2BGlobal;
    Vec34 _forceFeetGlobal, _forceFeetBody,_posFeetGlobal,_velFeetGlobal;

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;

    void calRotCToI1();
    Eigen::Vector3d rotMatToEulerAngles(const Eigen::Matrix3d& rotMat);
    Mat3 RotCToI;
    Vec12 v; 
    Vec19 pin_q;
    Vec18 pin_u; 
    Vec3 euler;
};

#endif  // BALANCETEST_H