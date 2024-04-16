/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_BalanceTest.h"

State_BalanceTest::State_BalanceTest(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::BALANCETEST, "balanceTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    _Kpp = Vec3(150, 150, 150).asDiagonal();
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    _kpw = 200;
    _Kdw = Vec3(30, 30, 30).asDiagonal();
}

void State_BalanceTest::enter(){
    _pcdInit = _est->getPosition();
    _pcd = _pcdInit;
    _RdInit = _lowState->getRotMat();

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_BalanceTest::run(){
    _userValue = _lowState->userValue;

    _pcd(0) = _pcdInit(0) + invNormalize(_userValue.ly, _xMin, _xMax);
    _pcd(1) = _pcdInit(1) - invNormalize(_userValue.lx, _yMin, _yMax);
    _pcd(2) = _pcdInit(2) + invNormalize(_userValue.ry, _zMin, _zMax);

    float yaw = invNormalize(_userValue.rx, _yawMin, _yawMax);
    _Rd = rpyToRotMat(0, 0, yaw)*_RdInit;

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _dYaw = _lowState->getDYaw();

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();

    _lowCmd->setStableGain();
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_q);
}

void State_BalanceTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_BalanceTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::BALANCETEST;
    }
}

void State_BalanceTest::calcTau(){
    calRotCToI1();
    pin_q.setZero();
    pin_q.segment(0,3)=_posBody;
    pin_q.segment(3,4)=_lowState->quaternion();
    pin_q.segment(7,12)=vec34ToVec12(_posFeetGlobal);
    pin_u.setZero();
    pin_u.segment(0,3)=_velBody;
    pin_u[5]=_dYaw;
    pin_u.segment(6,12)=vec34ToVec12(_velFeetGlobal);

    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    Vec34 accel;
    accel.setZero();

     _tau = - _balCtrl->calF(*_contact,pin_q,pin_u,_ddPcd,_dWbd,accel,RotCToI);
    // _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    // _q = vec34ToVec12(_lowState->getQ());
    // _tau = _robModel->getTau(_q, _forceFeetBody);
}
void State_BalanceTest::calRotCToI1(){

   Eigen::MatrixXd X(4, 4);
   X.col(3)<<  1,
               1,
               1,
               1;
   X.block(0,0,4,3)=_posFeetGlobal.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d plane_coeffs = svd.matrixV().col(3).segment(0,3);
    Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
    Eigen::Vector3d rotationAxis = plane_coeffs.cross(zAxis);
    double angle = acos(plane_coeffs.dot(zAxis) / (plane_coeffs.norm() * zAxis.norm()));
    rotationAxis =rotationAxis.normalized();
    Eigen::AngleAxisd angleAxis(angle, rotationAxis);
    RotCToI = angleAxis.toRotationMatrix();
}
