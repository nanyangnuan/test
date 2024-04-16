/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_StepTest.h"

State_StepTest::State_StepTest(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::STEPTEST, "stepTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact),
                  _phase(ctrlComp->phase){

    _gaitHeight = 0.05;

    _KpSwing = Vec3(600, 600, 200).asDiagonal();
    _KdSwing = Vec3(20, 20, 5).asDiagonal();

    _Kpp = Vec3(50, 50, 300).asDiagonal();
    _Kpw = Vec3(600, 600, 600).asDiagonal();
    _Kdp = Vec3(5, 5, 20).asDiagonal();
    _Kdw = Vec3(10, 10, 10).asDiagonal();
}

void State_StepTest::enter(){
    _pcd = _est->getPosition();
    _Rd  = _lowState->getRotMat();
    _posFeetGlobalInit = _est->getFeetPos();
    _posFeetGlobalGoal = _posFeetGlobalInit;
    _ctrlComp->setStartWave();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_StepTest::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _dYaw = _lowState->getDYaw();

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();


    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _posFeetGlobalGoal(2, i) = _posFeetGlobalInit(2, i) + (1-cos((*_phase)(i)*2*M_PI))*_gaitHeight;
            _velFeetGlobalGoal(2, i) = sin((*_phase)(i)*2*M_PI)*2*M_PI*_gaitHeight;
        }
    }

    calcTau();

    _lowCmd->setZeroGain();
    _lowCmd->setTau(_tau);
}

void State_StepTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_StepTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::STEPTEST;
    }
}

void State_StepTest::calcTau(){
        calRotCToI2();
    // _posError = _pcd - _posBody;
    // _velError = _vCmdGlobal - _velBody;
    pin_q.setZero();
    pin_q.segment(0,3)=_posBody;
    pin_q.segment(3,4)=_lowState->quaternion();
    pin_q.segment(7,12)=vec34ToVec12(_posFeetGlobal);
    pin_u.setZero();
    pin_u.segment(0,3)=_velBody;
    pin_u[5]=_dYaw;
    pin_u.segment(6,12)=vec34ToVec12(_velFeetGlobal);
    _ddPcd = _Kpp*(_pcd - RotCToI.transpose()*_posBody) + _Kdp * (Vec3(0, 0, 0) - RotCToI.transpose()*_velBody);
    _dWbd  = _Kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    
    Vec34 accel;
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            accel.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }
     _tau = - _balCtrl->calF(*_contact,pin_q,pin_u,_ddPcd,_dWbd,accel,RotCToI);
    // _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    // _q = vec34ToVec12(_lowState->getQ());
    // _tau = _robModel->getTau(_q, _forceFeetBody);
}
void State_StepTest::calRotCToI2(){

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
