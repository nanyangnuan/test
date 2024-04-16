#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_
#include <pinocchio/fwd.hpp> 
#include "pinocchio/parsers/urdf.hpp" 
class getmodel{
    public:
    pinocchio_model *model;
    pinocchio::Model a1;
    pinocchio::Model a1;
    getmodel(){
     a1=model->model;
    }

};

class pinocchio_model{
    public:
    pinocchio_model()
    {
    const std::string urdf_filename = "/home/nan/unitree/src/unitree_ros/robots/a1_description/urdf/a1.urdf";                     
    pinocchio::JointModelFreeFlyer root_joint; 
    pinocchio::urdf::buildModel(urdf_filename,root_joint,model);
    };
    pinocchio::Model model;
};


#endif