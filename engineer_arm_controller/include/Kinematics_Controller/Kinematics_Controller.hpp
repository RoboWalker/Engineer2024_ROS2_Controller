#pragma once

#define IKFAST_HAS_LIBRARY

#include "main.hpp"
#include "eigen3/Eigen/Dense"
#include "Kinematics_Controller/ikfast.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>


/**
 * @brief 机械臂正逆运动学控制器
 * @details 机械臂正逆运动学控制器，包含正逆运动学计算函数
 *          使用方法（正运动学）：
 *          1.设置当前关节角度：setNowJointAngles
 *          2.设置目标末端位置：setTargetEndEffector
 *          3.正运动学计算：forwardKinematics
 *          4.获取当前末端位置（欧拉角）：getNowEndEffectorEuler
 *          5.获取当前末端位置（旋转矩阵）：getNowEndEffectorRotationMatrix
 *          使用方法（逆运动学）：
 *          1.设置当前关节角度：setNowJointAngles
 *          2.设置目标末端位置：setTargetEndEffector
 *          3.逆运动学计算：inverseKinematics
 *          4.获取目标关节角度：getTargetJointAngles
*/
class Kinematics_Controller
{
    public:
        Kinematics_Controller();
        ~Kinematics_Controller();

        void setNowJointAngles(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);
        //目标末端位置使用旋转矩阵表示，以下函数将欧拉角转换为旋转矩阵
        void setTargetEndEffector(double x, double y, double z, double roll, double pitch, double yaw);
        Eigen::Matrix<double, 6, 1> getTargetJointAngles();
        Eigen::Matrix<double, 6, 1> getNowEndEffectorEuler();
        Eigen::Matrix4d getNowEndEffectorRotationMatrix();
        void forwardKinematics();
        //以下函数只能在正运动学计算后使用，它自动从8个逆运动学解中选择一个最合适的解
        void inverseKinematics();
        //以下函数在地矿模式下使用
        void inverseKinematicsGround();
        bool groundMode = false;

    private:
        Eigen::Matrix<double, 6, 1> now_joint_angles_;
        Eigen::Matrix<double, 6, 1> target_joint_angles_;
        Eigen::Matrix<double, 6, 1> now_end_Effector_euler_; //XYZRPY
        Eigen::Matrix4d now_end_Effector_rotation_matrix_;
        Eigen::Matrix4d target_end_Effector_rotation_matrix_;
        KDL::Tree kdl_tree;
        KDL::Chain kdl_chain;
        KDL::ChainFkSolverPos_recursive* fk_solver;
        KDL::Frame kdl_frame;
        
};

extern Kinematics_Controller kinematics_controller;