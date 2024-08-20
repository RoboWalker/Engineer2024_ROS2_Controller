#include "Kinematics_Controller/Kinematics_Controller.hpp"

Kinematics_Controller::Kinematics_Controller(){
    if (!kdl_parser::treeFromFile(ament_index_cpp::get_package_share_directory("engineer_arm_controller") + "/urdf/engineer_arm_urdf.urdf", kdl_tree)){
        throw std::runtime_error("Failed to construct kdl tree");
   }
    if(!kdl_tree.getChain("base_link", "link6", kdl_chain)){
        throw std::runtime_error("Failed to construct kdl chain");
    }
    fk_solver =  new KDL::ChainFkSolverPos_recursive(kdl_chain);
    now_joint_angles_.setZero();
    target_joint_angles_.setZero();
    now_end_Effector_euler_.setZero();
    now_end_Effector_rotation_matrix_.setZero();
    target_end_Effector_rotation_matrix_.setZero();

    setTargetEndEffector(0.5, 0.2, 0.0, M_PI, 0.0, 0.0);
}

Kinematics_Controller::~Kinematics_Controller(){
    delete fk_solver;
}

/**
 * @brief Set the now joint angles
*/
void Kinematics_Controller::setNowJointAngles(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6)
{
    now_joint_angles_(0, 0) = joint1;
    now_joint_angles_(1, 0) = joint2;
    now_joint_angles_(2, 0) = joint3;
    now_joint_angles_(3, 0) = joint4;
    now_joint_angles_(4, 0) = joint5;
    now_joint_angles_(5, 0) = joint6;
}

/**
 * @brief Set the target end effector
*/
void Kinematics_Controller::setTargetEndEffector(double x, double y, double z, double roll, double pitch, double yaw){
    //避免xyz为0导致IKFast算不出来
    //我也不知道为什么。除0错误？
    if(x == 0.0){
        x = 0.0001;
    }
    if(y == 0.0){
        y = 0.0001;
    }
    if(z == 0.0){
        z = 0.0001;
    }
    
    Eigen::Vector3d translation(x, y, z);
    Eigen::Affine3d transform = Eigen::Translation3d(translation) *
                                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    if (yaw  < 0.001f && yaw > -0.001f && pitch > 1.569f)
    {
        groundMode = true;
    }
    else{
        groundMode = false;
    }
    

    target_end_Effector_rotation_matrix_ = transform.matrix();
}

/**
 * @brief 判断解是否合法
*/
bool isValidJoint(std::vector<IkReal> solution){
    if(solution[0] > M_PI * 4.0 / 18.0) return false;
    if(solution[0] < -M_PI * 5.0 / 18.0) return false;
    if(solution[1] > 0.24) return false;
    if (solution[2] < 0.0) return false;
    if(solution[4] > M_PI / 2) return false;
    if(solution[4] < -M_PI / 2) return false;
    return true;
}


//角度归一化
double NormAngle(double angle){
    float a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

/**
 * @brief Inverse kinematics based on IKFAST
*/
void Kinematics_Controller::inverseKinematics(){
    ikfast::IkSolutionList<IkReal> solutions;
    IkReal eerot[9],eetrans[3];
    eerot[0] = target_end_Effector_rotation_matrix_(0, 0);
    eerot[1] = target_end_Effector_rotation_matrix_(0, 1);
    eerot[2] = target_end_Effector_rotation_matrix_(0, 2);
    eetrans[0] = target_end_Effector_rotation_matrix_(0, 3);
    eerot[3] = target_end_Effector_rotation_matrix_(1, 0);
    eerot[4] = target_end_Effector_rotation_matrix_(1, 1);
    eerot[5] = target_end_Effector_rotation_matrix_(1, 2);
    eetrans[1] = target_end_Effector_rotation_matrix_(1, 3);
    eerot[6] = target_end_Effector_rotation_matrix_(2, 0);
    eerot[7] = target_end_Effector_rotation_matrix_(2, 1);
    eerot[8] = target_end_Effector_rotation_matrix_(2, 2);
    eetrans[2] = target_end_Effector_rotation_matrix_(2, 3);

    //RCLCPP_INFO(node->get_logger(), "x: %f, y: %f, z: %f", eetrans[0], eetrans[1], eetrans[2]);

    // bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);
    // if(!bSuccess){
    //     std::cout << "Inverse kinematics failed." << std::endl;
    //     //print the target_end_Effector_rotation_matrix_
    //     std::cout << "target_end_Effector_rotation_matrix_:" << std::endl;
    //     std::cout << target_end_Effector_rotation_matrix_ << std::endl;
    //     return;
    // }
    //如果计算不成功，则x和z以不同比例往里收，直至成功
    int count = 0;
    bool bSuccess = false;
    while (!bSuccess)
    {
        bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

        if (bSuccess)
        {
            std::vector<IkReal> solvalues(GetNumJoints());
            std::vector<IkReal> closestSolution = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            double minError = std::numeric_limits<double>::max();

            for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
                const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
                std::vector<IkReal> vsolfree(sol.GetFree().size());
                sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                // 计算当前解与当前位置的平方误差
                double error = 0.0;
                for (std::size_t j = 0; j < solvalues.size(); ++j) {
                    double jointError = NormAngle(solvalues[j] - now_joint_angles_(j));
                    error += jointError * jointError;
                }

                // 如果当前解的误差更小，则更新最小误差和最接近的解
                if (error < minError && isValidJoint(solvalues)) {
                    minError = error;
                    closestSolution = solvalues;
                }
            }

            if (closestSolution[0] < 4.0)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    target_joint_angles_(i, 0) = closestSolution[i];
                }
            }
            else{
                bSuccess = false;
                std::cout << "No valid solutions!" << std::endl;
            }
        }
        
        if (!bSuccess)
        {
            if(fabs(eetrans[2] / eetrans[0]) > 0.6){
                if (eetrans[0] < 0.3)
                {
                    eetrans[2] *= 0.8;
                }
                else if (eetrans[0] < 0.35)
                {
                    eetrans[0] *= 0.9;
                }
                else if (eetrans[0] < 0.4)
                {
                    eetrans[0] *= 0.95;
                }
                else
                {
                    eetrans[2] *= 0.98;
                }
            }
            else{
                eetrans[0] *= 0.995;
            }

            if (eetrans[1] < -0.2)
            {
                eetrans[1] *= 0.98;
            }
        }
        

        count ++;
        if (count > 50)
        {
            std::cerr << "Inverse kinematics failed." << std::endl;
            return;
        }
        
    }
    
    
    

    // // 输出最接近的解
    // std::cout << "Closest Solution: ";
    // for (const auto& jointValue : closestSolution) {
    //     std::cout << jointValue << ", ";
    // }
    // std::cout << std::endl;
}

/**
 * @brief Inverse kinematics for ground mode
*/
void Kinematics_Controller::inverseKinematicsGround(){
    //大小臂长度
    double lf = 0.325;
    double lt = 0.240;
    //计算虚拟臂长度
    double x = target_end_Effector_rotation_matrix_(0, 3) * 0.85;
    double y = target_end_Effector_rotation_matrix_(1, 3);
    double z = target_end_Effector_rotation_matrix_(2, 3);
    double lVirtual = sqrt(x*x + y*y + z*z);
    float sigma, alpha, beta, theta;
    if (lVirtual > lf + lt - 0.002)
    {
        return; //无解
    }
    if (x < 0.15)
    {
        return;
    }
    
    sigma = asin(y/lVirtual);
    alpha = acos((lVirtual * lVirtual + lf * lf - lt * lt) / (2 * lf * lVirtual));
    beta = acos((lf*lf +lt*lt - lVirtual * lVirtual) / (2 * lt * lf));
    theta = asin(z/x);

    if (isnan(sigma) || isnan(alpha) || isnan(beta) || isnan(theta))
    {
        return;
    }

    target_joint_angles_(0, 0) = theta;
    target_joint_angles_(1, 0) = -alpha - sigma;
    target_joint_angles_(2, 0) = -beta + M_PI;
    target_joint_angles_(3, 0) = -target_joint_angles_(1, 0) - target_joint_angles_(2, 0);
    target_joint_angles_(4, 0) = theta + M_PI / 2;
}

/**
 * @brief Forward kinematics based on KDL
*/
void Kinematics_Controller::forwardKinematics(){
    //define kdl_frame_

    KDL::JntArray jnt_pos_in(kdl_chain.getNrOfJoints());
    KDL::JntArray jnt_pos_out(kdl_chain.getNrOfJoints());
    for (size_t i = 0; i < 6; i++)
    {
        jnt_pos_in(i) = now_joint_angles_(i, 0);
    }
    fk_solver->JntToCart(jnt_pos_in, kdl_frame);
    kdl_frame.M.GetRPY(now_end_Effector_euler_(3, 0), now_end_Effector_euler_(4, 0), now_end_Effector_euler_(5, 0));
    //get XYZ
    now_end_Effector_euler_(0, 0) = kdl_frame.p.x();
    now_end_Effector_euler_(1, 0) = kdl_frame.p.y();
    now_end_Effector_euler_(2, 0) = kdl_frame.p.z();
    //get rotation matrix
    Eigen::Vector3d translation(kdl_frame.p.x(), kdl_frame.p.y(), kdl_frame.p.z());
    Eigen::Affine3d transform = Eigen::Translation3d(translation) *
                                Eigen::AngleAxisd(now_end_Effector_euler_(5, 0), Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(now_end_Effector_euler_(4, 0), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(now_end_Effector_euler_(3, 0), Eigen::Vector3d::UnitX());
    now_end_Effector_rotation_matrix_ = transform.matrix();
    
    // now_end_Effector_rotation_matrix_ = kdl_frame.M.data;
    // now_end_Effector_euler_ = kdl_frame.M.GetEulerZYX(2, 1, 0);
}

/**
 * @brief Get the target joint angles
*/
Eigen::Matrix<double, 6, 1> Kinematics_Controller::getTargetJointAngles(){
    return target_joint_angles_;
}

/**
 * @brief Get the now end effector euler
*/
Eigen::Matrix<double, 6, 1> Kinematics_Controller::getNowEndEffectorEuler(){
    return now_end_Effector_euler_;
}

/**
 * @brief Get the now end effector rotation matrix
*/
Eigen::Matrix4d Kinematics_Controller::getNowEndEffectorRotationMatrix(){
    return now_end_Effector_rotation_matrix_;
}

Kinematics_Controller kinematics_controller;
