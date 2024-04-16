#include "unitree_arm_sdk/control/unitreeArm.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/SVD>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>

using namespace UNITREE_ARM;

class Z1ARM : public unitreeArm{
public:
    Z1ARM():unitreeArm(false){};
    ~Z1ARM(){};

    void moveJ(const Vec6& targetPos, double speed);
    void printState();
};

void Z1ARM::moveJ(const Vec6& targetPos, double speed) {
    std::cout << "[MOVEJ]" << std::endl;
    MoveJ(targetPos, speed);
}

void Z1ARM::printState(){
    std::cout<<"------ joint State ------"<<std::endl;
    std::cout<<"qState: "<<lowstate->getQ().transpose()<<std::endl;
    std::cout<<"qdState: "<<lowstate->getQd().transpose()<<std::endl;
    std::cout<<"tauState: "<<lowstate->getTau().transpose()<<std::endl;

    std::cout<<"------ Endeffector Cartesian Posture ------"<<std::endl;
    std::cout<<"roll pitch yaw x y z"<<std::endl;
    std::cout<<lowstate->endPosture.transpose()<<std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "z1arm_node"); // 初始化ROS节点
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::cout << std::fixed << std::setprecision(3);
    Z1ARM arm;
    arm.sendRecvThread->start();
    arm.backToStart();


    /*
    T_gripper2point
    */

    Eigen::Matrix4d T_gripper2point;
    T_gripper2point << 1, 0, 0, 0.036,
                       0, 1, 0,     0,
                       0, 0, 1,     0,
                       0, 0, 0,     1;


    /*
    T_base2cam
    */

    Eigen::Matrix4d T_base2cam; // 默认初始化

    // T_base2cam << 1, 0, 0, 0.02,
    //               0, 0, 1,-0.21,
    //               0,-1, 0, 0.25,
    //               0, 0, 0,    1;

    T_base2cam <<  0.9638282388711432, -0.1058842306256739   , -0.2445887480227978 , -0.3197752258233834,
                   0.2459065446531279, -0.0006287268890128317,  0.9692933384684215 , -0.307961259791021 ,
                  -0.1027866589169517, -0.9943782652528704   ,  0.02543164055624625,  0.2319453039599461,
                                    0,                      0,                    0,                   1;

    /*
    T_base2gripper
    */

    // Eigen::Matrix4d T_base2gripper;
    // T_base2gripper << 1, 0, 0,   0,
    //                   0, 1, 0,   0,
    //                   0, 0, 1, 0.3,
    //                   0, 0, 0,   1;

    // // 首先将机械臂运动到某个位置
    // Vec6 targetPosJ;
    // targetPosJ << 0, 0, 0, 0, 0, 0.3;

    // std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待机械臂移动到位

    // arm.moveJ(targetPosJ, 1.0);    

    // std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待机械臂移动到位
    

    /*
    T_cameraToTag
    */

    // 使用相机确定tag_2的位置
    geometry_msgs::TransformStamped cameraToTagTransform;
    try {
        // 等待直到可以获取到相机到标签的变换
        cameraToTagTransform = tfBuffer.lookupTransform("camera_color_frame", "new_tag_1", ros::Time(0), ros::Duration(3.0));
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }
    
    Eigen::Matrix4d T_cameraToTag = tf2::transformToEigen(cameraToTagTransform.transform).matrix();


    /*
    T_base2gripper_new
    */

    Eigen::Matrix4d T_base2gripper_new = T_base2cam * T_cameraToTag * T_gripper2point.inverse();

    // std::cout << "T_base2gripper:\n" << T_base2gripper << "\n\n";
    // std::cout << "T_gripper2cam:\n" << T_gripper2cam << "\n\n";
    std::cout << "T_base2cam:\n" << T_base2cam << "\n\n";
    std::cout << "T_cameraToTag:\n" << T_cameraToTag << "\n\n";
    std::cout << "T_gripper2point:\n" << T_gripper2point << "\n\n";
    std::cout << "T_base2gripper_new:\n" << T_base2gripper_new << "\n\n";

    
    // 提取旋转矩阵
    Eigen::Matrix3d rotationMatrix = T_base2gripper_new.block<3, 3>(0, 0);

    // 将旋转矩阵转换为欧拉角
    Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(2,1,0);

    // 将欧拉角（弧度）转换为目标位置向量的前三个元素
    Vec6 targetPos;
    targetPos << euler_angles[2], euler_angles[1], euler_angles[0], T_base2gripper_new(0,3), T_base2gripper_new(1,3), T_base2gripper_new(2,3);

    arm.moveJ(targetPos, 1.0);

    std::cout << "targetPos:\n" << targetPos << "\n\n";
    std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待机械臂移动到位

    
    // 关机
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();

    return 0;
}