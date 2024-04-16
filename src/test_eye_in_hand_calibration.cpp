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
    T_gripper2cam
    */

    // // 定义并初始化R_gripper2cam
    // cv::Mat R_gripper2cam = (cv::Mat_<double>(3,3) << 
    //     0.9943513466955649, -0.003675407081653774, -0.1060749296841651,
    //     0.004617814746956042, 0.9999520109468534, 0.008640115179027052,
    //     0.1060380833082117, -0.009081144538448204, 0.9943206010649646);

    // // 定义并初始化t_gripper2cam
    // cv::Mat t_gripper2cam = (cv::Mat_<double>(3,1) << 
    //     0.0148126071397331,
    //     -0.01561281794945135,
    //     0.05988437336079462);

    // // 计算标签在机械臂坐标系下的位置
    // Eigen::Matrix3d eigen_R_gripper2cam;
    // Eigen::Vector3d eigen_t_gripper2cam;
    // cv::cv2eigen(R_gripper2cam, eigen_R_gripper2cam); // 转换旋转矩阵
    // cv::cv2eigen(t_gripper2cam, eigen_t_gripper2cam); // 转换平移向量

    // Eigen::Matrix4d T_gripper2cam = Eigen::Matrix4d::Identity(); // 创建 4x4 单位矩阵
    // T_gripper2cam.block<3, 3>(0, 0) = eigen_R_gripper2cam; // 设置旋转部分
    // T_gripper2cam.block<3, 1>(0, 3) = eigen_t_gripper2cam; // 设置平移部分

    Eigen::Matrix4d T_gripper2cam; // 默认初始化

    // T_gripper2cam << 1, 0, 0,  0.022,
    //                  0, 1, 0, -0.007,
    //                  0, 0, 1,  0.067,
    //                  0, 0, 0, 1;

    T_gripper2cam << 1, 0, 0,  0.016,
                     0, 1, 0, -0.012,
                     0, 0, 1,  0.06105,
                     0, 0, 0, 1;

    // T_gripper2cam <<  0.9945376719574746  , 0.027389926974716   , -0.1007204594793252  ,  0.02389474794960636 ,
    //                  -0.02736511962003101 , 0.9996241790104634  ,  0.001628178687441789, -0.009928995920360121,
    //                   0.100727202311928   , 0.001136942380497087,  0.9949134324535152  ,  0.06047415659420811 ,
    //                                     0,                    0,                     0,                     1;

    // R_cam2gripper: 
    // [0.9945376719574746, 0.027389926974716, -0.1007204594793252;
    // -0.02736511962003101, 0.9996241790104634, 0.001628178687441789;
    // 0.100727202311928, 0.001136942380497087, 0.9949134324535152]
    // t_cam2gripper: 
    // [0.02389474794960636;
    // -0.009928995920360121;
    // 0.06047415659420811]




    /*
    T_base2gripper
    */

    Eigen::Matrix4d T_base2gripper;
    T_base2gripper << 1, 0, 0,   0,
                      0, 1, 0,   0,
                      0, 0, 1, 0.3,
                      0, 0, 0,   1;

    // 首先将机械臂运动到某个位置
    Vec6 targetPosJ;
    targetPosJ << 0, 0, 0, 0, 0, 0.3;

    std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待机械臂移动到位

    arm.moveJ(targetPosJ, 1.0);    

    std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待机械臂移动到位
    

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
    T_base2tag_point
    */

    Eigen::Matrix4d T_base2tag_point = T_base2gripper * T_gripper2cam * T_cameraToTag * T_gripper2point.inverse();
    // Eigen::Matrix4d T_base2tag_point = T_base2gripper * T_cameraToTag * T_gripper2point.inverse();

    std::cout << "T_base2gripper:\n" << T_base2gripper << "\n\n";
    std::cout << "T_gripper2cam:\n" << T_gripper2cam << "\n\n";
    std::cout << "T_cameraToTag:\n" << T_cameraToTag << "\n\n";
    std::cout << "T_gripper2point:\n" << T_gripper2point << "\n\n";
    std::cout << "T_base2tag_point:\n" << T_base2tag_point << "\n\n";

    
    // 提取旋转矩阵
    Eigen::Matrix3d rotationMatrix = T_base2tag_point.block<3, 3>(0, 0);

    // 将旋转矩阵转换为欧拉角
    Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(2,1,0);

    // 将欧拉角（弧度）转换为目标位置向量的前三个元素
    Vec6 targetPos;
    targetPos << euler_angles[2], euler_angles[1], euler_angles[0], T_base2tag_point(0,3), T_base2tag_point(1,3), T_base2tag_point(2,3);

    arm.moveJ(targetPos, 1.0);

    std::cout << "targetPos:\n" << targetPos << "\n\n";
    std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待机械臂移动到位

    
    // 关机
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();

    return 0;
}