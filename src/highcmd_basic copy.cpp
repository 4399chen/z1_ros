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
#include <string>

using namespace UNITREE_ARM;

std::vector<cv::Mat> R_gripper2base, t_gripper2base;
std::vector<cv::Mat> R_target2cam, t_target2cam;

class Z1ARM : public unitreeArm{
public:
    // Z1ARM():unitreeArm(true){};
    Z1ARM():unitreeArm(false){};
    ~Z1ARM(){};

    void moveJ(const Vec6& targetPos, double gripperPos, double speed);
    void moveL(const Vec6& targetPos, double gripperPos, double speed);
    void moveC(const Vec6& viaPoint, const Vec6& targetPos, double gripperPos, double speed);

    void printState();

    void collectData(const std::vector<Vec6>& targetPositions, double gripperPos, double speed);
    
    std::vector<Vec6> loadTargetPositionsFromCSV(const std::string& filename);
private:
    double gripper_pos = 0.0;
    double joint_speed = 2.0;
    double cartesian_speed = 0.5;
};

void Z1ARM::moveJ(const Vec6& targetPos, double gripperPos, double speed) {
    std::cout << "[MOVEJ]" << std::endl;
    // MoveJ(targetPos, gripperPos, speed);
    MoveJ(targetPos, speed);
}

void Z1ARM::moveL(const Vec6& targetPos, double gripperPos, double speed) {
    std::cout << "[MOVEL]" << std::endl;
    MoveL(targetPos, gripperPos, speed);
}

void Z1ARM::moveC(const Vec6& viaPoint, const Vec6& targetPos, double gripperPos, double speed) {
    std::cout << "[MOVEC]" << std::endl;
    MoveC(viaPoint, targetPos, gripperPos, speed);
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

std::vector<Vec6> Z1ARM::loadTargetPositionsFromCSV(const std::string& filename) {
    std::vector<Vec6> targetPositions;
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        Vec6 position;
        int i = 0;
        while (std::getline(lineStream, cell, ',')) {
            position[i++] = std::stod(cell);
        }
        targetPositions.push_back(position);
    }
    return targetPositions;
}

// 数据收集函数的实现
void Z1ARM::collectData(const std::vector<Vec6>& targetPositions, double gripperPos, double speed) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(2.0).sleep(); // 等待TF缓存

    for (const auto& pos : targetPositions) {
        moveJ(pos, gripperPos, speed);
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待机械臂移动到位

        try {
            geometry_msgs::TransformStamped cameraToTagTransform = tfBuffer.lookupTransform("camera_color_frame", "tag_1", ros::Time(0), ros::Duration(1.0));
            
            Eigen::Matrix4d transformMatrix = tf2::transformToEigen(cameraToTagTransform.transform).matrix();
            cv::Mat cv_matrix(4, 4, CV_64F);

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    cv_matrix.at<double>(i, j) = transformMatrix(i, j);
                }
            }

            cv::Mat R, t;
            cv::Rodrigues(cv_matrix(cv::Rect(0, 0, 3, 3)), R); // 旋转矩阵转换为罗德里格斯向量
            t = cv_matrix(cv::Rect(3, 0, 1, 3)); // 提取平移向量

            R_target2cam.push_back(R);
            t_target2cam.push_back(t);
        } catch (const tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            continue; // 如果这次查询失败，则继续下一次循环
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "z1arm_node"); // 初始化ROS节点
    ros::NodeHandle nh;

    std::cout << std::fixed << std::setprecision(3);
    Z1ARM arm;
    arm.sendRecvThread->start();

    arm.backToStart();

    // 定义目标位置和参数
    // std::vector<Vec6> targetPositions = {
    //     (Vec6() <<  0.0,  0.4,  0.0,  0.30, -0.30,  0.40).finished(),
    //     (Vec6() <<  0.2,  0.4,  0.0,  0.30,  0.00,  0.40).finished()
    // };

    // 从CSV文件加载目标位置
    std::vector<Vec6> targetPositions = arm.loadTargetPositionsFromCSV("/home/work/catkin_ws/src/z1_ros/config/1.cvs");

    double gripperPos = 0.0; // 示例夹持器位置
    double speed = 0.5; // 示例速度

    arm.collectData(targetPositions, gripperPos, speed);

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();

    for (const auto& pos : targetPositions) {
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        rotationMatrix = Eigen::AngleAxisd(pos(0), Eigen::Vector3d::UnitX())
                       * Eigen::AngleAxisd(pos(1), Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(pos(2), Eigen::Vector3d::UnitZ());

        cv::Mat cv_R(3, 3, CV_64F);
        cv::Mat cv_t(3, 1, CV_64F);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cv_R.at<double>(i, j) = rotationMatrix(i, j);
            }
            cv_t.at<double>(i, 0) = pos(i + 3);
        }

        cv::Mat R_vec;
        cv::Rodrigues(cv_R, R_vec); // Convert rotation matrix to Rodrigues vector

        R_gripper2base.push_back(R_vec.clone());
        t_gripper2base.push_back(cv_t.clone());
    }

    // 验证R_gripper2base, t_gripper2base, R_target2cam, t_target2cam数量是否相符
    if (R_gripper2base.size() != t_gripper2base.size() || 
        R_gripper2base.size() != R_target2cam.size() || 
        R_gripper2base.size() != t_target2cam.size()) {
        std::cerr << "错误: R_gripper2base, t_gripper2base, R_target2cam, 和 t_target2cam 的数量不一致。" << std::endl;
        return -1; // 或者选择其他的错误处理方式
    } else {
        cv::Mat R_cam2gripper, t_cam2gripper;
        cv::calibrateHandEye(
            R_gripper2base, t_gripper2base, 
            R_target2cam, t_target2cam, 
            R_cam2gripper, t_cam2gripper, 
            cv::CALIB_HAND_EYE_TSAI
        );

        // 打印结果
        std::cout << "R_cam2gripper: \n" << R_cam2gripper << std::endl;
        std::cout << "t_cam2gripper: \n" << t_cam2gripper << std::endl;
    }


    return 0;
}