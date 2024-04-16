#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

class Z1ARM : public unitreeArm{
public:
    Z1ARM():unitreeArm(true){};
    ~Z1ARM(){};

    void moveJ(const Vec6& targetPos, double gripperPos, double speed);
    void moveL(const Vec6& targetPos, double gripperPos, double speed);
    void moveC(const Vec6& viaPoint, const Vec6& targetPos, double gripperPos, double speed);

    void printState();

private:
    double gripper_pos = 0.0;
    double joint_speed = 2.0;
    double cartesian_speed = 0.5;
};

void Z1ARM::moveJ(const Vec6& targetPos, double gripperPos, double speed) {
    std::cout << "[MOVEJ]" << std::endl;
    MoveJ(targetPos, gripperPos, speed);
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

int main() {
    std::cout << std::fixed << std::setprecision(3);
    Z1ARM arm;
    arm.sendRecvThread->start();

    arm.backToStart();

    Vec6 targetPosJ, targetPosL, viaPoint, targetPosC;
    // Example positions - should be replaced with actual target positions
    targetPosJ << 0.5, 0.1, 0.1, 0.5, -0.2, 0.5;
    targetPosL << 0, 0, 0, 0.45, -0.2, 0.2;
    viaPoint << 0, 0, 0, 0.45, 0, 0.4;
    targetPosC << 0, 0, 0, 0.45, 0.2, 0.2;

    double gripperPos = 0.0; // Example gripper position
    double speed = 2.0; // Example speed

    arm.moveJ(targetPosJ, gripperPos, speed);
    arm.moveL(targetPosL, gripperPos, 0.5); // Using different speed for MOVEL
    arm.moveC(viaPoint, targetPosC, gripperPos, 0.5); // Using different speed for MOVEC
    
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}
