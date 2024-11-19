#include "youbot_driver/testing/YouBotArmTest.hpp"

using namespace youbot;

YouBotArmTest::YouBotArmTest():dof(5) {

  EthercatMaster::getInstance("youbot-ethercat.cfg", CONFIG_FOLDER_PATH, true);


}

YouBotArmTest::~YouBotArmTest() {

}

void YouBotArmTest::setUp() {
  // oneSec = 1000000;
}

void YouBotArmTest::tearDown() {
  //	EthercatMaster::destroy();
}

void YouBotArmTest::youBotArmTest() {
  int oneSec = 1000000;

  YouBotManipulator myArm("youbot-manipulator");
  myArm.doJointCommutation();
  std::cout << "Calibrating the arm!" << std::endl;
  myArm.calibrateManipulator();

  std::stringstream jointNameStream;
  std::vector<JointAngleSetpoint> upstretchedpose;
  std::vector<JointAngleSetpoint> foldedpose;
  JointAngleSetpoint desiredJointAngle;

  desiredJointAngle.angle = 2.56244 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.04883 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = -2.43523 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.73184 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.5 * radian;
  upstretchedpose.push_back(desiredJointAngle);

  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = -0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.2 * radian;
  foldedpose.push_back(desiredJointAngle);


  for (int i = 1; i <= dof; i++) {
    jointNameStream << "Joint_" << i << "_" << __func__;
    jointNameStream.str("");
  }

  int counter = 0;
  while (counter < 1) {
    usleep(oneSec);
    counter++;
  }

  // send torques to joints
  std::cout << "Testing torque interface" << std::endl;

  JointTorqueSetpoint desiredJointTorque;
  std::vector<JointTorqueSetpoint> commandTorques;

  // joint 0 torque
  desiredJointTorque.torque = 1.0 * si::newton_meters;
  commandTorques.push_back(desiredJointTorque);
  desiredJointTorque.torque = 0.0 * si::newton_meters;
  commandTorques.push_back(desiredJointTorque);
  desiredJointTorque.torque = 0.0 * si::newton_meters;
  commandTorques.push_back(desiredJointTorque);
  desiredJointTorque.torque = 0.0 * si::newton_meters;
  commandTorques.push_back(desiredJointTorque);
  desiredJointTorque.torque = 0.0 * si::newton_meters;
  commandTorques.push_back(desiredJointTorque);

  // move with command torques
  myArm.setJointData(commandTorques);
  counter = 0;
  while (counter < 2) {
    usleep(oneSec);
    counter++;
  }

  std::cout << "Testing torque interface done" << std::endl;

  
  // 1 sec no movement
  counter = 0;
  while (counter < 1) {
    usleep(oneSec);
    counter++;
  }
}

int main()
{
  YouBotArmTest youbot_arm_torque_test;
  youbot_arm_torque_test.setUp();
  youbot_arm_torque_test.youBotArmTest();
  youbot_arm_torque_test.tearDown();
}